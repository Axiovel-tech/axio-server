"""Tests for the timecode clock and the LTC audio source."""

import logging
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import trio

from flockwave.server.ext.timecode.clock import TimecodeClock, TimecodeFrame
from flockwave.server.ext.timecode.ltc import LtcEncoder
from flockwave.server.ext.timecode.sources import run_ltc_audio_source
from flockwave.server.ext.timecode.timecode import FrameRate, Timecode

FPS_25 = FrameRate(25)
DF = FrameRate(30, drop_frame=True)
T0 = 1_752_000_000.0


def _frame(seconds: float, wall_time: float, rate: FrameRate = FPS_25):
    count = round(seconds * rate.fps)
    return TimecodeFrame(Timecode.from_frame_count(count, rate), rate, wall_time)


class _Signals:
    def __init__(self, clock):
        self.events = []
        for name in ("started", "stopped", "changed"):
            getattr(clock, name).connect(self._recorder(name), sender=clock, weak=False)

    def _recorder(self, name):
        return lambda sender, **kwds: self.events.append((name, kwds.get("delta")))

    def names(self):
        return [name for name, _ in self.events]


def test_clock_follows_least_delayed_frames():
    """Frames arrive with a jittery latency; the clock settles on the least
    delayed ones and only re-anchors when that estimate moves."""
    rng = np.random.default_rng(2)
    clock = TimecodeClock(FPS_25)
    signals = _Signals(clock)
    tc_zero = T0 - 3600.0

    for i in range(100):
        latency = 0.002 + rng.exponential(0.03)
        clock.feed(_frame(3600 + i / 25, T0 + i / 25 + latency))

    assert clock.running
    assert clock.ticks_per_second == 25
    assert signals.names()[0] == "started"
    assert all(name == "changed" for name in signals.names()[1:])
    error = clock.ticks_given_time(T0 + 10) / 25 - (T0 + 10 - tc_zero)
    assert -0.008 < error < -0.001


def test_clock_drop_frame_runs_at_real_time():
    clock = TimecodeClock(DF)
    clock.feed(TimecodeFrame(Timecode(1, 0, 0, 0), DF, T0))
    assert clock.ticks_given_time(T0 + 1) / clock.ticks_per_second == pytest.approx(
        107_892 / DF.fps + 1
    )


def test_clock_jump_and_drift():
    clock = TimecodeClock(FPS_25)
    signals = _Signals(clock)
    for i in range(50):
        clock.feed(_frame(100 + i / 25, T0 + i / 25))
    assert signals.names() == ["started"]

    # The desk scrubs back ten seconds
    clock.feed(_frame(90 + 50 / 25, T0 + 50 / 25))
    assert signals.events[-1][0] == "changed"
    assert signals.events[-1][1] == pytest.approx(-250)

    # The source runs 0.1 % slow: re-anchored in steps just over 5 ms
    signals.events.clear()
    for i in range(51, 1000):
        clock.feed(_frame(90 + i / 25, T0 + i / 25 * 1.001))
    deltas = [delta for name, delta in signals.events]
    assert signals.names() == ["changed"] * len(deltas)
    assert len(deltas) >= 6
    assert all(-0.15 < delta < -0.125 for delta in deltas)


def test_clock_stops_and_freezes_when_frames_stop():
    clock = TimecodeClock(FPS_25, freshness=1.0)
    signals = _Signals(clock)
    clock.feed(_frame(100, T0))
    clock.feed(_frame(100.04, T0 + 0.04))

    clock.check_freshness(T0 + 0.5)
    assert clock.running
    clock.check_freshness(T0 + 1.5)
    assert not clock.running
    assert signals.names() == ["started", "stopped"]
    assert clock.ticks_given_time(T0 + 5) == pytest.approx(100.04 * 25)

    clock.feed(_frame(200, T0 + 2))
    assert clock.running
    assert signals.names()[-1] == "started"
    assert clock.ticks_given_time(T0 + 3) == pytest.approx(201 * 25)


class _FakeInputStream:
    """Stands in for ``sounddevice.InputStream``: delivers a two-channel
    signal with LTC on the second channel from a thread. Its stream clock
    runs 1000 s behind the wall clock and each callback comes 3 ms after the
    block's last sample was captured, like a real input."""

    BLOCK = 512
    WALL_AT_FIRST_SAMPLE = T0

    def __init__(self, signal, wall_clock, *, channels, samplerate, callback, **kwds):
        assert channels == 2
        self._signal = signal
        self._wall_clock = wall_clock
        self._rate = samplerate
        self._callback = callback
        self._stop = threading.Event()

    def __enter__(self):
        self._thread = threading.Thread(target=self._run)
        self._thread.start()
        return self

    def __exit__(self, *args):
        self._stop.set()
        self._thread.join()

    def _run(self):
        status = SimpleNamespace(input_overflow=False)
        for start in range(0, len(self._signal), self.BLOCK):
            if self._stop.wait(0.001):
                return
            block = self._signal[start : start + self.BLOCK]
            first_sample = self.WALL_AT_FIRST_SAMPLE + start / self._rate
            self._wall_clock[0] = first_sample + len(block) / self._rate + 0.003
            adc_time = first_sample - 1000.0
            time_info = SimpleNamespace(
                currentTime=self._wall_clock[0] - 1000.0, inputBufferAdcTime=adc_time
            )
            indata = np.stack([np.zeros_like(block), block], axis=1)
            self._callback(indata, len(block), time_info, status)


async def test_ltc_audio_source_stamps_frames_with_adc_time(monkeypatch):
    sample_rate = 48_000
    encoder = LtcEncoder(FPS_25, sample_rate, Timecode(1, 0, 0, 0), start_sample=100)
    signal = encoder.render(0, sample_rate)
    wall_clock = [0.0]
    device = {
        "index": 3,
        "name": "fake",
        "max_input_channels": 2,
        "default_samplerate": sample_rate,
    }
    fake = SimpleNamespace(
        query_devices=lambda device_, kind: device,
        InputStream=lambda **kwds: _FakeInputStream(signal, wall_clock, **kwds),
        PortAudioError=OSError,
    )
    monkeypatch.setitem(sys.modules, "sounddevice", fake)
    monkeypatch.setattr(
        "flockwave.server.ext.timecode.sources.time", lambda: wall_clock[0]
    )

    frames = []
    log = logging.getLogger("test_ext_timecode")
    with trio.move_on_after(1):
        await run_ltc_audio_source(
            {"channel": 2},
            FPS_25,
            log,
            frames.append,
        )

    assert len(frames) == 23
    for frame in frames:
        index = round((frame.wall_time - T0) * FPS_25.fps)
        assert frame.timecode == encoder.timecode_of_frame(index)
        expected = T0 + encoder.frame_start(index) / sample_rate
        assert frame.wall_time == pytest.approx(expected, abs=1e-5)
