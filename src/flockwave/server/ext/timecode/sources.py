"""Timecode sources. A source decodes timecode frames from one kind of input
and hands each of them, stamped with the server wall-clock time at which the
frame started, to a callback. Adding an input (MIDI timecode, Art-Net) means
adding a source function and registering it in ``SOURCES``.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from logging import Logger
from time import time
from typing import Any

import numpy as np
import trio
import trio.socket

from .clock import TimecodeFrame
from .ltc import LtcDecoder, LtcFrame
from .timecode import FrameRate, Timecode

__all__ = ("SOURCES", "TimecodeSource")

FrameCallback = Callable[[TimecodeFrame], None]

TimecodeSource = Callable[
    [dict[str, Any], FrameRate, Logger, FrameCallback], Awaitable[None]
]
"""Runs until cancelled, calling the callback with each received frame.
Receives the extension configuration, the configured frame rate and a
logger."""

#: The audio input is reopened when no samples arrived for this long.
AUDIO_STALL_TIMEOUT = 5.0

#: Delay before the audio input is opened again after a failure.
AUDIO_RETRY_DELAY = 5.0

#: Duration of the audio blocks handed over by the audio input. Frames are
#: stamped by their position in the stream, so this only sets how soon a
#: frame is seen, not how precisely it is timed.
AUDIO_BLOCK_DURATION = 0.02


async def run_udp_source(
    configuration: dict[str, Any], rate: FrameRate, log: Logger, on_frame: FrameCallback
) -> None:
    """Receives one ``HH:MM:SS:FF`` timecode per UDP datagram, e.g. from a
    hardware LTC reader. Frames are stamped with their arrival time."""
    port = int(configuration.get("port", 5005))
    with trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM) as sock:
        await sock.bind(("0.0.0.0", port))
        log.info(f"Listening for timecode datagrams on UDP port {port}")
        while True:
            data, _ = await sock.recvfrom(64)
            wall_time = time()
            try:
                timecode = Timecode.parse(data.decode("ascii", "ignore"))
            except ValueError:
                continue
            on_frame(TimecodeFrame(timecode, rate, wall_time))


async def run_ltc_audio_source(
    configuration: dict[str, Any], rate: FrameRate, log: Logger, on_frame: FrameCallback
) -> None:
    """Decodes LTC from one channel of an audio input. Frames are stamped
    with the time their first transition reached the ADC, as reported by
    PortAudio."""
    try:
        import sounddevice
    except OSError as ex:
        raise RuntimeError(
            f"LTC audio input needs the PortAudio library (libportaudio2): {ex}"
        ) from None

    device = configuration.get("device")
    channel = int(configuration.get("channel", 1))
    sample_rate = configuration.get("sample_rate")

    while True:
        try:
            await _decode_audio_input(
                sounddevice, device, channel, sample_rate, rate, log, on_frame
            )
        except (sounddevice.PortAudioError, ValueError) as ex:
            log.warning(f"LTC audio input failed: {ex}")
        await trio.sleep(AUDIO_RETRY_DELAY)


async def _decode_audio_input(
    sounddevice,
    device: int | str | None,
    channel: int,
    sample_rate: float | None,
    rate: FrameRate,
    log: Logger,
    on_frame: FrameCallback,
) -> None:
    info = sounddevice.query_devices(device, "input")
    if not 1 <= channel <= info["max_input_channels"]:
        raise ValueError(
            f"{info['name']!r} has no input channel {channel} "
            f"(channels are numbered from 1)"
        )
    sample_rate = float(sample_rate or info["default_samplerate"])
    if sounddevice.query_hostapis(info["hostapi"])["name"] == "PulseAudio":
        log.warning(
            f"{info['name']!r} is a PulseAudio device; PortAudio times its "
            "input only to tens of milliseconds, choose an ALSA device"
        )

    token = trio.lowlevel.current_trio_token()
    sender, receiver = trio.open_memory_channel[tuple[np.ndarray, float, bool]](64)
    dropped = False

    def deliver(block: tuple[np.ndarray, float, bool]) -> None:
        nonlocal dropped
        try:
            sender.send_nowait(block)
        except trio.WouldBlock:
            dropped = True

    def callback(indata, frame_count, time_info, status) -> None:
        now = time()
        adc_time = time_info.inputBufferAdcTime
        if adc_time > 0:
            first_sample_time = now - (time_info.currentTime - adc_time)
        else:
            first_sample_time = now - frame_count / sample_rate
        token.run_sync_soon(
            deliver,
            (
                indata[:, channel - 1].copy(),
                first_sample_time,
                bool(status.input_overflow),
            ),
        )

    decoder = LtcDecoder(sample_rate)
    check = _FrameRateCheck(rate, sample_rate, log)
    with sounddevice.InputStream(
        device=info["index"],
        channels=channel,
        samplerate=sample_rate,
        blocksize=round(sample_rate * AUDIO_BLOCK_DURATION),
        dtype="float32",
        callback=callback,
    ):
        log.info(
            f"Decoding {rate} fps LTC from channel {channel} of {info['name']!r} "
            f"at {sample_rate:g} Hz"
        )
        while True:
            with trio.move_on_after(AUDIO_STALL_TIMEOUT) as stall:
                samples, first_sample_time, overflow = await receiver.receive()
            if stall.cancelled_caught:
                log.warning(f"No audio from {info['name']!r}, reopening the input")
                return

            if overflow or dropped:
                dropped = False
                decoder.reset()
            first_sample = decoder.position
            for frame in decoder.decode(samples):
                check.observe(frame)
                wall_time = (
                    first_sample_time
                    + (frame.start_sample - first_sample) / sample_rate
                )
                on_frame(TimecodeFrame(frame.timecode, rate, wall_time))


class _FrameRateCheck:
    """Warns once when the decoded LTC does not run at the configured rate;
    such a stream would be read with the wrong frame arithmetic."""

    def __init__(self, rate: FrameRate, sample_rate: float, log: Logger):
        self._rate = rate
        self._sample_rate = sample_rate
        self._log = log
        self._previous: LtcFrame | None = None
        self._warned = False

    def observe(self, frame: LtcFrame) -> None:
        previous, self._previous = self._previous, frame
        if self._warned or previous is None:
            return
        rate = self._rate
        if (
            frame.timecode.to_frame_count(rate) - previous.timecode.to_frame_count(rate)
            != 1
            or frame.start_sample <= previous.start_sample
        ):
            return
        fps = self._sample_rate / (frame.start_sample - previous.start_sample)
        if abs(fps - rate.fps) > 0.5 or frame.drop_frame != rate.drop_frame:
            kind = "drop-frame " if frame.drop_frame else ""
            self._log.warning(
                f"The LTC input carries {kind}timecode at about {fps:.2f} fps but "
                f"the timecode extension is configured for {rate} fps"
            )
            self._warned = True


SOURCES: dict[str, TimecodeSource] = {
    "ltc_audio": run_ltc_audio_source,
    "udp": run_udp_source,
}
