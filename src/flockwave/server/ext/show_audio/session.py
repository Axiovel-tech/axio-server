"""One output stream of the show music: the audio-thread side that puts a
track's sample 0 on the DAC at a wall-clock time.
"""

from __future__ import annotations

from collections.abc import Callable
from time import time
from typing import Any, Protocol

import numpy as np
import trio

from .timing import frames_until, render_block
from .track import AudioTrack

__all__ = (
    "ExtraOutput",
    "PlaybackSession",
    "Stream",
    "open_output_stream",
)

STATS_BLOCKS = 32
"""Number of callback blocks whose timing errors are summarised at the start
and at the end of the track."""

OUTPUT_LATENCY = 0.2
"""Output buffering requested from PortAudio, in seconds. Generous so that the
Python callback survives the event loop holding the GIL for a while; the
schedule compensates it through the DAC timestamps.
"""


class ExtraOutput(Protocol):
    """Writes one extra output channel of the music stream (e.g. LTC) from its
    own lead-in before show T0 until the end of the track.
    """

    channel: int
    """Index of the output channel to write; above the track's channels."""

    lead_s: float
    """How long before show T0 the channel needs the stream running."""

    def fill(self, out: np.ndarray, show_time: float, sample_rate: int) -> None:
        """Fills one block of the channel whose first frame reaches the DAC at
        ``show_time`` seconds on the show clock. Runs on the audio thread, so
        it must not block, lock or allocate buffers.
        """
        ...


class Stream(Protocol):
    description: str

    async def aclose(self) -> None: ...


class PlaybackSession:
    """One output stream that puts a track's sample 0 on the DAC at a given
    wall-clock time and then plays the track continuously.

    `callback()` runs on the audio thread. The two threads share plain
    attributes only: the event loop may move `start_time` and `sample0_time`
    until the track has started; the callback owns the playback position and
    the timing statistics.
    """

    track: AudioTrack
    start_time: float
    """Show T0 as a UNIX timestamp."""

    sample0_time: float
    """When sample 0 must reach the DAC, as a UNIX timestamp."""

    extra: ExtraOutput | None
    underflows: int
    silenced: bool

    def __init__(
        self,
        track: AudioTrack,
        start_time: float,
        sample0_time: float,
        *,
        extra: ExtraOutput | None = None,
        clock: Callable[[], float] = time,
    ):
        self.track = track
        self.start_time = start_time
        self.sample0_time = sample0_time
        self.extra = extra
        self.underflows = 0
        self.silenced = False

        self._clock = clock
        self._position = -1
        self._joined_at = -1
        self._blocks = 0
        self._first_errors = np.zeros(STATS_BLOCKS)
        self._recent_errors = np.zeros(STATS_BLOCKS)

    @property
    def channels(self) -> int:
        """Number of channels of the output stream."""
        if self.extra is None:
            return self.track.channels
        return max(self.track.channels, self.extra.channel + 1)

    @property
    def finished(self) -> bool:
        return self._position >= self.track.samples.shape[0]

    @property
    def position(self) -> float:
        """Playback position in the track, in seconds; negative before the
        start.
        """
        return self._position / self.track.sample_rate

    @property
    def joined_at(self) -> float | None:
        """Position in the track where playback began, in seconds; ``None``
        before the start.
        """
        if self._joined_at < 0:
            return None
        return self._joined_at / self.track.sample_rate

    @property
    def started(self) -> bool:
        return self._position >= 0

    def start_error(self) -> float | None:
        """Median offset between where the DAC timestamps put the track and
        where it is due over the first blocks, in seconds (positive = late);
        ``None`` until enough blocks have played.
        """
        if self._blocks < STATS_BLOCKS:
            return None
        return float(np.median(self._first_errors))

    def end_drift(self) -> float | None:
        """Same as `start_error()` over the most recent blocks."""
        count = min(self._blocks, STATS_BLOCKS)
        if count == 0:
            return None
        return float(np.median(self._recent_errors[:count]))

    def callback(self, outdata: np.ndarray, frames: int, time_info: Any, status: Any):
        if status.output_underflow:
            self.underflows += 1

        # PortAudio reports the DAC time on the stream's own clock; its offset
        # from the stream's current time is the remaining output latency.
        dac_time = self._clock() + (
            time_info.outputBufferDacTime - time_info.currentTime
        )
        sample_rate = self.track.sample_rate
        samples = self.track.samples

        if self.silenced:
            outdata.fill(0)
            return

        position = self._position
        lead = 0
        if position < 0:
            lead = frames_until(self.sample0_time, dac_time, sample_rate)
            if lead >= frames:
                outdata.fill(0)
                self._fill_extra(outdata, dac_time)
                return
            if lead < 0:
                position, lead = -lead, 0
            else:
                position = 0
            self._joined_at = position

        self._position = render_block(outdata, samples, position, lead)
        if position < samples.shape[0]:
            error = (
                dac_time
                + lead / sample_rate
                - self.sample0_time
                - position / sample_rate
            )
            if self._blocks < STATS_BLOCKS:
                self._first_errors[self._blocks] = error
            self._recent_errors[self._blocks % STATS_BLOCKS] = error
            self._blocks += 1

        self._fill_extra(outdata, dac_time)

    def _fill_extra(self, outdata: np.ndarray, dac_time: float) -> None:
        extra = self.extra
        if extra is not None:
            extra.fill(
                outdata[:, extra.channel],
                dac_time - self.start_time,
                self.track.sample_rate,
            )


async def open_output_stream(
    session: PlaybackSession, device: str | int | None
) -> Stream:
    """Opens and starts a PortAudio output stream for a session."""
    # Shielded so that a cancellation cannot orphan a stream that the worker
    # thread has already started; the caller takes ownership before its next
    # checkpoint.
    with trio.CancelScope(shield=True):
        return await trio.to_thread.run_sync(_start_portaudio_stream, session, device)


def _start_portaudio_stream(
    session: PlaybackSession, device: str | int | None
) -> _PortAudioStream:
    try:
        import sounddevice
    except OSError as ex:
        raise RuntimeError("PortAudio library not found; install libportaudio2") from ex

    info = sounddevice.query_devices(device, "output")
    host_api = sounddevice.query_hostapis(info["hostapi"])["name"]
    if host_api == "PulseAudio":
        # Measured on PipeWire: its DAC timestamps put the audio ~50 ms early
        raise RuntimeError(
            f"{info['name']!r} is a PulseAudio host API device, whose DAC "
            "timestamps are inaccurate; use an ALSA device such as 'default'"
        )

    stream = sounddevice.OutputStream(
        samplerate=session.track.sample_rate,
        channels=session.channels,
        dtype="float32",
        device=info["index"],
        latency=OUTPUT_LATENCY,
        callback=session.callback,
    )
    try:
        stream.start()
    except BaseException:
        stream.close()
        raise
    return _PortAudioStream(stream, f"{info['name']!r} ({host_api})")


class _PortAudioStream:
    def __init__(self, stream, description: str):
        self._stream = stream
        self.description = description

    async def aclose(self) -> None:
        await trio.to_thread.run_sync(self._stream.close)
