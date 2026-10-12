"""The ``mtc`` clock, which follows the frames decoded by a timecode source."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from time import time

from flockwave.server.model import ClockBase

from .timecode import FrameRate, Timecode

__all__ = ("TimecodeClock", "TimecodeFrame")

#: A frame further than this from the one before it (after accounting for
#: the wall time between them) is a jump in the timecode, not jitter.
JUMP_TOLERANCE_FRAMES = 3

#: The clock is re-anchored when its estimate drifts further than this.
DRIFT_TOLERANCE = 0.005

#: Length of the window the minimum-latency estimate is taken over.
ESTIMATE_WINDOW = 2.0


@dataclass(frozen=True)
class TimecodeFrame:
    """One frame received from a timecode source."""

    timecode: Timecode
    rate: FrameRate
    wall_time: float
    """Server wall-clock time (``time.time()``) at which the frame started."""

    @property
    def seconds(self) -> float:
        """Real seconds between 00:00:00:00 and the start of this frame."""
        return self.timecode.to_frame_count(self.rate) / self.rate.fps


class TimecodeClock(ClockBase):
    """Clock that follows an external timecode. Ticks are frames since
    00:00:00:00 at the timecode's real frame rate.

    Every frame gives a sample of the wall time at which the timecode read
    zero; sources deliver frames late by a varying latency, so the clock
    uses the earliest of these over a short window. The clock is running
    while frames keep arriving; it stops (and freezes at the last frame)
    once none arrived for ``freshness`` seconds, which the owner detects by
    calling :meth:`check_freshness` periodically.
    """

    def __init__(self, rate: FrameRate, freshness: float = 1.0):
        super().__init__(id="mtc", epoch=None)
        self._rate = rate
        self._freshness = float(freshness)
        self._running = False
        self._last_frame: TimecodeFrame | None = None
        self._zero_time = 0.0
        self._samples: deque[tuple[float, float]] = deque()

    @property
    def last_frame(self) -> TimecodeFrame | None:
        return self._last_frame

    @property
    def running(self) -> bool:
        return self._running

    @property
    def ticks_per_second(self) -> float:
        return self._rate.fps

    def ticks_given_time(self, now: float) -> float:
        if self._last_frame is None:
            return 0.0
        if not self._running:
            return self._last_frame.seconds * self._rate.fps
        return (now - self._zero_time) * self._rate.fps

    def check_freshness(self, now: float | None = None) -> None:
        """Stops the clock if no frame arrived within the freshness window."""
        if now is None:
            now = time()
        if (
            self._running
            and self._last_frame is not None
            and now - self._last_frame.wall_time > self._freshness
        ):
            self._running = False
            self.stopped.send(self)

    def feed(self, frame: TimecodeFrame) -> None:
        """Processes a frame received from the timecode source."""
        if not frame.timecode.is_valid_at(frame.rate):
            return

        previous = self._last_frame
        self._last_frame = frame
        zero_time = frame.wall_time - frame.seconds

        if not self._running:
            self._rate = frame.rate
            self._restart_estimate(frame, zero_time)
            self._running = True
            self.started.send(self)
            return

        assert previous is not None
        step = (frame.seconds - previous.seconds) - (
            frame.wall_time - previous.wall_time
        )
        if (
            frame.rate != self._rate
            or abs(step) * self._rate.fps > JUMP_TOLERANCE_FRAMES
        ):
            self._rate = frame.rate
            self._move_zero_time_to(zero_time)
            self._restart_estimate(frame, zero_time)
            return

        self._samples.append((frame.wall_time, zero_time))
        while self._samples[0][0] < frame.wall_time - ESTIMATE_WINDOW:
            self._samples.popleft()
        estimate = min(sample for _, sample in self._samples)
        if abs(estimate - self._zero_time) > DRIFT_TOLERANCE:
            self._move_zero_time_to(estimate)

    def _move_zero_time_to(self, zero_time: float) -> None:
        delta = (self._zero_time - zero_time) * self._rate.fps
        self._zero_time = zero_time
        self.changed.send(self, delta=delta)

    def _restart_estimate(self, frame: TimecodeFrame, zero_time: float) -> None:
        self._zero_time = zero_time
        self._samples.clear()
        self._samples.append((frame.wall_time, zero_time))
