"""SMPTE timecode labels and the frame rates they are counted at."""

from __future__ import annotations

import re
from dataclasses import dataclass

__all__ = ("FrameRate", "SUPPORTED_RATES", "Timecode")


@dataclass(frozen=True)
class FrameRate:
    """A timecode frame rate: the number of frames labelled per second and
    whether the labels skip frame numbers to follow the 1000/1001 NTSC rate
    (drop-frame timecode)."""

    nominal: int
    drop_frame: bool = False

    @classmethod
    def from_fps(cls, fps: float | str) -> FrameRate:
        """Returns the supported rate closest to ``fps`` (24, 25, 29.97 or 30;
        29.97 is drop-frame)."""
        value = float(fps)
        for rate in SUPPORTED_RATES:
            if abs(rate.fps - value) < 0.01:
                return rate
        raise ValueError(f"Unsupported timecode frame rate: {fps}")

    @property
    def fps(self) -> float:
        """Real frames per second."""
        return self.nominal * 1000 / 1001 if self.drop_frame else float(self.nominal)

    @property
    def frames_per_day(self) -> int:
        return Timecode(24, 0, 0, 0).to_frame_count(self)

    def __str__(self) -> str:
        return "29.97df" if self.drop_frame else str(self.nominal)


SUPPORTED_RATES = (FrameRate(24), FrameRate(25), FrameRate(30, True), FrameRate(30))

_TIMECODE_PATTERN = re.compile(r"^(\d{1,2}):(\d{2}):(\d{2})[:;.,](\d{2})$")

#: Drop-frame labels skip frame numbers 0 and 1 at the start of every minute
#: except each tenth one.
_DROPPED_PER_MINUTE = 2
_DF_FRAMES_PER_MINUTE = 30 * 60 - _DROPPED_PER_MINUTE
_DF_FRAMES_PER_10_MINUTES = 10 * _DF_FRAMES_PER_MINUTE + _DROPPED_PER_MINUTE


@dataclass(frozen=True)
class Timecode:
    """A timecode label, ``HH:MM:SS:FF``."""

    hours: int
    minutes: int
    seconds: int
    frames: int

    @classmethod
    def parse(cls, text: str) -> Timecode:
        """Parses ``HH:MM:SS:FF``; ``;``, ``.`` or ``,`` are accepted before
        the frame number."""
        match = _TIMECODE_PATTERN.match(text.strip())
        if not match:
            raise ValueError(f"Invalid timecode: {text!r}")
        return cls(*(int(part) for part in match.groups()))

    @classmethod
    def from_frame_count(cls, count: int, rate: FrameRate) -> Timecode:
        """The label of the frame ``count`` frames after 00:00:00:00, wrapping
        around at 24 hours."""
        count %= rate.frames_per_day
        if rate.drop_frame:
            tens, rest = divmod(count, _DF_FRAMES_PER_10_MINUTES)
            count += 9 * _DROPPED_PER_MINUTE * tens
            if rest > _DROPPED_PER_MINUTE:
                count += _DROPPED_PER_MINUTE * (
                    (rest - _DROPPED_PER_MINUTE) // _DF_FRAMES_PER_MINUTE
                )
        seconds, frames = divmod(count, rate.nominal)
        minutes, seconds = divmod(seconds, 60)
        hours, minutes = divmod(minutes, 60)
        return cls(hours, minutes, seconds, frames)

    def is_valid_at(self, rate: FrameRate) -> bool:
        if not (
            0 <= self.hours < 24
            and 0 <= self.minutes < 60
            and 0 <= self.seconds < 60
            and 0 <= self.frames < rate.nominal
        ):
            return False
        return not (
            rate.drop_frame
            and self.seconds == 0
            and self.minutes % 10 != 0
            and self.frames < _DROPPED_PER_MINUTE
        )

    def to_frame_count(self, rate: FrameRate) -> int:
        """The number of frames between 00:00:00:00 and this label."""
        count = (
            (self.hours * 60 + self.minutes) * 60 + self.seconds
        ) * rate.nominal + self.frames
        if rate.drop_frame:
            total_minutes = self.hours * 60 + self.minutes
            count -= _DROPPED_PER_MINUTE * (total_minutes - total_minutes // 10)
        return count

    def __str__(self) -> str:
        return f"{self.hours:02}:{self.minutes:02}:{self.seconds:02}:{self.frames:02}"
