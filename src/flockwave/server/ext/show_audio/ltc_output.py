"""LTC on an extra channel of the music stream, for equipment that follows
the show as timecode slaves: the frame labelled with the configured start
timecode begins exactly at show T0.
"""

from __future__ import annotations

from statistics import median
from typing import Any

import numpy as np

from flockwave.server.ext.timecode.ltc import LtcEncoder
from flockwave.server.ext.timecode.timecode import FrameRate, Timecode

from .session import ChannelWriter

__all__ = ("LtcOutput",)

RESCHEDULE_TOLERANCE = 0.002
"""Before T0, a show clock time further than this from the LTC timeline means
the show start moved, and the LTC timeline follows it, in seconds."""

SETTLE_S = 1.0
"""The output stream opens this long before the LTC preroll, so the LTC
timeline is anchored on the median of the blocks' DAC timestamps rather than
on the first block's alone, which PipeWire reports ~0.1 ms late."""


class LtcOutput:
    """Configuration of the LTC output channel."""

    channel: int
    lead_s: float
    preroll_s: float
    rate: FrameRate
    start: Timecode

    def __init__(
        self, channel: int, rate: FrameRate, start: Timecode, preroll_s: float
    ):
        if channel < 0:
            raise ValueError("LTC output channels are numbered from 1")
        if not start.is_valid_at(rate):
            raise ValueError(f"{start} is not a valid timecode at {rate} fps")
        if preroll_s < 0:
            raise ValueError("LTC preroll must not be negative")

        self.channel = channel
        self.lead_s = preroll_s + SETTLE_S
        self.preroll_s = preroll_s
        self.rate = rate
        self.start = start

    @classmethod
    def from_json(cls, config: dict[str, Any]) -> LtcOutput:
        """Creates the LTC output from its ``ltc_output`` configuration, where
        channels are numbered from 1.

        Raises:
            ValueError: if the configuration is invalid
        """
        return cls(
            channel=int(config["channel"]) - 1,
            rate=FrameRate.from_fps(config.get("fps", 25)),
            start=Timecode.parse(str(config.get("start", "01:00:00:00"))),
            preroll_s=float(config.get("preroll_s", 10)),
        )

    def create_writer(self, sample_rate: int) -> ChannelWriter:
        return _LtcWriter(self, sample_rate)

    def describe(self) -> str:
        return (
            f"LTC {self.rate} fps on output channel {self.channel + 1}, "
            f"{self.start} at show T0, {self.preroll_s:g} s preroll"
        )


class _LtcWriter:
    """Renders the LTC of one output stream on the stream's own sample
    timeline. The timeline is anchored when the preroll begins; before T0 it
    moves with the show start, after T0 it never jumps.
    """

    def __init__(self, output: LtcOutput, sample_rate: int):
        self._output = output
        self._sample_rate = sample_rate
        self._preroll_frames = round(output.preroll_s * output.rate.fps)
        self._tolerance = RESCHEDULE_TOLERANCE * sample_rate
        self._encoder: LtcEncoder | None = None
        self._t0_estimates: list[float] = []
        self._t0_sample = 0.0
        self._next_sample = 0

    def __call__(self, out: np.ndarray, show_time: float) -> None:
        frames = out.shape[0]
        t0_sample = self._next_sample - show_time * self._sample_rate

        if self._encoder is None:
            estimates = self._t0_estimates
            if estimates and abs(t0_sample - estimates[-1]) > self._tolerance:
                estimates.clear()
            estimates.append(t0_sample)
            if show_time + frames / self._sample_rate < -self._output.preroll_s:
                out[:] = 0
                self._next_sample += frames
                return
            self._anchor(median(estimates))
        elif show_time < 0 and abs(t0_sample - self._t0_sample) > self._tolerance:
            self._anchor(t0_sample)

        assert self._encoder is not None
        out[:] = self._encoder.render(self._next_sample, frames)
        self._next_sample += frames

    def _anchor(self, t0_sample: float) -> None:
        output = self._output
        self._t0_sample = t0_sample
        first_label = Timecode.from_frame_count(
            output.start.to_frame_count(output.rate) - self._preroll_frames,
            output.rate,
        )
        self._encoder = LtcEncoder(
            output.rate,
            self._sample_rate,
            first_label,
            start_sample=t0_sample
            - self._preroll_frames * self._sample_rate / output.rate.fps,
        )
