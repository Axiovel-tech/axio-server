"""Timing rules of the show music player, kept free of audio I/O so they can
be tested without a sound card.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

from flockwave.server.ext.show.config import DroneShowConfiguration, StartMethod

__all__ = (
    "Phase",
    "followed_start_time",
    "frames_until",
    "is_committed",
    "playback_phase",
    "render_block",
)

TAIL_S = 0.5
"""Time after the scheduled end of the track until the output stays open, so
the last buffered samples reach the DAC even if the device clock runs slow.
"""


class Phase(Enum):
    """Where the player is relative to a scheduled track."""

    WAIT = "wait"
    """Too early to open the output stream."""

    PLAY = "play"
    """The output stream must be open: before sample 0 it plays silence, after
    it the track (joining mid-track if sample 0 is already in the past)."""

    OVER = "over"
    """The track has ended."""


def followed_start_time(
    config: DroneShowConfiguration, show_clock_start_time: float | None
) -> float | None:
    """Returns the show start that the music follows: the one the server will
    start by itself (automatic start method, authorized in any scope, start
    time set on the show clock), or ``None``.
    """
    if (
        config.start_method is StartMethod.AUTO
        and config.authorized_to_start
        and show_clock_start_time is not None
    ):
        return show_clock_start_time
    return None


def is_committed(now: float, start_time: float, point_of_no_return_s: float) -> bool:
    """Returns whether the show starting at ``start_time`` has passed the
    show extension's point of no return (seconds on the show clock).
    """
    return now >= start_time + point_of_no_return_s


def playback_phase(
    now: float, sample0_time: float, duration: float, arm_lead_s: float
) -> Phase:
    """Returns the phase of a track whose sample 0 is due on the DAC at
    ``sample0_time``.
    """
    if now < sample0_time - arm_lead_s:
        return Phase.WAIT
    elif now < sample0_time + duration + TAIL_S:
        return Phase.PLAY
    else:
        return Phase.OVER


def frames_until(target_time: float, dac_time: float, sample_rate: int) -> int:
    """Returns the number of frames between the frame that reaches the DAC at
    ``dac_time`` and ``target_time``; negative if ``target_time`` is earlier.
    """
    return round((target_time - dac_time) * sample_rate)


def render_block(out: np.ndarray, samples: np.ndarray, position: int, lead: int) -> int:
    """Fills an output block with ``lead`` frames of silence followed by the
    track from ``position`` on, padding with silence after the end of the
    track. Channels of ``out`` beyond the track's are silenced.

    Args:
        out: the output block, frames x channels
        samples: the track, frames x channels
        position: index of the first track frame to write
        lead: number of silent frames before the track data

    Returns:
        the track position after the block
    """
    frames = out.shape[0]
    channels = samples.shape[1]
    count = max(0, min(frames - lead, samples.shape[0] - position))

    out[:lead] = 0
    out[lead : lead + count, :channels] = samples[position : position + count]
    out[lead : lead + count, channels:] = 0
    out[lead + count :] = 0
    return position + count
