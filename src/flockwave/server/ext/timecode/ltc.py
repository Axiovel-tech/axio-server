"""SMPTE 12M linear timecode (LTC) audio encoder and decoder.

An LTC frame is 80 bit cells, biphase-mark coded: every cell starts with a
level transition and a ``1`` bit has a second one in its middle, so the
signal carries its own clock and is immune to polarity inversion. Bits 64-79
hold a fixed sync word that marks the end of the frame. A polarity correction
bit keeps the number of transitions per frame even, so every frame starts
from the same level.

Sample positions are absolute indices on the stream's sample timeline; a
fractional position is a time between two samples.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import floor

import numpy as np

from .timecode import FrameRate, Timecode

__all__ = ("LtcDecoder", "LtcEncoder", "LtcFrame", "frame_bits")

BITS_PER_FRAME = 80

#: Bits 64-79 as an integer whose bit ``i`` is the bit transmitted ``i``-th.
SYNC_WORD = 0xBFFC

_DROP_FRAME_BIT = 10

#: (first bit, width) of the BCD digits, units first.
_FRAME_DIGITS = ((0, 4), (8, 2))
_SECOND_DIGITS = ((16, 4), (24, 3))
_MINUTE_DIGITS = ((32, 4), (40, 3))
_HOUR_DIGITS = ((48, 4), (56, 2))


def _polarity_bit(rate: FrameRate) -> int:
    return 59 if rate.nominal == 25 else 27


def _put_bcd(word: int, value: int, digits: tuple[tuple[int, int], ...]) -> int:
    for start, _width in digits:
        word |= (value % 10) << start
        value //= 10
    return word


def _get_bcd(word: int, digits: tuple[tuple[int, int], ...]) -> int | None:
    value, scale = 0, 1
    for start, width in digits:
        digit = (word >> start) & ((1 << width) - 1)
        if digit > 9:
            return None
        value += digit * scale
        scale *= 10
    return value


def frame_word(timecode: Timecode, rate: FrameRate) -> int:
    """The 80 bits of the LTC frame for ``timecode`` as an integer whose bit
    ``i`` is the bit transmitted ``i``-th. User bits are left zero."""
    word = _put_bcd(0, timecode.frames, _FRAME_DIGITS)
    word = _put_bcd(word, timecode.seconds, _SECOND_DIGITS)
    word = _put_bcd(word, timecode.minutes, _MINUTE_DIGITS)
    word = _put_bcd(word, timecode.hours, _HOUR_DIGITS)
    word |= int(rate.drop_frame) << _DROP_FRAME_BIT
    word |= SYNC_WORD << 64
    if word.bit_count() % 2:
        word |= 1 << _polarity_bit(rate)
    return word


def frame_bits(timecode: Timecode, rate: FrameRate) -> np.ndarray:
    """The 80 bits of the LTC frame for ``timecode``, in transmission order."""
    word = frame_word(timecode, rate)
    return np.array([(word >> i) & 1 for i in range(BITS_PER_FRAME)], dtype=np.uint8)


@dataclass(frozen=True)
class LtcFrame:
    """A decoded LTC frame."""

    timecode: Timecode
    drop_frame: bool
    start_sample: float
    """Position of the transition that starts the frame's first bit cell."""


def _parse_frame(word: int, start_sample: float) -> LtcFrame | None:
    frames = _get_bcd(word, _FRAME_DIGITS)
    seconds = _get_bcd(word, _SECOND_DIGITS)
    minutes = _get_bcd(word, _MINUTE_DIGITS)
    hours = _get_bcd(word, _HOUR_DIGITS)
    if frames is None or seconds is None or minutes is None or hours is None:
        return None
    if seconds >= 60 or minutes >= 60 or hours >= 24:
        return None
    return LtcFrame(
        Timecode(hours, minutes, seconds, frames),
        bool((word >> _DROP_FRAME_BIT) & 1),
        start_sample,
    )


class LtcEncoder:
    """Renders an LTC stream on an absolute sample timeline: the frame
    labelled ``start`` begins at sample position ``start_sample`` and each
    following frame advances the label by one. The signal is silent before
    ``start_sample``.

    Any window of the timeline can be rendered in any order, so an audio
    callback can ask for exactly the samples it is about to play. Each
    sample is the average of the ideal square wave over the sample period,
    which places the transitions at their exact fractional positions.
    """

    def __init__(
        self,
        rate: FrameRate,
        sample_rate: float,
        start: Timecode,
        start_sample: float = 0.0,
        amplitude: float = 0.5,
    ):
        self._rate = rate
        self._samples_per_frame = sample_rate / rate.fps
        self._start_count = start.to_frame_count(rate)
        self._start_sample = float(start_sample)
        self._amplitude = float(amplitude)
        self._transitions: dict[int, np.ndarray] = {}

        cell = self._samples_per_frame / BITS_PER_FRAME
        self._cell_starts = np.arange(BITS_PER_FRAME) * cell
        self._cell_middles = self._cell_starts + cell / 2

    @property
    def samples_per_frame(self) -> float:
        return self._samples_per_frame

    def frame_start(self, index: int) -> float:
        """Sample position where the ``index``-th frame of the stream starts."""
        return self._start_sample + index * self._samples_per_frame

    def timecode_of_frame(self, index: int) -> Timecode:
        return Timecode.from_frame_count(self._start_count + index, self._rate)

    def render(self, first_sample: int, count: int) -> np.ndarray:
        """The ``count`` samples starting at ``first_sample``, as float32."""
        lo = first_sample - 0.5
        hi = first_sample + count - 0.5
        first_frame = max(0, floor((lo - self._start_sample) / self._samples_per_frame))
        last_frame = floor((hi - self._start_sample) / self._samples_per_frame)
        if last_frame < first_frame:
            return np.zeros(count, dtype=np.float32)

        for index in [i for i in self._transitions if i < first_frame]:
            del self._transitions[index]
        transitions = np.concatenate(
            [self._frame_transitions(i) for i in range(first_frame, last_frame + 1)]
        )

        # Every frame starts from the low level; the level after the k-th
        # transition alternates high, low, high...
        levels = np.where(np.arange(len(transitions)) % 2 == 0, 1.0, -1.0)
        levels = np.concatenate(([0.0 if first_frame == 0 else -1.0], levels))
        knots = np.concatenate(([lo], np.clip(transitions, lo, hi), [hi]))
        integral = np.concatenate(([0.0], np.cumsum(levels * np.diff(knots))))
        bounds = lo + np.arange(count + 1)
        samples = np.diff(np.interp(bounds, knots, integral)) * self._amplitude
        return samples.astype(np.float32)

    def _frame_transitions(self, index: int) -> np.ndarray:
        transitions = self._transitions.get(index)
        if transitions is None:
            bits = frame_bits(self.timecode_of_frame(index), self._rate)
            offsets = np.sort(
                np.concatenate((self._cell_starts, self._cell_middles[bits == 1]))
            )
            transitions = self.frame_start(index) + offsets
            self._transitions[index] = transitions
        return transitions


class LtcDecoder:
    """Streaming LTC decoder. Feed consecutive blocks of mono samples to
    :meth:`decode`; it returns the frames that completed in the block.

    Transitions are found with hysteresis around the signal's midpoint
    (robust to gain, DC offset and noise) and located at the interpolated
    midpoint crossing. The bit-cell length is tracked from the signal, so
    any frame rate from 24 to 30 fps decodes at any sample rate.
    """

    #: Frame rates the bit-cell tracking may follow. Keeping it inside this
    #: range stops noise from dragging it onto half or double the real cell.
    MIN_FPS = 23.5
    MAX_FPS = 31.0

    #: Hysteresis as a fraction of the signal's half range.
    HYSTERESIS = 0.3

    #: Signals with a smaller half range are treated as silence.
    MIN_AMPLITUDE = 1e-3

    def __init__(self, sample_rate: float):
        self._sample_rate = float(sample_rate)
        self._position = 0
        self._history_size = max(1, int(self._sample_rate * 0.02))
        self._shortest_cell = self._sample_rate / (self.MAX_FPS * BITS_PER_FRAME)
        self._longest_cell = self._sample_rate / (self.MIN_FPS * BITS_PER_FRAME)
        self._cell_starts: deque[float] = deque(maxlen=BITS_PER_FRAME)
        self.reset()

    @property
    def position(self) -> int:
        """Absolute index of the next sample to be decoded."""
        return self._position

    def reset(self) -> None:
        """Forgets the signal seen so far, e.g. after a gap in the input. The
        sample positions keep counting."""
        self._history = np.zeros(0)
        self._state = 0
        self._previous_sample = 0.0
        self._cell = self._sample_rate / (27 * BITS_PER_FRAME)
        self._last_transition: float | None = None
        self._half_cell_start: float | None = None
        self._word = 0
        self._cell_starts.clear()

    def decode(self, samples: np.ndarray) -> list[LtcFrame]:
        x = np.asarray(samples, dtype=np.float64).ravel()
        if not x.size:
            return []
        base = self._position
        self._position += x.size

        window = np.concatenate((self._history, x))
        self._history = window[-self._history_size :]
        low, high = np.quantile(window, (0.02, 0.98))
        half_range = (high - low) / 2
        if half_range < self.MIN_AMPLITUDE:
            self._state = 0
            self._previous_sample = x[-1]
            return []

        frames: list[LtcFrame] = []
        for transition in self._find_transitions(x, base, (high + low) / 2, half_range):
            frame = self._on_transition(transition)
            if frame is not None:
                frames.append(frame)
        return frames

    def _find_transitions(
        self, x: np.ndarray, base: int, midpoint: float, half_range: float
    ) -> np.ndarray:
        y = np.concatenate(([self._previous_sample], x)) - midpoint
        self._previous_sample = x[-1]

        threshold = self.HYSTERESIS * half_range
        levels = np.where(y[1:] > threshold, 1, np.where(y[1:] < -threshold, -1, 0))
        settled = np.flatnonzero(levels)
        if not settled.size:
            return np.zeros(0)
        settled_levels = levels[settled]
        preceding = np.concatenate(([self._state], settled_levels[:-1]))
        self._state = int(settled_levels[-1])
        flips = settled[(settled_levels != preceding) & (preceding != 0)] + 1

        # The transition happened at the last midpoint crossing before the
        # signal settled beyond the hysteresis band.
        crossings = np.flatnonzero(np.diff(y >= 0)) + 1
        if crossings.size:
            nearest = np.searchsorted(crossings, flips, side="right") - 1
            crossings = np.where(nearest >= 0, crossings[np.maximum(nearest, 0)], flips)
        else:
            crossings = flips
        before, after = y[crossings - 1], y[crossings]
        with np.errstate(divide="ignore", invalid="ignore"):
            fraction = np.clip(before / (before - after), 0.0, 1.0)
        return base + crossings - 2 + np.nan_to_num(fraction)

    def _on_transition(self, transition: float) -> LtcFrame | None:
        previous = self._last_transition
        self._last_transition = transition
        if previous is None:
            return None

        interval = transition - previous
        if interval < 0.25 * self._cell or interval > 1.5 * self._cell:
            self._lose_sync()
            return None

        if interval < 0.75 * self._cell:
            if self._half_cell_start is None:
                self._half_cell_start = previous
                return None
            cell_start = self._half_cell_start
            self._half_cell_start = None
            return self._on_bit(1, cell_start, transition - cell_start)

        if self._half_cell_start is not None:
            self._lose_sync()
        return self._on_bit(0, previous, interval)

    def _on_bit(self, bit: int, cell_start: float, length: float) -> LtcFrame | None:
        self._cell += 0.1 * (length - self._cell)
        self._cell = min(max(self._cell, self._shortest_cell), self._longest_cell)
        self._word = (self._word >> 1) | (bit << (BITS_PER_FRAME - 1))
        self._cell_starts.append(cell_start)
        if len(self._cell_starts) == BITS_PER_FRAME and self._word >> 64 == SYNC_WORD:
            frame = _parse_frame(self._word, self._cell_starts[0])
            self._cell_starts.clear()
            return frame
        return None

    def _lose_sync(self) -> None:
        self._half_cell_start = None
        self._cell_starts.clear()
