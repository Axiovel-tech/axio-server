"""Show music player: puts sample 0 of the show's track on the DAC at the
scheduled show start (plus the configured offset) and plays it through.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from enum import Enum
from logging import Logger
from math import inf
from time import time
from typing import Any

import trio

from flockwave.server.utils.formatting import format_timestamp_nicely

from .session import (
    STATS_BLOCKS,
    ExtraOutput,
    PlaybackSession,
    Stream,
    open_output_stream,
)
from .timing import Phase, is_committed, playback_phase
from .track import AudioTrack

__all__ = (
    "PlaybackCommittedError",
    "PlayerState",
    "ShowAudioPlayer",
)

POLL_INTERVAL = 0.1
"""Interval at which the player checks an open session, in seconds."""

RETRY_INTERVAL = 1.0
"""Delay before retrying to open an output stream that failed to open."""

STOP_RELEASE = 1.0
"""An operator stop holds until the followed show start is cleared or moves
further than this, in seconds; timecode re-anchoring moves it by far less."""


class PlaybackCommittedError(RuntimeError):
    """Raised when a change would disturb music committed to a running show."""

    def __init__(self):
        super().__init__(
            "Music is committed to the running show; stop it first "
            "with X-SHOW-AUDIO-STOP"
        )


class PlayerState(Enum):
    IDLE = "idle"
    ARMED = "armed"
    PLAYING = "playing"
    FINISHED = "finished"
    ERROR = "error"


@dataclass(frozen=True)
class _Target:
    """A track to play for a given show start."""

    start_time: float
    offset_ms: float
    track: AudioTrack

    @property
    def sample0_time(self) -> float:
        return self.start_time + self.offset_ms / 1000


class ShowAudioPlayer:
    """Follows the show start that the server schedules and plays the show's
    track in sync with it.

    Before the show's point of no return the player follows reschedules and
    cancellations; after it the playback is committed and only `stop()` or
    shutting down the player ends it. A start already in the past joins the
    track at the matching position.
    """

    on_change: Callable[[], None] | None
    """Called when anything reported by `json` may have changed."""

    extra_output: ExtraOutput | None

    def __init__(
        self,
        *,
        log: Logger,
        device: str | int | None = None,
        offset_ms: float = 0.0,
        arm_lead_s: float = 5.0,
        point_of_no_return_s: Callable[[], float | None] = lambda: -10.0,
        open_stream: Callable[
            [PlaybackSession, str | int | None], Awaitable[Stream]
        ] = open_output_stream,
        clock: Callable[[], float] = time,
    ):
        self.on_change = None
        self.extra_output = None

        self._log = log
        self._device = device
        self._offset_ms = offset_ms
        self._arm_lead_s = arm_lead_s
        self._point_of_no_return_s = point_of_no_return_s
        self._open_stream = open_stream
        self._clock = clock

        self._track: AudioTrack | None = None
        self._followed: float | None = None
        self._committed: _Target | None = None
        self._stopped_at: float | None = None
        self._finished_start_time: float | None = None

        self._session: PlaybackSession | None = None
        self._stream: Stream | None = None
        self._start_logged = False
        self._start_error_logged = False

        self._state = PlayerState.IDLE
        self._error: str | None = None
        self._retry_at = -inf
        self._changed = trio.Event()

    @property
    def json(self) -> dict[str, Any]:
        """The player status reported by ``X-SHOW-AUDIO-INF``."""
        return {
            "track": self._track.json if self._track else None,
            "state": self._state.value,
            "startTime": (
                self._committed.start_time if self._committed else self._followed
            ),
            "offsetMs": self._offset_ms,
            "device": self._device,
            "error": self._error,
        }

    @property
    def state(self) -> PlayerState:
        return self._state

    @property
    def track(self) -> AudioTrack | None:
        return self._track

    def ensure_not_committed(self) -> None:
        """Raises `PlaybackCommittedError` if the music is committed to a
        show.
        """
        self._commit_if_due(self._clock())
        if self._committed is not None:
            raise PlaybackCommittedError()

    def follow(self, start_time: float | None) -> None:
        """Sets the show start to follow: the UNIX timestamp of show T0 when
        the server will start the show by itself, ``None`` otherwise.
        """
        self._commit_if_due(self._clock())
        if start_time != self._followed:
            self._followed = start_time
            if self._stopped_at is not None and (
                start_time is None or abs(start_time - self._stopped_at) > STOP_RELEASE
            ):
                self._stopped_at = None
            self._wake()

    def set_offset_ms(self, offset_ms: float) -> None:
        """Sets the playback offset (positive = audio later).

        Raises:
            PlaybackCommittedError: if the music is committed to a show
        """
        self.ensure_not_committed()
        self._offset_ms = offset_ms
        self._wake()

    def set_track(self, track: AudioTrack | None) -> None:
        """Replaces or unloads the track.

        Raises:
            PlaybackCommittedError: if the music is committed to a show
        """
        self.ensure_not_committed()
        self._track = track
        self._error = None
        self._wake()

    def stop(self) -> None:
        """Stops the playback now and keeps it stopped for the followed show
        start until that start is cleared or moves by more than
        `STOP_RELEASE`.
        """
        if self._session is not None:
            self._session.silenced = True
            self._log.warning("Show music stopped by the operator")
        self._stopped_at = self._followed
        self._committed = None
        self._wake()

    async def run(self) -> None:
        """Runs the player until cancelled."""
        try:
            while True:
                timeout = await self._step()
                changed = self._changed
                if not changed.is_set():
                    with trio.move_on_after(max(timeout, 0)):
                        await changed.wait()
                self._changed = trio.Event()
        finally:
            with trio.CancelScope(shield=True):
                await self._close_session()

    def _arm_lead(self, target: _Target) -> float:
        lead = self._arm_lead_s
        if self.extra_output is not None:
            lead = max(lead, self.extra_output.lead_s + target.offset_ms / 1000)
        return lead

    def _commit_if_due(self, now: float) -> None:
        if (
            self._committed is not None
            or self._followed is None
            or self._track is None
            or self._stopped_at is not None
        ):
            return

        target = _Target(self._followed, self._offset_ms, self._track)
        point_of_no_return_s = self._point_of_no_return_s()
        if (
            point_of_no_return_s is not None
            and is_committed(now, target.start_time, point_of_no_return_s)
            and playback_phase(now, target.sample0_time, target.track.duration, 0)
            is not Phase.OVER
        ):
            self._committed = target
            self._log.info(
                "Show music committed to the show start at "
                f"{format_timestamp_nicely(self._followed)}"
            )

    async def _close_session(self) -> None:
        session, stream = self._session, self._stream
        if session is None or stream is None:
            return

        self._session = self._stream = None
        session.silenced = True
        await stream.aclose()

        if session.started:
            drift = session.end_drift()
            self._log.info(
                "Show music ended at {:.1f} s into the track; end-of-track "
                "drift {}, {} underflow(s)".format(
                    session.position,
                    _format_ms(drift),
                    session.underflows,
                )
            )

    def _current_target(self) -> _Target | None:
        if self._committed is not None:
            return self._committed
        if (
            self._followed is None
            or self._track is None
            or self._stopped_at is not None
        ):
            return None
        return _Target(self._followed, self._offset_ms, self._track)

    def _set_state(self, state: PlayerState) -> None:
        self._state = state
        if self.on_change:
            self.on_change()

    async def _step(self) -> float:
        """Brings the output stream in line with the current target and
        returns the time to wait until the next step unless something changes.
        """
        now = self._clock()
        self._commit_if_due(now)

        target = self._current_target()
        if target is None:
            await self._close_session()
            self._error = None
            self._set_state(PlayerState.IDLE)
            return inf

        arm_lead = self._arm_lead(target)
        phase = playback_phase(
            now, target.sample0_time, target.track.duration, arm_lead
        )

        if phase is Phase.WAIT:
            await self._close_session()
            self._set_state(PlayerState.IDLE)
            wake_at = target.sample0_time - arm_lead
            point_of_no_return_s = self._point_of_no_return_s()
            if self._committed is None and point_of_no_return_s is not None:
                wake_at = min(wake_at, target.start_time + point_of_no_return_s)
            return wake_at - now

        if phase is Phase.OVER:
            if self._session is not None and self._session.started:
                self._finished_start_time = target.start_time
            await self._close_session()
            if self._committed is target:
                self._committed = None
            self._set_state(
                PlayerState.FINISHED
                if self._finished_start_time == target.start_time
                else PlayerState.IDLE
            )
            return inf

        return await self._play(target, now)

    async def _play(self, target: _Target, now: float) -> float:
        session = self._session
        if session is not None:
            moved = (
                session.start_time != target.start_time
                or session.sample0_time != target.sample0_time
            )
            if session.track is not target.track or (moved and session.started):
                await self._close_session()
                session = None
            elif moved:
                session.start_time = target.start_time
                session.sample0_time = target.sample0_time
                self._log.info(
                    "Show music rescheduled to "
                    f"{format_timestamp_nicely(target.sample0_time)}"
                )

        if session is None:
            if now < self._retry_at:
                return self._retry_at - now
            return await self._arm(target)

        self._log_start(session)
        if session.finished:
            self._set_state(PlayerState.FINISHED)
        elif session.started:
            self._set_state(PlayerState.PLAYING)
        else:
            self._set_state(PlayerState.ARMED)
        return POLL_INTERVAL

    def _log_start(self, session: PlaybackSession) -> None:
        if session.started and not self._start_logged:
            self._start_logged = True
            joined_at = session.joined_at or 0.0
            if joined_at > 0:
                self._log.warning(
                    f"Show music joined the running show {joined_at:.2f} s into "
                    f"the track"
                )
            else:
                self._log.info("Show music started")

        error = None if self._start_error_logged else session.start_error()
        if error is not None:
            self._start_error_logged = True
            self._log.info(
                f"Show music start error {_format_ms(error)} "
                f"(median of {STATS_BLOCKS} blocks)"
            )

    async def _arm(self, target: _Target) -> float:
        try:
            session = PlaybackSession(
                target.track,
                target.start_time,
                target.sample0_time,
                extra=self.extra_output,
                clock=self._clock,
            )
            stream = await self._open_stream(session, self._device)
        except Exception as ex:
            message = f"Cannot open the audio output: {ex}"
            if message != self._error:
                self._log.error(message)
            self._error = message
            self._retry_at = self._clock() + RETRY_INTERVAL
            self._set_state(PlayerState.ERROR)
            return RETRY_INTERVAL

        self._session, self._stream = session, stream
        self._start_logged = self._start_error_logged = False
        self._error = None
        self._retry_at = -inf
        self._log.info(
            f"Show music armed on {stream.description}: {target.track.name!r}, "
            f"sample 0 at {format_timestamp_nicely(target.sample0_time)}"
        )
        self._set_state(PlayerState.ARMED)
        return 0

    def _wake(self) -> None:
        self._changed.set()
        if self.on_change:
            self.on_change()


def _format_ms(seconds: float | None) -> str:
    return "n/a" if seconds is None else f"{seconds * 1000:+.2f} ms"
