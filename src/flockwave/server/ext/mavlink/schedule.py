"""The show start as the drones hold it: one schedule in UWB cluster time,
shared by every MAVLink network of the server.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from trio import Event

from flockwave.server.ext.show.takeoff import TakeoffConfiguration

from .packets import ShowScheduleStatus, create_show_schedule_packet
from .types import MAVLinkMessageSpecification

__all__ = ("ShowSchedule",)

MOVE_THRESHOLD_US = 1000
"""A start moved by at least this many microseconds by the operator is a new
schedule; a start derived again from the cluster clock estimate is re-sent
under the same schedule ID once it differs from the sent one by more."""

FAST_PERIOD_LEAD = 15.0
"""The schedule is sent every `FAST_PERIOD` seconds from this many seconds
before the start until the start, every `PERIOD` seconds otherwise."""

FAST_PERIOD = 0.2
PERIOD = 1.0


class ClusterClock(Protocol):
    def cluster_us_at(self, wall: float) -> int | None: ...
    def wall_at(self, cluster_us: int) -> float | None: ...


class ShowSchedule:
    """The scheduled show start in microseconds of the UWB cluster clock that
    every tag recovers, under an ID that changes whenever the operator sets,
    moves or clears the start.

    Before the point of no return the start is derived again from the cluster
    clock estimate as it moves; from the point of no return to the end of the
    show it is frozen and only an abort (clearing the start) changes it.
    """

    cluster_clock: Callable[[], ClusterClock | None]
    """Returns the cluster clock estimate; ``None`` if there is none."""

    is_locked: Callable[[], bool]
    """Returns whether the show is between its point of no return and its
    end."""

    id: int
    """The ID of the schedule (1-65535)."""

    start_cluster_us: int
    """The start in cluster microseconds as sent to the drones; zero for no
    start."""

    active: bool
    """Whether the server owns the start on the drones; with the RC start
    method and an authorized start the RC holder does."""

    def __init__(self):
        self.cluster_clock = lambda: None
        self.is_locked = lambda: False
        self.id = 1
        self.start_cluster_us = 0
        self.active = False
        self._start: float | None = None
        self._last_start: float | None = None
        self._changed = Event()

    @property
    def start(self) -> float | None:
        """The start set by the operator, as a UNIX timestamp."""
        return self._start

    def update(self, config: TakeoffConfiguration) -> None:
        """Takes the start that the operator set from the given takeoff
        configuration."""
        self.active = config.should_update_takeoff_time
        if not self.active:
            return

        start = config.takeoff_time
        if start is None and self._start is None:
            return
        if (
            start is not None
            and self._start is not None
            and abs(start - self._start) * 1e6 < MOVE_THRESHOLD_US
        ):
            return

        self._start = start
        if start is not None:
            self._last_start = start
        self.id = self.id % 0xFFFF + 1
        self.start_cluster_us = self._derive() or 0
        self._changed.set()
        self._changed = Event()

    def tick(self) -> None:
        """Derives the start again from the cluster clock estimate unless the
        start is locked."""
        if self._start is None or self.is_locked():
            return
        derived = self._derive()
        if (
            derived is not None
            and abs(derived - self.start_cluster_us) > MOVE_THRESHOLD_US
        ):
            self.start_cluster_us = derived

    def packet(self, now: float) -> MAVLinkMessageSpecification | None:
        """The schedule packet to send at the given time; ``None`` if none
        is due: the RC holder owns the start, the start is not known in
        cluster time yet, or it has passed.
        """
        if not self.active:
            return None
        if self._start is not None and (
            self.start_cluster_us == 0 or self._start < now
        ):
            return None
        return create_show_schedule_packet(self.id, self.start_cluster_us)

    def period(self, now: float) -> float:
        """Seconds until the schedule is sent again. After an abort it keeps
        the pace of the aborted start."""
        start = self._last_start
        if start is not None and 0 < start - now <= FAST_PERIOD_LEAD:
            return FAST_PERIOD
        return PERIOD

    def disagrees_with(self, echo: ShowScheduleStatus) -> bool:
        """Whether a drone's echo shows that it does not hold this schedule.
        A committed drone's start moves with the drift of its crystal."""
        return echo.schedule_id != self.id or (
            not echo.committed and echo.start_cluster_us != self.start_cluster_us
        )

    def wall_at(self, cluster_us: int) -> float | None:
        clock = self.cluster_clock()
        return clock.wall_at(cluster_us) if clock is not None else None

    async def wait_for_change(self) -> None:
        """Waits until the schedule ID changes."""
        await self._changed.wait()

    def _derive(self) -> int | None:
        clock = self.cluster_clock()
        if clock is None or self._start is None:
            return None
        return clock.cluster_us_at(self._start)
