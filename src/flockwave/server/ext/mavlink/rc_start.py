"""Fleet-safe RC show start: reflect one drone's RC-initiated start time
to the whole fleet as a scheduled start.

With the classic per-drone RC start, every drone anchors the start to the
instant IT received the RC flip (the firmware's DRONE_SHOW_START aux
schedules ``receipt + 10 s`` on its own clock), so the fleet inherits the
drone-to-drone RC receipt jitter -- and a drone that misses the flip never
starts. On a fleet with a shared show clock (GPS, or the UWB-timebase
synthetic GPS pin) the safe pattern is: treat the FIRST drone that reports
an RC-set start time as the observer that stamped the flip, then push that
ONE absolute start time to every drone through the regular scheduled-start
machinery. Every drone then fires locally at the same shared-clock instant;
the ~10 s fuse gives the reflection seconds of margin, and nothing
time-critical crosses the radio at the start instant.

The reflector only ever acts when the operator has armed it: start method
RC, show start authorized, and no server-side scheduled start yet. On
reflection the start method switches to AUTO, so the adopted start time is
visible (and revocable) in the GCS exactly like an operator-scheduled one.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Callable, Iterable, Optional

from flockwave.server.ext.show.config import DroneShowConfiguration, StartMethod

if TYPE_CHECKING:
    from .driver import MAVLinkUAV

__all__ = ("RCStartReflector", "find_rc_start_time")

#: A reported start time is a plausible RC start only inside this window:
#: slightly in the past absorbs status latency; the future bound rejects
#: stale or garbage schedules (the firmware fuse is ~10 s).
_PAST_SLACK_S = 5.0
_FUTURE_LIMIT_S = 300.0


def find_rc_start_time(
    uavs: Iterable["MAVLinkUAV"], now: Optional[float] = None
) -> Optional[int]:
    """Returns the earliest plausible drone-reported scheduled takeoff time
    (UNIX seconds) among ``uavs``, or ``None``. The earliest report wins so
    every reflection of the same RC flip converges on one instant."""
    if now is None:
        now = time.time()
    best: Optional[int] = None
    for uav in uavs:
        t = uav.scheduled_takeoff_time
        if t is None:
            continue
        if t < now - _PAST_SLACK_S or t > now + _FUTURE_LIMIT_S:
            continue
        if best is None or t < best:
            best = t
    return best


class RCStartReflector:
    """Watches the fleet for an RC-initiated start and reflects it into the
    show configuration (via the show extension's ``schedule_start`` API), so
    the regular scheduled-takeoff machinery pushes the SAME start time to
    every drone."""

    def __init__(
        self,
        get_configuration: Callable[[], DroneShowConfiguration],
        schedule_start: Callable[[float], None],
        log=None,
    ):
        self._get_configuration = get_configuration
        self._schedule_start = schedule_start
        self._log = log

    def check(self, uavs: Iterable["MAVLinkUAV"], now: Optional[float] = None) -> bool:
        """One scan: reflect if armed (RC method + authorized + unscheduled)
        and a plausible drone-reported start time exists. Returns whether a
        reflection happened."""
        config = self._get_configuration()
        if (
            config.start_method is not StartMethod.RC
            or not config.authorized_to_start
            or config.start_time_on_clock is not None
        ):
            return False

        start_time = find_rc_start_time(uavs, now)
        if start_time is None:
            return False

        if self._log:
            self._log.info(
                f"RC show start detected at T={start_time}; scheduling the "
                "whole fleet on it"
            )
        self._schedule_start(float(start_time))
        return True
