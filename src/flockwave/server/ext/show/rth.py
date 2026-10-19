"""Certified return to home for `.gdm` shows.

Implements the ground side of AxioStack ``contracts/gdm-format.md`` §6.7 and
§9.1, and the ``X-SHOW-RTH`` request of ``contracts/gdm-ground-rth.md``.

The server neither plans nor checks return paths. The trajectory engine
precomputed one path per drone for every branch time of the show and
certified that *any subset* of drones can take them (§8). The server only
picks the first branch time that the command can still reach and sends the
selected drones the command to take it.
"""

from __future__ import annotations

from bisect import bisect_left
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field

from flockwave.server.errors import FlockwaveError

__all__ = (
    "RTHDispatch",
    "RTHPlan",
    "RTHPlanRegistry",
    "RTHRequestRefused",
    "first_branch_time_at_or_after",
    "plan_rth_dispatch",
)


class RTHRequestRefused(FlockwaveError):
    """Raised when an RTH request is refused as a whole, so no drone is
    commanded.
    """


@dataclass(frozen=True)
class RTHPlan:
    """The RTH plan that one drone holds, as far as the ground needs it."""

    show_hash: str
    """Hash of the show that the drone holds (``.gdmb`` header show hash).
    Drones that hold the same show share the same branch times.
    """

    times_us: tuple[int, ...]
    """The branch times of the plan (``show.json`` ``rth.times_us``), in
    microseconds of show time, strictly increasing.
    """

    def __post_init__(self) -> None:
        if not self.times_us:
            raise ValueError("An RTH plan has at least one branch time")
        if any(t < 0 for t in self.times_us):
            raise ValueError("RTH branch times must not be negative")
        if any(a >= b for a, b in zip(self.times_us, self.times_us[1:])):
            raise ValueError("RTH branch times must be strictly increasing")


class RTHPlanRegistry:
    """The RTH plans held by the UAVs, keyed by UAV ID.

    The `.gdm` upload path registers the plan of a UAV once its `.gdmb` is
    verified on the vehicle, and clears it when the UAV gets a show without
    one.
    """

    _plans: dict[str, RTHPlan]

    def __init__(self) -> None:
        self._plans = {}

    def clear(self, uav_id: str) -> None:
        """Forgets the plan of the given UAV, if any."""
        self._plans.pop(uav_id, None)

    def clear_all(self) -> None:
        """Forgets every plan."""
        self._plans.clear()

    def get(self, uav_id: str) -> RTHPlan | None:
        """Returns the plan of the given UAV, or ``None`` if it holds none."""
        return self._plans.get(uav_id)

    def set(self, uav_id: str, plan: RTHPlan) -> None:
        """Records that the given UAV holds the given plan."""
        self._plans[uav_id] = plan

    @property
    def uav_ids(self) -> list[str]:
        """The IDs of the UAVs that hold a plan, sorted."""
        return sorted(self._plans)


@dataclass(frozen=True)
class RTHDispatch:
    """The outcome of planning an RTH request that was not refused."""

    branch_time_us: int
    """The branch time to send to every UAV in ``uav_ids``."""

    uav_ids: list[str]
    """The UAVs to command, in request order."""

    errors: dict[str, str] = field(default_factory=dict)
    """Requested UAVs that are not commanded, with the reason."""


def first_branch_time_at_or_after(
    times_us: Sequence[int], earliest_us: int
) -> int | None:
    """Returns the first branch time that is not earlier than the given time,
    or ``None`` if every branch time is earlier.
    """
    index = bisect_left(times_us, earliest_us)
    return times_us[index] if index < len(times_us) else None


def plan_rth_dispatch(
    registry: RTHPlanRegistry,
    target_ids: Iterable[str],
    *,
    show_time_us: int | None,
    latency_us: int,
) -> RTHDispatch:
    """Decides which branch time to send to which of the requested UAVs.

    Any subset of drones is certified to return safely (§8). So a requested
    UAV that holds no plan is reported in ``errors``, the others are still
    commanded, and the drone without a plan keeps flying the show. The
    request is refused as a whole only when it cannot send anyone home along
    a certified path.

    Parameters:
        registry: the RTH plans held by the UAVs
        target_ids: the requested UAVs
        show_time_us: the current show time in microseconds, or ``None`` if
            the show clock is not running
        latency_us: how far ahead of the current show time the branch time
            must be for the command to reach every drone

    Raises:
        RTHRequestRefused: if no UAV is requested, the show is not running,
            no requested UAV holds a plan, the requested UAVs hold different
            shows, or no branch time remains far enough ahead
    """
    target_ids = list(dict.fromkeys(target_ids))
    if not target_ids:
        raise RTHRequestRefused("No drones selected for RTH")

    if show_time_us is None or show_time_us < 0:
        raise RTHRequestRefused("The show is not running")

    errors: dict[str, str] = {}
    holders: dict[str, RTHPlan] = {}
    for uav_id in target_ids:
        plan = registry.get(uav_id)
        if plan is None:
            errors[uav_id] = "No certified RTH plan on this drone"
        else:
            holders[uav_id] = plan

    if not holders:
        raise RTHRequestRefused("No selected drone holds a certified RTH plan")

    plans = set(holders.values())
    if len(plans) > 1:
        # Certification covers the drones of one show only.
        raise RTHRequestRefused(
            "The selected drones hold different shows; upload the same show "
            "to all of them"
        )

    (plan,) = plans
    branch_time_us = first_branch_time_at_or_after(
        plan.times_us, show_time_us + latency_us
    )
    if branch_time_us is None:
        raise RTHRequestRefused(
            f"No RTH branch time remains at least {latency_us // 1000} ms ahead"
        )

    return RTHDispatch(
        branch_time_us=branch_time_us, uav_ids=list(holders), errors=errors
    )
