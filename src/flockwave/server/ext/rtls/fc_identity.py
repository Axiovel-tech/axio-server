"""The flight controller each tag reports through its FC_SYS_ID / FC_STATE
parameters, as surfaced in the X-RTLS-INF ``flightController`` member.

The firmware persists the system id of the autopilot it last confirmed on
its UART and re-advertises it with its association state, also while the
drone sleeps with the flight controller unpowered.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Literal, Mapping, TypedDict

__all__ = ("FlightController", "flight_controller_claims")


class _FlightControllerState(TypedDict):
    state: Literal["live", "remembered", "ambiguous"]


class FlightController(_FlightControllerState, total=False):
    systemId: int
    reason: str


#: FC_STATE encoding of rtls-link-zephyr's flight_controller::Association
_UNKNOWN, _LIVE, _REMEMBERED, _AMBIGUOUS = 0, 1, 2, 3


def flight_controller_claims(
    devices: Mapping[int, tuple[Mapping[str, object], bool]],
) -> dict[int, FlightController]:
    """Maps each device id to its ``flightController`` entry, given its
    decoded parameters and sleep flag. Devices without an identity are left
    out; an identity claimed by more than one device is ambiguous on every
    device except the single one that hears it live."""
    claims: dict[int, FlightController] = {}
    for system_id, (params, sleeping) in devices.items():
        claim = _claim(params.get("FC_SYS_ID"), params.get("FC_STATE"), sleeping)
        if claim is not None:
            claims[system_id] = claim
    _contest_shared_identities(claims)
    return claims


def _claim(identity: object, state: object, sleeping: bool) -> FlightController | None:
    if identity is None or state is None:
        return None
    if not isinstance(identity, int) or not 0 <= identity <= 255:
        return _inconsistent()
    if state == _UNKNOWN and identity == 0:
        return None
    if state == _AMBIGUOUS:
        claim = FlightController(
            state="ambiguous",
            reason="the tag hears conflicting flight-controller heartbeats",
        )
        if identity:
            claim["systemId"] = identity
        return claim
    if state in (_LIVE, _REMEMBERED) and identity:
        # the advertisement that carried "live" may predate the sleep flip,
        # and a sleeping drone's flight controller is unpowered
        live = state == _LIVE and not sleeping
        return FlightController(
            systemId=identity, state="live" if live else "remembered"
        )
    return _inconsistent()


def _inconsistent() -> FlightController:
    return FlightController(
        state="ambiguous", reason="inconsistent flight-controller identity"
    )


def _contest_shared_identities(claims: dict[int, FlightController]) -> None:
    holders: defaultdict[int, list[int]] = defaultdict(list)
    for system_id, claim in claims.items():
        if "systemId" in claim:
            holders[claim["systemId"]].append(system_id)
    for identity, tags in holders.items():
        if len(tags) < 2:
            continue
        live = [tag for tag in tags if claims[tag]["state"] == "live"]
        winner = live[0] if len(live) == 1 else None
        for tag in tags:
            if tag != winner:
                others = [str(other) for other in sorted(tags) if other != tag]
                noun = "tag" if len(others) == 1 else "tags"
                claims[tag] = FlightController(
                    systemId=identity,
                    state="ambiguous",
                    reason=f"flight controller {identity} is also claimed by "
                    f"{noun} {', '.join(others)}",
                )
