"""The anchor cell lock (X-RTLS-LOCK).

The firmware owns the lock (rtls-link-zephyr
``docs/reference/anchor-power-path.md``, "Lock") and reports it as the
``lock`` stat. The server writes ``LOCK`` to the online anchors, warns before
locking, derives the cell's lock state from the stats and gates the show
start on it.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional, TypedDict

import trio

from .cell_settings import cell_warnings

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "handle_lock_request",
    "lock_json",
    "locked_detail",
    "show_gate",
)

LOCK_PARAM = "LOCK"

#: how a message says an anchor is in a power state
_STATE_PHRASES = {
    "booting": "is booting",
    "preview": "is in preview",
    "standby": "is in standby",
    "asleep": "is asleep",
    "off": "is off",
    "stopping": "is stopping",
    "shutdown": "is shutting down",
    "fault": "has a power fault",
}


class LockWarning(TypedDict):
    code: str
    device: Optional[int]
    detail: str


class ShowGate(TypedDict):
    required: bool
    refused: bool
    reason: Optional[str]


class AnchorView(TypedDict):
    """An anchor as the lock sees it, live or lost."""

    systemId: int
    name: str
    role: Optional[str]
    online: bool
    stats: dict[str, Any]


def state_phrase(state: str) -> str:
    return _STATE_PHRASES.get(state, f"is in power state {state}")


def anchor_views(ext: "RtlsExtension") -> list[AnchorView]:
    """Every known anchor: the live ones with their fresh stats, the lost
    ones without."""
    from .extension import ANCHOR_ROLES, _device_name, role_from_params

    devices = ext._protocol.devices if ext._protocol else {}
    candidates = [(d, True) for d in devices.values()] + [
        (d, False) for sid, d in ext._lost.items() if sid not in devices
    ]
    views: list[AnchorView] = []
    for device, online in sorted(candidates, key=lambda c: c[0].system_id):
        params = ext._identity_params(device)
        role = role_from_params(params)
        if role in ANCHOR_ROLES:
            system_id = device.system_id
            views.append(
                {
                    "systemId": system_id,
                    "name": _device_name(system_id, params, role)
                    or f"RTLS anchor {system_id}",
                    "role": role,
                    "online": online,
                    "stats": (ext._fresh_stats(system_id) or {}) if online else {},
                }
            )
    return views


def lock_warnings(views: list[AnchorView]) -> list[LockWarning]:
    """What the operator acknowledges before locking."""
    if not views:
        return [_warning("no-anchors", None, "no anchors are known")]
    warnings: list[LockWarning] = []
    for view in views:
        system_id, name, stats = view["systemId"], view["name"], view["stats"]
        if not view["online"]:
            warnings.append(_warning("offline", system_id, f"{name} is offline"))
            continue
        if (
            view["role"] == "anchor-responder"
            and stats.get("powerState") in (None, "operating")
            and stats.get("rangingRateHz") == 0
        ):
            warnings.append(
                _warning("a0-not-heard", system_id, f"{name} does not hear A0")
            )
        if stats.get("lowBattery"):
            warnings.append(_warning("low-battery", system_id, f"{name}: low battery"))
        elif stats.get("batteryWarning"):
            warnings.append(
                _warning(
                    "battery-warning",
                    system_id,
                    f"{name}: battery below its warning level",
                )
            )
        if stats.get("moved"):
            warnings.append(
                _warning("moved", system_id, f"{name} moved since it was placed")
            )
        if stats.get("radioRestarts"):
            restarts = stats["radioRestarts"]
            warnings.append(
                _warning(
                    "radio-restarts", system_id, f"{name}: radio restarted {restarts}x"
                )
            )
    return warnings


def lock_refusal(views: list[AnchorView]) -> Optional[str]:
    """Why the cell cannot be locked now: an online anchor that has not
    reported operating (only an operating anchor accepts the lock)."""
    reasons = []
    for view in views:
        state = view["stats"].get("powerState")
        if not view["online"] or state == "operating":
            continue
        if state is None:
            reasons.append(f"{view['name']} has not reported its power state")
        else:
            todo = (
                "wake it first" if state in ("asleep", "standby") else "start it first"
            )
            reasons.append(f"{view['name']} {state_phrase(state)}; {todo}")
    return "; ".join(reasons) or None


async def handle_lock_request(
    ext: "RtlsExtension", body: dict[str, Any]
) -> dict[str, Any]:
    """The X-RTLS-LOCK response body. Raises ValueError for an invalid
    action."""
    action = body.get("action", "status")
    if action not in ("status", "lock", "unlock"):
        raise ValueError(
            f"Invalid action: {action!r} (expected status, lock or unlock)"
        )
    views = anchor_views(ext)
    warnings = lock_warnings(views) + cell_warnings(ext)
    reason = "A survey is running" if ext._survey.running else lock_refusal(views)
    result, results = action, None
    if action == "lock":
        if reason:
            result = "refused"
        elif warnings and body.get("acknowledgeWarnings") is not True:
            result = "warnings"
        else:
            results = await _write_lock(ext, views, 1)
    elif action == "unlock":
        results = await _write_lock(ext, views, 0)
    if results is not None:
        done = bool(results) and all(entry["accepted"] for entry in results.values())
        result = (
            ("locked" if action == "lock" else "unlocked") if done else "incomplete"
        )
        reason = None if results else "No online anchors"
    return {
        "type": "X-RTLS-LOCK",
        "action": action,
        "result": result,
        "reason": reason,
        "warnings": warnings,
        "results": results or {},
        "anchorLock": lock_json(ext),
    }


async def _write_lock(
    ext: "RtlsExtension", views: list[AnchorView], value: int
) -> dict[str, dict[str, Any]]:
    """Writes ``LOCK`` to every online anchor at once; per anchor whether it
    took the value."""
    results: dict[str, dict[str, Any]] = {}

    async def write(view: AnchorView) -> None:
        system_id = view["systemId"]
        try:
            ack = await ext.set_param(system_id, LOCK_PARAM, value, "uint8")
        except (KeyError, ValueError, trio.TooSlowError) as ex:
            detail = (
                ext._no_device(system_id)
                if isinstance(ex, KeyError)
                else (str(ex) or "no answer")
            )
            results[str(system_id)] = {"accepted": False, "detail": detail}
            return
        accepted = ack["accepted"] and int(ack["value"]) == value
        entry: dict[str, Any] = {"accepted": accepted}
        if not accepted:
            entry["detail"] = f"{view['name']} refused LOCK={value}"
        results[str(system_id)] = entry

    async with trio.open_nursery() as nursery:
        for view in views:
            if view["online"]:
                nursery.start_soon(write, view)
    return results


def lock_state(views: list[AnchorView]) -> dict[str, Any]:
    """The cell's lock as the online anchors report it (``lock`` stat)."""
    online = [view for view in views if view["online"]]
    unlocked = [view["systemId"] for view in online if not view["stats"].get("locked")]
    return {
        "locked": bool(online) and not unlocked,
        "lockedCount": len(online) - len(unlocked),
        "anchorCount": len(online),
        "unlocked": unlocked,
    }


def show_gate(
    ext: "RtlsExtension", views: Optional[list[AnchorView]] = None
) -> ShowGate:
    """Whether the server refuses to authorize or schedule a flight now:
    with the gate on, at least one online anchor and every online anchor
    must report its lock. SHOW-SETCFG and ``anchorLock.showGate`` both use
    this verdict."""
    views = anchor_views(ext) if views is None else views
    state = lock_state(views)
    required = ext._lock_required_for_show
    reason = None
    if required and not state["anchorCount"]:
        reason = "no anchors locked; lock them in Control first"
    elif required and not state["locked"]:
        reason = (
            f"the anchors are not locked ({state['lockedCount']}/"
            f"{state['anchorCount']} online anchors locked); lock them in Control first"
        )
    return {"required": required, "refused": reason is not None, "reason": reason}


def lock_json(ext: "RtlsExtension") -> dict[str, Any]:
    """``anchorLock`` as X-RTLS-INF and X-RTLS-LOCK carry it."""
    views = anchor_views(ext)
    return {**lock_state(views), "showGate": show_gate(ext, views)}


def locked_detail(ext: "RtlsExtension", system_id: int) -> Optional[str]:
    """``RTLS anchor A3 is locked`` when a refused command's anchor reports
    its lock, else None."""
    for view in anchor_views(ext):
        if view["systemId"] == system_id and view["stats"].get("locked"):
            return f"{view['name']} is locked"
    return None


def _warning(code: str, device: Optional[int], detail: str) -> LockWarning:
    return {"code": code, "device": device, "detail": detail}
