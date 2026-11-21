"""The extension's side of the anchor cell lock: what the operator sees and
acknowledges when locking (the anchor views and warnings), the X-RTLS-LOCK
request, and the show gate that SHOW-SETCFG and X-RTLS-INF share."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Iterable, Optional, TypedDict

from .anchor_lock import (
    FIRMWARE_PARAM,
    AnchorLock,
    LockWarning,
    SealedAnchor,
    is_cell_param,
)

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "AnchorView",
    "ShowGate",
    "anchor_views",
    "cell_warnings",
    "handle_lock_request",
    "lock_json",
    "lock_warnings",
    "show_gate",
    "snapshot_entry",
)


#: ``uwbStatus`` values of a UWB start the anchor refused or could not make
#: (disabled, invalid config, hardware failure)
UWB_START_REFUSED = frozenset((2, 3, 4))


class ShowGate(TypedDict):
    required: bool
    refused: bool
    reason: Optional[str]


class AnchorView(TypedDict):
    """An anchor as the lock judges it, live or lost."""

    systemId: int
    name: str
    role: Optional[str]
    online: bool
    stats: Optional[dict[str, Any]]
    params: dict[str, Any]
    uptimeMs: Optional[int]
    uptimeAt: Optional[float]


def snapshot_entry(view: AnchorView) -> SealedAnchor:
    """The sealed record of one anchor from its view."""
    params = view["params"]
    stats = view["stats"] or {}
    return {
        "name": view["name"],
        "role": view["role"],
        "firmware": params.get(FIRMWARE_PARAM),
        "params": {k: v for k, v in params.items() if is_cell_param(k)},
        "bootCount": stats.get("bootCount"),
        "uptimeMs": view["uptimeMs"],
        "powerState": stats.get("powerState"),
        "moved": stats.get("moved"),
        "online": view["online"],
    }


def cell_warnings(anchors: Iterable[AnchorView]) -> list[LockWarning]:
    """What an operator acknowledges when locking the cell."""
    anchors = list(anchors)
    if not anchors:
        return [_warning("no-anchors", None, "no anchors are known")]
    warnings: list[LockWarning] = []
    initiators = [a for a in anchors if a["role"] == "anchor-initiator"]
    if not any(a["online"] for a in initiators):
        warnings.append(_warning("a0-not-operating", None, "no initiator (A0) online"))
    for anchor in anchors:
        warnings.extend(_anchor_warnings(anchor))
    return warnings


def _anchor_warnings(anchor: AnchorView) -> list[LockWarning]:
    system_id, name = anchor["systemId"], anchor["name"]
    if not anchor["online"]:
        return [_warning("offline", system_id, f"{name} is offline")]
    stats = anchor["stats"] or {}
    found = []
    state = stats.get("powerState")
    if anchor["role"] == "anchor-initiator" and state not in (None, "operating"):
        found.append(("a0-not-operating", f"{name} (A0) is {state}"))
    if anchor["role"] == "anchor-responder":
        if state not in (None, "operating"):
            found.append(("not-operating", f"{name} is {state}"))
        elif stats.get("rangingRateHz") == 0:
            found.append(("a0-not-heard", f"{name} does not hear A0"))
    if stats.get("lowBattery"):
        found.append(("low-battery", f"{name}: low battery{_volts(stats)}"))
    elif stats.get("batteryWarning"):
        found.append(
            ("low-battery", f"{name}: battery below its warning level{_volts(stats)}")
        )
    if stats.get("moved"):
        found.append(("moved", f"{name} moved since it was placed"))
    if stats.get("uwbStatus") in UWB_START_REFUSED:
        found.append(("uwb-start-refused", f"{name}: UWB did not start"))
    if stats.get("radioRestarts"):
        found.append(
            ("radio-restarts", f"{name}: radio restarted {stats['radioRestarts']}x")
        )
    return [_warning(code, system_id, detail) for code, detail in found]


def _volts(stats: dict[str, Any]) -> str:
    voltage = stats.get("batteryVoltage")
    return f" ({voltage:.2f} V)" if voltage is not None else ""


def _warning(code: str, device: Optional[int], detail: str) -> LockWarning:
    return {"code": code, "device": device, "detail": detail}


def anchor_views(ext: "RtlsExtension") -> list[AnchorView]:
    """Every known anchor, live or lost, as the lock judges it: its
    identity, whether it is online, its fresh stats, parameters and uptime."""
    from .extension import ANCHOR_ROLES, _device_name, role_from_params

    devices = ext._protocol.devices if ext._protocol else {}
    candidates = [(d, True) for d in devices.values()] + [
        (d, False) for sid, d in ext._lost.items() if sid not in devices
    ]
    views: list[AnchorView] = []
    for device, online in sorted(candidates, key=lambda c: c[0].system_id):
        params = ext._identity_params(device)
        role = role_from_params(params)
        if role not in ANCHOR_ROLES:
            continue
        system_id = device.system_id
        views.append(
            {
                "systemId": system_id,
                "name": _device_name(system_id, params, role) or f"device {system_id}",
                "role": role,
                "online": online,
                "stats": ext._fresh_stats(system_id) if online else None,
                "params": params,
                "uptimeMs": ext._uptime_ms.get(system_id) if online else None,
                "uptimeAt": ext._uptime_at.get(system_id) if online else None,
            }
        )
    return views


def lock_warnings(lock: AnchorLock, views: list[AnchorView]) -> list[LockWarning]:
    """The cell's warnings, plus, for a re-seal, the sealed members that are
    no longer known at all (dropping them needs an acknowledgement)."""
    warnings = cell_warnings(views)
    known = {view["systemId"] for view in views}
    for system_id in sorted(set(lock.snapshot) - known):
        warnings.append(
            _warning(
                "missing", system_id, f"{lock.name(system_id)} of the seal is not seen"
            )
        )
    return warnings


def handle_lock_request(ext: "RtlsExtension", body: dict[str, Any]) -> dict[str, Any]:
    """The X-RTLS-LOCK response body. Raises ValueError for an invalid
    action or a lock state that cannot be saved."""
    action = body.get("action", "status")
    if action not in ("status", "lock", "unlock"):
        raise ValueError(
            f"Invalid action: {action!r} (expected status, lock or unlock)"
        )
    lock = ext._anchor_lock
    views = anchor_views(ext)
    warnings = lock_warnings(lock, views)
    result = action
    try:
        if action == "lock":
            pending = warnings or lock.breaks
            if pending and body.get("acknowledgeWarnings") is not True:
                result = "warnings"
            else:
                lock.seal(
                    {view["systemId"]: snapshot_entry(view) for view in views},
                    warnings,
                    time.time(),
                    {view["systemId"]: view["uptimeAt"] for view in views},
                )
                result = "locked"
        elif action == "unlock":
            lock.unseal()
            result = "unlocked"
    except OSError as ex:
        raise ValueError(
            f"the anchor lock could not be saved ({ex}); nothing changed"
        ) from ex
    return {
        "type": "X-RTLS-LOCK",
        "action": action,
        "result": result,
        "warnings": warnings,
        "anchorLock": lock_json(ext),
    }


def show_gate(ext: "RtlsExtension") -> ShowGate:
    """Whether the server refuses to authorize or schedule a show now, and
    why: with the gate on and anchors known, the cell must be locked without
    breaks. SHOW-SETCFG and X-RTLS-INF's ``anchorLock.showGate`` both use
    this verdict."""
    lock = ext._anchor_lock
    required = ext._lock_required_for_show and bool(lock.locked or anchor_views(ext))
    reason = None
    if required and not lock.locked:
        reason = (
            "the anchors are not locked; lock them in Control before "
            "authorizing or scheduling the show"
        )
    elif required and lock.breaks:
        details = "; ".join(entry["detail"] for entry in lock.breaks[:3])
        more = len(lock.breaks) - 3
        reason = (
            f"the anchor lock is broken ({details}"
            + (f"; {more} more" if more > 0 else "")
            + "); review and re-seal it in Control first"
        )
    return {"required": required, "refused": reason is not None, "reason": reason}


def lock_json(ext: "RtlsExtension") -> dict[str, Any]:
    """``anchorLock`` as X-RTLS-INF and X-RTLS-LOCK carry it."""
    return {**ext._anchor_lock.json(), "showGate": show_gate(ext)}
