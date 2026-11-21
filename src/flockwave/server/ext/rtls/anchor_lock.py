"""The anchor cell lock (X-RTLS-LOCK): a sealed snapshot of the anchors, and
the breaks seen since, judged from the events the extension already handles.

The lock never acts on devices. The extension refuses anchor-changing
commands while it is locked, records a break whenever an anchor of the
snapshot changes on its own, and answers the show extension whether a show
may be authorized (:func:`show_gate`).
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, Iterable, Optional, TypedDict

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "AnchorLock",
    "LOCKED_REASON",
    "ROLE_UNKNOWN_REASON",
    "cell_warnings",
    "handle_lock_request",
    "lock_json",
    "show_gate",
)

LOCKED_REASON = "anchors are locked; unlock them in Control first"
ROLE_UNKNOWN_REASON = "role unknown while anchors are locked; not commanded"

#: the parameter that carries the firmware version
FIRMWARE_PARAM = "FW_VERSION"

#: parameters outside the ``UWB_`` session and table set that define the cell
CELL_PARAMS = frozenset(("MAV_SYS_ID", "BAT_PROFILE"))

#: ``uwbStatus`` values of a UWB start the anchor refused or could not make
#: (disabled, invalid config, hardware failure)
UWB_START_REFUSED = frozenset((2, 3, 4))

#: how far an uptime may drift from the host time elapsed between two of its
#: samples before it counts as a reboot, in ms (report and network jitter)
UPTIME_SLACK_MS = 1000


class LockWarning(TypedDict):
    code: str
    device: Optional[int]
    detail: str


class LockBreak(TypedDict):
    code: str
    device: int
    detail: str
    at: float


class ShowGate(TypedDict):
    required: bool
    refused: bool
    reason: Optional[str]


class SealedAnchor(TypedDict):
    name: str
    role: Optional[str]
    firmware: Any
    params: dict[str, Any]
    bootCount: Optional[int]
    uptimeMs: Optional[int]
    powerState: Optional[str]
    moved: Optional[bool]
    online: bool


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


def is_cell_param(name: str) -> bool:
    """Whether a parameter defines the cell (part of the lock snapshot)."""
    return name.startswith("UWB_") or name in CELL_PARAMS


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
    if anchor["role"] == "anchor-responder" and stats.get("rangingRateHz") == 0:
        found.append(("a0-not-heard", f"{name} does not hear A0"))
    if stats.get("lowBattery"):
        found.append(("low-battery", f"{name}: low battery{_volts(stats)}"))
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


class AnchorLock:
    """The lock state: whether the cell is locked, its sealed snapshot, the
    warnings acknowledged when sealing it and the breaks seen since.

    The ``on_*`` observers compare each event with the last value seen for
    that anchor (starting from the snapshot, persisted with the breaks), so
    every change becomes one break; they return the new breaks. An operator
    transition is kept only once it is saved; a telemetry break is kept even
    when saving it fails (``persist_error`` says why)."""

    def __init__(self, path: Optional[Path] = None):
        #: where the state persists; None keeps it in memory only
        self.path = path
        self.locked = False
        self.locked_at: Optional[float] = None
        self.acknowledged: list[LockWarning] = []
        self.breaks: list[LockBreak] = []
        self.snapshot: dict[int, SealedAnchor] = {}
        #: anchors outside the seal that already have a membership break
        self.outsiders: set[int] = set()
        #: why the last save failed, None after a successful one
        self.persist_error: Optional[str] = None
        #: last values seen per sealed anchor
        self._seen: dict[int, dict[str, Any]] = {}
        #: sealed members not rediscovered since a restart: their deadline
        #: on the extension's monotonic clock
        self._awaiting: dict[int, float] = {}

    def json(self) -> dict[str, Any]:
        return {
            "locked": self.locked,
            "lockedAt": self.locked_at,
            "anchors": sorted(self.snapshot),
            "acknowledgedWarnings": list(self.acknowledged),
            "breaks": list(self.breaks),
            "persistError": self.persist_error,
        }

    def covers(self, system_id: int) -> bool:
        return self.locked and system_id in self.snapshot

    def name(self, system_id: int) -> str:
        return self.snapshot[system_id]["name"]

    def seal(
        self,
        snapshot: dict[int, SealedAnchor],
        acknowledged: list[LockWarning],
        now: float,
        uptime_at: Optional[dict[int, Optional[float]]] = None,
    ) -> None:
        """Lock (or re-seal) the cell: a fresh snapshot, no breaks. Raises
        OSError, keeping the previous state, when it cannot be saved.
        ``uptime_at`` gives the host time of each sealed uptime sample."""
        seen = _seen_from(snapshot, uptime_at or {})
        self._write(_state(True, now, acknowledged, [], snapshot, set(), seen))
        self.locked, self.locked_at = True, now
        self.snapshot, self.acknowledged, self.breaks = snapshot, acknowledged, []
        self.outsiders, self._seen, self._awaiting = set(), seen, {}

    def unseal(self) -> None:
        """Unlock; raises OSError, keeping the lock, when it cannot be saved."""
        self._write(_state(False, None, [], [], {}, set(), {}))
        self.locked, self.locked_at = False, None
        self.snapshot, self.acknowledged, self.breaks = {}, [], []
        self.outsiders, self._seen, self._awaiting = set(), {}, {}

    # ---- integrity watch ----

    def on_online(self, system_id: int) -> None:
        """A sealed anchor was (re)discovered."""
        self._awaiting.pop(system_id, None)
        if self.covers(system_id):
            self._seen[system_id]["online"] = True

    def on_offline(self, system_id: int, now: float) -> list[LockBreak]:
        if not self.covers(system_id):
            return []
        self._awaiting.pop(system_id, None)
        self._seen[system_id]["online"] = False
        return self._break(
            "offline", system_id, f"{self.name(system_id)} went offline", now
        )

    def on_missing(
        self, monotonic: float, live: Iterable[int], now: float
    ) -> list[LockBreak]:
        """The expiry pass: a sealed member that was online when the state
        was saved and is not rediscovered within the device timeout after a
        restart went offline while the server was down."""
        live = set(live)
        found: list[LockBreak] = []
        for system_id, deadline in list(self._awaiting.items()):
            if system_id in live:
                self._awaiting.pop(system_id)
            elif monotonic >= deadline:
                self._awaiting.pop(system_id)
                self._seen[system_id]["online"] = False
                found += self._break(
                    "offline",
                    system_id,
                    f"{self.name(system_id)} did not return after the server restarted",
                    now,
                )
        return found

    def on_anchor(self, system_id: int, name: str, now: float) -> list[LockBreak]:
        """An anchor outside the seal appeared while locked: once per anchor."""
        if not self.locked or system_id in self.snapshot or system_id in self.outsiders:
            return []
        self.outsiders.add(system_id)
        return self._break(
            "membership", system_id, f"{name} is not part of the sealed cell", now
        )

    def on_stats(
        self, system_id: int, stats: dict[str, Any], now: float
    ) -> list[LockBreak]:
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        found: list[LockBreak] = []
        boots = stats.get("bootCount")
        if boots is not None:
            # the boot count only stands in for the uptime it cannot see
            # (it also stays 0 across cold boots)
            if (
                seen["uptimeAt"] is None
                and seen["bootCount"] is not None
                and boots != seen["bootCount"]
            ):
                found += self._rebooted(
                    system_id, f"boot count {seen['bootCount']} -> {boots}", now
                )
            seen["bootCount"] = boots
        if "moved" in stats:
            if stats["moved"] and not seen["moved"]:
                found += self._break(
                    "moved", system_id, f"{self.name(system_id)} moved", now
                )
            seen["moved"] = stats["moved"]
        state = stats.get("powerState")
        if state is not None:
            if seen["powerState"] == "operating" and state != "operating":
                found += self._break(
                    "left-operating",
                    system_id,
                    f"{self.name(system_id)} left operating ({state})",
                    now,
                )
            seen["powerState"] = state
        return found

    def on_uptime(
        self, system_id: int, uptime_ms: int, monotonic: float, now: float
    ) -> list[LockBreak]:
        """A reboot is an uptime that did not advance with the host time
        since the previous sample (counted modulo 2**32, so the 49.7-day
        wrap is no reboot)."""
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        previous, previous_at = seen["uptimeMs"], seen["uptimeAt"]
        seen["uptimeMs"], seen["uptimeAt"] = uptime_ms, monotonic
        if previous is None or previous_at is None:
            return []
        advance = (uptime_ms - previous) & 0xFFFFFFFF
        elapsed = (monotonic - previous_at) * 1000.0
        if abs(advance - elapsed) <= UPTIME_SLACK_MS:
            return []
        return self._rebooted(system_id, "uptime reset", now)

    def on_param(
        self, system_id: int, name: str, value: Any, now: float
    ) -> list[LockBreak]:
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        label = self.name(system_id)
        if name == FIRMWARE_PARAM:
            previous, seen["firmware"] = seen["firmware"], value
            if previous is None or previous == value:
                return []
            return self._break(
                "firmware-changed",
                system_id,
                f"{label} firmware {previous} -> {value}",
                now,
            )
        if not is_cell_param(name):
            return []
        params = seen["params"]
        known, previous = name in params, params.get(name)
        params[name] = value
        if not known or previous == value:
            return []
        return self._break(
            "param-changed", system_id, f"{label} {name} {previous} -> {value}", now
        )

    def _rebooted(self, system_id: int, why: str, now: float) -> list[LockBreak]:
        return self._break(
            "rebooted", system_id, f"{self.name(system_id)} rebooted ({why})", now
        )

    def _break(
        self, code: str, system_id: int, detail: str, now: float
    ) -> list[LockBreak]:
        entry: LockBreak = {
            "code": code,
            "device": system_id,
            "detail": detail,
            "at": now,
        }
        self.breaks.append(entry)
        self._persist()
        return [entry]

    # ---- persistence ----

    def _persist(self) -> None:
        """Saves the current state; a failure is recorded, not raised (the
        receive loop must keep running and the break must still be pushed)."""
        try:
            self._write(
                _state(
                    self.locked,
                    self.locked_at,
                    self.acknowledged,
                    self.breaks,
                    self.snapshot,
                    self.outsiders,
                    self._seen,
                )
            )
        except OSError as ex:
            self.persist_error = str(ex) or type(ex).__name__

    def _write(self, state: dict[str, Any]) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(state, indent=2))
        temporary.replace(self.path)
        self.persist_error = None

    def load(self, device_timeout: float, monotonic: float) -> None:
        """Restores the persisted state; an absent file is an unlocked cell.
        Sealed members that were online must be rediscovered within
        ``device_timeout`` seconds of ``monotonic`` (see :meth:`on_missing`)."""
        if self.path is None or not self.path.exists():
            return
        state = json.loads(self.path.read_text())
        self.locked = bool(state.get("locked"))
        self.locked_at = state.get("lockedAt")
        self.acknowledged = list(state.get("acknowledgedWarnings", []))
        self.breaks = list(state.get("breaks", []))
        self.snapshot = {int(k): v for k, v in state.get("snapshot", {}).items()}
        self.outsiders = {int(k) for k in state.get("outsiders", [])}
        seen = {int(k): v for k, v in state.get("lastSeen", {}).items()}
        self._seen = seen or _seen_from(self.snapshot, {})
        for values in self._seen.values():
            # host times do not survive a restart
            values["uptimeAt"] = None
        self._awaiting = {
            system_id: monotonic + device_timeout
            for system_id, values in self._seen.items()
            if self.locked and values.get("online")
        }


def _seen_from(
    snapshot: dict[int, SealedAnchor], uptime_at: dict[int, Optional[float]]
) -> dict[int, dict[str, Any]]:
    return {
        system_id: {
            "firmware": entry["firmware"],
            "params": dict(entry["params"]),
            "bootCount": entry["bootCount"],
            "uptimeMs": entry["uptimeMs"],
            "uptimeAt": uptime_at.get(system_id)
            if entry["uptimeMs"] is not None
            else None,
            "powerState": entry["powerState"],
            "moved": entry["moved"],
            "online": entry["online"],
        }
        for system_id, entry in snapshot.items()
    }


def _state(
    locked: bool,
    locked_at: Optional[float],
    acknowledged: list[LockWarning],
    breaks: list[LockBreak],
    snapshot: dict[int, SealedAnchor],
    outsiders: set[int],
    seen: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    return {
        "locked": locked,
        "lockedAt": locked_at,
        "acknowledgedWarnings": acknowledged,
        "breaks": breaks,
        "snapshot": {str(k): v for k, v in snapshot.items()},
        "outsiders": sorted(outsiders),
        "lastSeen": {
            str(k): {name: v for name, v in values.items() if name != "uptimeAt"}
            for k, values in seen.items()
        },
    }


# ---- the extension's side: views, requests, the show gate ----


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
