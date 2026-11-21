"""The anchor cell lock (X-RTLS-LOCK): a sealed snapshot of the anchors, and
the breaks seen since, judged from the events the extension already handles.

The lock never acts on devices. The extension refuses anchor-changing
commands while it is locked, and records a break whenever an anchor of the
snapshot changes on its own.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Optional

__all__ = ("AnchorLock", "LOCKED_REASON", "cell_warnings", "snapshot_entry")

LOCKED_REASON = "anchors are locked; unlock them in Control first"

#: the parameter that carries the firmware version
FIRMWARE_PARAM = "FW_VERSION"

#: parameters outside the ``UWB_`` session and table set that define the cell
CELL_PARAMS = frozenset(("MAV_SYS_ID", "BAT_PROFILE"))

#: ``uwbStatus`` values of a UWB start the anchor refused or could not make
#: (disabled, invalid config, hardware failure)
UWB_START_REFUSED = frozenset((2, 3, 4))


def is_cell_param(name: str) -> bool:
    """Whether a parameter defines the cell (part of the lock snapshot)."""
    return name.startswith("UWB_") or name in CELL_PARAMS


def snapshot_entry(view: dict[str, Any]) -> dict[str, Any]:
    """The sealed record of one anchor from its view (see
    :func:`cell_warnings`)."""
    params = view.get("params", {})
    stats = view.get("stats") or {}
    return {
        "name": view["name"],
        "role": view.get("role"),
        "firmware": params.get(FIRMWARE_PARAM),
        "params": {k: v for k, v in params.items() if is_cell_param(k)},
        "bootCount": stats.get("bootCount"),
        "powerState": stats.get("powerState"),
        "moved": stats.get("moved"),
        "online": bool(view.get("online")),
    }


def cell_warnings(anchors: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """What an operator acknowledges when locking the cell. Each anchor
    view carries ``systemId``, ``name``, ``role``, ``online`` and its fresh
    X-RTLS-STATS body as ``stats`` (None without one)."""
    anchors = list(anchors)
    if not anchors:
        return [_warning("no-anchors", None, "no anchors are known")]
    warnings: list[dict[str, Any]] = []
    initiators = [a for a in anchors if a.get("role") == "anchor-initiator"]
    if not any(a["online"] for a in initiators):
        warnings.append(_warning("a0-not-operating", None, "no initiator (A0) online"))
    for anchor in anchors:
        warnings.extend(_anchor_warnings(anchor))
    return warnings


def _anchor_warnings(anchor: dict[str, Any]) -> list[dict[str, Any]]:
    system_id, name = anchor["systemId"], anchor["name"]
    if not anchor["online"]:
        return [_warning("offline", system_id, f"{name} is offline")]
    stats = anchor.get("stats") or {}
    found = []
    state = stats.get("powerState")
    if anchor.get("role") == "anchor-initiator" and state not in (None, "operating"):
        found.append(("a0-not-operating", f"{name} (A0) is {state}"))
    if anchor.get("role") == "anchor-responder" and stats.get("rangingRateHz") == 0:
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


def _warning(code: str, device: Optional[int], detail: str) -> dict[str, Any]:
    return {"code": code, "device": device, "detail": detail}


class AnchorLock:
    """The lock state: whether the cell is locked, its sealed snapshot, the
    warnings acknowledged when sealing it and the breaks seen since.

    The ``on_*`` observers compare each event with the last value seen for
    that anchor (starting from the snapshot), so every change becomes one
    break; they return the new breaks."""

    def __init__(self, path: Optional[Path] = None):
        #: where the state persists; None keeps it in memory only
        self.path = path
        self.locked = False
        self.locked_at: Optional[float] = None
        self.acknowledged: list[dict[str, Any]] = []
        self.breaks: list[dict[str, Any]] = []
        self.snapshot: dict[int, dict[str, Any]] = {}
        #: last values seen per sealed anchor; rebuilt from the snapshot
        self._seen: dict[int, dict[str, Any]] = {}

    def json(self) -> dict[str, Any]:
        return {
            "locked": self.locked,
            "lockedAt": self.locked_at,
            "anchors": sorted(self.snapshot),
            "acknowledgedWarnings": list(self.acknowledged),
            "breaks": list(self.breaks),
        }

    def covers(self, system_id: int) -> bool:
        return self.locked and system_id in self.snapshot

    def seal(
        self,
        snapshot: dict[int, dict[str, Any]],
        acknowledged: list[dict[str, Any]],
        now: float,
    ) -> None:
        """Lock (or re-seal) the cell: a fresh snapshot, no breaks."""
        self.locked, self.locked_at = True, now
        self.snapshot, self.acknowledged, self.breaks = snapshot, acknowledged, []
        self._reset_seen()
        self.save()

    def unseal(self) -> None:
        self.locked, self.locked_at = False, None
        self.snapshot, self.acknowledged, self.breaks = {}, [], []
        self._seen = {}
        self.save()

    # ---- integrity watch ----

    def on_offline(self, system_id: int, now: float) -> list[dict[str, Any]]:
        if not self.covers(system_id):
            return []
        return self._break("offline", system_id, "went offline", now)

    def on_stats(
        self, system_id: int, stats: dict[str, Any], now: float
    ) -> list[dict[str, Any]]:
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        found: list[dict[str, Any]] = []
        boots = stats.get("bootCount")
        if boots is not None:
            if seen["bootCount"] is not None and boots != seen["bootCount"]:
                found += self._break(
                    "rebooted",
                    system_id,
                    f"rebooted (boot count {seen['bootCount']} -> {boots})",
                    now,
                )
            seen["bootCount"] = boots
        if "moved" in stats:
            if stats["moved"] and not seen["moved"]:
                found += self._break("moved", system_id, "moved", now)
            seen["moved"] = stats["moved"]
        state = stats.get("powerState")
        if state is not None:
            if seen["powerState"] == "operating" and state != "operating":
                found += self._break(
                    "left-operating", system_id, f"left operating ({state})", now
                )
            seen["powerState"] = state
        return found

    def on_uptime(
        self, system_id: int, uptime_ms: int, now: float
    ) -> list[dict[str, Any]]:
        """A rewound uptime is a reboot; only for an anchor that does not
        report its boot count (which is the better evidence)."""
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        previous, seen["uptimeMs"] = seen["uptimeMs"], uptime_ms
        if seen["bootCount"] is not None or previous is None or uptime_ms >= previous:
            return []
        return self._break("rebooted", system_id, "rebooted (uptime reset)", now)

    def on_param(
        self, system_id: int, name: str, value: Any, now: float
    ) -> list[dict[str, Any]]:
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        if name == FIRMWARE_PARAM:
            previous, seen["firmware"] = seen["firmware"], value
            if previous is None or previous == value:
                return []
            return self._break(
                "firmware-changed",
                system_id,
                f"firmware {previous} -> {value}",
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
            "param-changed", system_id, f"{name} {previous} -> {value}", now
        )

    def _break(
        self, code: str, system_id: int, what: str, now: float
    ) -> list[dict[str, Any]]:
        name = self.snapshot[system_id]["name"]
        entry = {
            "code": code,
            "device": system_id,
            "detail": f"{name} {what}",
            "at": now,
        }
        self.breaks.append(entry)
        self.save()
        return [entry]

    def _reset_seen(self) -> None:
        self._seen = {
            system_id: {
                "firmware": entry.get("firmware"),
                "params": dict(entry.get("params", {})),
                "bootCount": entry.get("bootCount"),
                "powerState": entry.get("powerState"),
                "moved": entry.get("moved"),
                "uptimeMs": None,
            }
            for system_id, entry in self.snapshot.items()
        }

    # ---- persistence ----

    def save(self) -> None:
        if self.path is None:
            return
        state = {
            "locked": self.locked,
            "lockedAt": self.locked_at,
            "acknowledgedWarnings": self.acknowledged,
            "breaks": self.breaks,
            "snapshot": {str(k): v for k, v in self.snapshot.items()},
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(state, indent=2))
        temporary.replace(self.path)

    def load(self) -> None:
        """Restores the persisted state; an absent file is an unlocked
        cell. The last values seen restart from the snapshot, so a change
        made while the server was down still shows as a break."""
        if self.path is None or not self.path.exists():
            return
        state = json.loads(self.path.read_text())
        self.locked = bool(state.get("locked"))
        self.locked_at = state.get("lockedAt")
        self.acknowledged = list(state.get("acknowledgedWarnings", []))
        self.breaks = list(state.get("breaks", []))
        self.snapshot = {int(k): v for k, v in state.get("snapshot", {}).items()}
        self._reset_seen()
