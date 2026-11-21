"""The anchor cell lock (X-RTLS-LOCK): a sealed snapshot of the anchors, and
the breaks seen since, judged from the events the extension already handles.

The lock never acts on devices. The extension refuses anchor-changing
commands while it is locked and records a break whenever an anchor of the
snapshot changes on its own; :mod:`.anchor_lock_gate` holds the operator's
side (views, warnings, requests) and the show gate.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Optional, TypedDict, cast

__all__ = (
    "AnchorLock",
    "LOCKED_REASON",
    "LockBreak",
    "LockWarning",
    "ROLE_UNKNOWN_REASON",
    "SealedAnchor",
)

LOCKED_REASON = "anchors are locked; unlock them in Control first"
ROLE_UNKNOWN_REASON = "role unknown while anchors are locked; not commanded"

#: the parameter that carries the firmware version
FIRMWARE_PARAM = "FW_VERSION"

#: parameters outside the ``UWB_`` session and table set that define the cell
CELL_PARAMS = frozenset(("MAV_SYS_ID", "BAT_PROFILE"))


#: how far an uptime may fall behind the host time elapsed between two of
#: its samples before it counts as a reboot, in ms. A late advert makes the
#: host time run ahead by its delay; a reboot keeps an anchor off the network
#: for 8-15 s.
UPTIME_SLACK_MS = 5000


class LockWarning(TypedDict):
    code: str
    device: Optional[int]
    detail: str


class LockBreak(TypedDict):
    code: str
    device: int
    detail: str
    at: float


class SealedAnchor(TypedDict):
    name: str
    role: Optional[str]
    firmware: Optional[str]
    params: dict[str, Any]
    bootCount: Optional[int]
    uptimeMs: Optional[int]
    powerState: Optional[str]
    moved: Optional[bool]
    online: bool


class LastSeen(TypedDict):
    """The last values seen of a sealed anchor, which its next reports are
    compared with."""

    firmware: Optional[str]
    params: dict[str, Any]
    bootCount: Optional[int]
    uptimeMs: Optional[int]
    #: monotonic receive time of ``uptimeMs``; not persisted
    uptimeAt: Optional[float]
    powerState: Optional[str]
    moved: Optional[bool]
    online: bool


def is_cell_param(name: str) -> bool:
    """Whether a parameter defines the cell (part of the lock snapshot)."""
    return name.startswith("UWB_") or name in CELL_PARAMS


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
        self._seen: dict[int, LastSeen] = {}
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
        """A reboot is an uptime that went back (counted modulo 2**32, so
        the 49.7-day wrap is no reboot) or fell behind the host time since
        the previous sample by more than UPTIME_SLACK_MS. An uptime that runs
        ahead (the previous advert arrived late) is no reboot."""
        if not self.covers(system_id):
            return []
        seen = self._seen[system_id]
        previous, previous_at = seen["uptimeMs"], seen["uptimeAt"]
        seen["uptimeMs"], seen["uptimeAt"] = uptime_ms, monotonic
        if previous is None or previous_at is None:
            return []
        advance = (uptime_ms - previous) & 0xFFFFFFFF
        elapsed = (monotonic - previous_at) * 1000.0
        rewound = advance >= 2**31
        if not rewound and advance >= elapsed - UPTIME_SLACK_MS:
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
        self.breaks.insert(0, entry)  # newest first
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
        seen = {
            int(k): cast(LastSeen, {**v, "uptimeAt": None})
            for k, v in state.get("lastSeen", {}).items()
        }
        self._seen = seen or _seen_from(self.snapshot, {})
        self._awaiting = {
            system_id: monotonic + device_timeout
            for system_id, values in self._seen.items()
            if self.locked and values.get("online")
        }


def _seen_from(
    snapshot: dict[int, SealedAnchor], uptime_at: dict[int, Optional[float]]
) -> dict[int, LastSeen]:
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
    seen: dict[int, LastSeen],
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
