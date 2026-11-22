"""Cell settings (X-RTLS-CELL): the ranging rate every device of the cell runs
and the anchors' TX level.

Every anchor and tag reads its session settings once at boot and reports the
code of the session it runs as the ``cellcfg`` stat (anchors also their TX
level as ``txgain``); the SDK computes the same code from a device's
parameters (``rtlslink.cell_settings``). The cell's reference is the online
initiator (A0): a device is ``restart`` while what it stores differs from what
it runs, ``ok`` when it runs what A0 runs, ``differ`` otherwise and
``unknown`` until it has reported both. Nothing is stored here: the state is
derived from the cached parameters and stats. One server is one cell.

Apply reads what it has not cached yet, writes the preset (block, slots, slot
length) and the anchors' TX level to every online device that differs, rolls
every write back when one is refused or unanswered, then, from the state after
the writes, reboots over SMP each running device that stores the target but
runs something else, A0 last. A device asleep or in standby is written, not
rebooted: its wake reboots it.
"""

from __future__ import annotations

from contextlib import suppress
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink import ota
from rtlslink.cell_settings import (
    CODE_PARAMS,
    RATE_PRESETS,
    TX_GAIN_PARAM,
    code,
    preset,
    rate_hz,
    tx_gain_range,
)

from flockwave.server.ext.show.extension import authorizes_flight

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "cell_rule",
    "cell_view",
    "cell_warnings",
    "handle_cell_request",
)

_PARAM_TYPES = {
    "UWB_BLOCK_MS": "uint16",
    "UWB_SLOTS_RR": "uint8",
    "UWB_SLOT_RSTU": "uint16",
    TX_GAIN_PARAM: "uint8",
}
_RATE_PARAMS = ("UWB_BLOCK_MS", "UWB_SLOTS_RR", "UWB_SLOT_RSTU")
_SESSION_PARAMS = ("UWB_CHANNEL", "UWB_SESSION_ID")
_READ_TIMEOUT = 2.0

#: how a lock warning, a Verify finding and Control name a device's state
_STATE_DETAILS = {
    "differ": "different cell settings",
    "restart": "restart needed",
    "unknown": "cell settings not reported",
}
_TX_DETAIL = "different TX level"


@dataclass(frozen=True)
class _Member:
    """An online device as the cell settings see it."""

    system_id: int
    name: str
    role: Optional[str]
    address: tuple[str, int]
    params: dict[str, Any]
    running: Optional[int]
    running_tx: Optional[int]
    #: an operating anchor or an awake tag: the device a reboot applies to
    operating: bool

    @property
    def anchor(self) -> bool:
        return self.role in ("anchor-initiator", "anchor-responder")

    @property
    def stored(self) -> Optional[int]:
        return code(self.params)

    @property
    def tx_gain(self) -> Optional[int]:
        value = self.params.get(TX_GAIN_PARAM) if self.anchor else None
        return None if value is None else int(value)

    @property
    def missing(self) -> list[str]:
        """The parameters the cell settings need and the cache lacks."""
        if self.role is None:
            return ["UWB_ROLE"]
        names = (*CODE_PARAMS, TX_GAIN_PARAM) if self.anchor else CODE_PARAMS
        return [name for name in names if name not in self.params]


def _members(ext: "RtlsExtension") -> list[_Member]:
    from .extension import _decoded_device_params, _device_name

    devices = ext._protocol.devices if ext._protocol else {}
    members = []
    for device in sorted(devices.values(), key=lambda d: d.system_id):
        role = ext._device_role(device.system_id)
        if role == "disabled":
            continue
        params = _decoded_device_params(device)
        identity = {**ext._retained_params.get(device.system_id, {}), **params}
        stats = ext._fresh_stats(device.system_id) or {}
        if role == "tag":
            operating = not getattr(device, "sleeping", False)
        else:
            operating = ext._power_state(device.system_id) == "operating"
        running, running_tx = stats.get("cellConfig"), stats.get("txGain")
        members.append(
            _Member(
                system_id=device.system_id,
                name=_device_name(device.system_id, identity, role)
                or str(device.system_id),
                role=role,
                address=tuple(device.address),
                params=params,
                running=None if running is None else int(running),
                running_tx=None if running_tx is None else int(running_tx),
                operating=operating,
            )
        )
    return members


def _reference(members: list[_Member]) -> Optional[_Member]:
    return next((m for m in members if m.role == "anchor-initiator"), None)


def _state(member: _Member, reference: Optional[_Member]) -> tuple[str, Optional[str]]:
    """unknown until reported; restart while it stores A0's settings and runs
    others; ok when it runs what A0 runs; differ otherwise."""
    reported = (member.running, member.stored)
    if member.anchor:
        reported += (member.running_tx, member.tx_gain)
    if member.role is None or None in reported:
        return "unknown", None
    if member.running != member.stored or (
        member.anchor and member.running_tx != member.tx_gain
    ):
        stores_reference = reference is None or (
            member.stored == reference.stored
            and (not member.anchor or member.tx_gain == reference.tx_gain)
        )
        return ("restart" if stores_reference else "differ"), None
    if reference is None or reference.running is None:
        return "unknown", None
    if member.running != reference.running:
        return "differ", None
    if member.anchor and member.running_tx != reference.running_tx:
        return "differ", _TX_DETAIL
    return "ok", None


def cell_view(
    ext: "RtlsExtension",
) -> tuple[dict[str, Any], dict[int, tuple[str, Optional[str]]]]:
    """``cellSettings`` as X-RTLS-INF and X-RTLS-CELL carry it, and the
    ``(state, detail)`` of every online device by system id, from one
    snapshot."""
    members = _members(ext)
    reference = _reference(members)
    states = {m.system_id: _state(m, reference) for m in members}
    params = reference.params if reference is not None else {}
    block = params.get("UWB_BLOCK_MS")
    summary = {
        "rateHz": rate_hz(params),
        "blockMs": None if block is None else int(block),
        "txGain": reference.tx_gain if reference is not None else None,
        "reference": reference.system_id if reference is not None else None,
        **{
            key: [sid for sid, (state, _) in states.items() if state == key]
            for key in ("differ", "restart", "unknown")
        },
    }
    return summary, states


def _not_ok(ext: "RtlsExtension") -> list[tuple[int, str]]:
    """``(system id, "<name>: <what>")`` of every online device that is not ok;
    none without A0, whose absence the lock and Verify report themselves."""
    members = _members(ext)
    reference = _reference(members)
    if reference is None:
        return []
    found = []
    for member in members:
        state, detail = _state(member, reference)
        if state != "ok":
            found.append(
                (member.system_id, f"{member.name}: {detail or _STATE_DETAILS[state]}")
            )
    return found


def cell_warnings(ext: "RtlsExtension") -> list[dict[str, Any]]:
    """Lock preflight warnings: every device that is not ok."""
    return [
        {"code": "cell-settings", "device": system_id, "detail": text}
        for system_id, text in _not_ok(ext)
    ]


def cell_rule(ext: "RtlsExtension") -> dict[str, Any]:
    """The X-RTLS-VERIFY ``cell`` rule: an error while any online device is
    not running the cell's settings."""
    problems = [text for _, text in _not_ok(ext)]
    return {
        "id": "cell",
        "label": "Cell settings",
        "severity": "error",
        "status": "fail" if problems else "pass",
        "detail": "; ".join(problems) or "every device runs the cell's settings",
    }


def _refusal(ext: "RtlsExtension", members: list[_Member]) -> Optional[str]:
    if any(
        m.anchor and (ext._fresh_stats(m.system_id) or {}).get("locked")
        for m in members
    ):
        return "Unlock the anchors first"
    if ext.app is not None:
        with suppress(AttributeError, KeyError, RuntimeError):
            if authorizes_flight(ext.app.import_api("show").get_configuration()):
                return "A show is authorized"
    return None


def _request(body: dict[str, Any]) -> tuple[Optional[int], Optional[int]]:
    """The requested ``(rateHz, txGain)``. Raises ValueError for an invalid
    request."""
    rate, tx = body.get("rateHz"), body.get("txGain")
    if rate is not None and rate not in RATE_PRESETS:
        raise ValueError(f"Unknown ranging rate: {rate!r}")
    if tx is not None:
        low, high, _ = tx_gain_range()
        if not isinstance(tx, int) or not low <= tx <= high:
            raise ValueError(f"TX level out of range: {tx!r}")
    return rate, tx


def _target(
    rate: Optional[int], tx: Optional[int], reference: Optional[_Member]
) -> dict[str, int]:
    """Every parameter value the request asks of the cell, the reference's for
    what it leaves out. Raises ValueError without a rate or a reference."""
    ref = reference.params if reference is not None else {}
    if rate is not None:
        target = preset(rate)
    elif reference is not None:
        target = {name: int(ref[name]) for name in _RATE_PARAMS}
    else:
        raise ValueError("rateHz is required while A0 is not online")
    tx = tx if tx is not None else (reference.tx_gain if reference else None)
    if tx is not None:
        target[TX_GAIN_PARAM] = tx
    return target


def _wanted(member: _Member, target: dict[str, int]) -> dict[str, int]:
    """What ``member`` must store: the target, without TX on a tag."""
    return {k: v for k, v in target.items() if member.anchor or k != TX_GAIN_PARAM}


def _needs_reset(
    member: _Member, target: dict[str, int], session: dict[str, Any]
) -> bool:
    """Whether a reboot makes ``member`` run the target: it is running, stores
    the target and the cell's channel and session, and runs something else."""
    stores = {**session, **_wanted(member, target)}
    if not member.operating or any(
        member.params.get(k) != v for k, v in stores.items()
    ):
        return False
    stale_tx = member.anchor and member.running_tx not in (None, member.tx_gain)
    return member.running not in (None, member.stored) or stale_tx


async def _write(
    ext: "RtlsExtension", member: _Member, name: str, value: int
) -> tuple[Optional[str], bool]:
    """``(None, _)`` when the device took the value, else ``(why not, whether
    the device certainly did not store it)``."""
    try:
        ack = await ext.set_param(member.system_id, name, value, _PARAM_TYPES[name])
    except trio.TooSlowError:
        return f"{member.name}: no answer to {name}={value}", False
    except (KeyError, ValueError) as ex:
        return f"{member.name}: {ex or 'offline'}", True
    if not ack["accepted"]:
        return f"{member.name} refused {name}={value}", True
    if int(ack["value"]) != value:
        return f"{member.name} stored {name}={ack['value']}", False
    return None, False


async def _read_missing(ext: "RtlsExtension", members: list[_Member]) -> None:
    """Reads every cell parameter an online device has not reported yet."""

    async def read(member: _Member) -> None:
        for name in member.missing:
            with suppress(KeyError, trio.TooSlowError):
                await ext.get_param(member.system_id, name, timeout=_READ_TIMEOUT)

    async with trio.open_nursery() as nursery:
        for member in members:
            nursery.start_soon(read, member)


async def _roll_back(
    ext: "RtlsExtension",
    members: list[_Member],
    undo: dict[int, list[tuple[str, Any]]],
    results: dict[str, dict[str, Any]],
) -> None:
    """Writes the old values back to every device this request wrote or may
    have written, each one attempted, every failure reported."""
    by_id = {m.system_id: m for m in members}

    async def roll_back(system_id: int, writes: list[tuple[str, Any]]) -> None:
        entry, failures = results[str(system_id)], []
        for name, old in reversed(writes):
            failure, _ = await _write(ext, by_id[system_id], name, int(old))
            if failure is not None:
                failures.append(failure)
        if failures:
            text = "rollback failed: " + "; ".join(failures)
            entry["detail"] = "; ".join(filter(None, (entry.get("detail"), text)))
        elif entry["written"] == "changed":
            entry["written"] = "unchanged"

    async with trio.open_nursery() as nursery:
        for system_id, writes in undo.items():
            if writes:
                nursery.start_soon(roll_back, system_id, writes)


async def _restart(
    ext: "RtlsExtension", members: list[_Member], results: dict[str, dict[str, Any]]
) -> bool:
    """Reboots ``members`` over SMP, A0 after the others; False when one failed."""
    failed = False

    async def restart(member: _Member) -> None:
        nonlocal failed
        entry = results.setdefault(
            str(member.system_id), {"written": "unchanged", "restarted": False}
        )
        port = ext._smp_ports.get(member.address, ota.SMP_PORT)
        try:
            await trio.to_thread.run_sync(
                lambda: ota.reset(member.address[0], port=port)
            )
        except Exception as ex:  # noqa: BLE001 - one device must not stop the rest
            failed = True
            entry["detail"] = f"restart failed: {ex or type(ex).__name__}"
            return
        entry["restarted"] = True

    async with trio.open_nursery() as nursery:
        for member in members:
            if member.role != "anchor-initiator":
                nursery.start_soon(restart, member)
    for member in members:
        if member.role == "anchor-initiator":
            await restart(member)
    return not failed


async def _apply(
    ext: "RtlsExtension", rate: Optional[int], tx: Optional[int]
) -> tuple[str, Optional[str], dict[str, dict[str, Any]]]:
    members = _members(ext)
    if not members:
        return "refused", "No online devices", {}
    reason = _refusal(ext, members)
    if reason is not None:
        return "refused", reason, {}
    for _ in range(2):  # a role read in the first pass names what the second reads
        await _read_missing(ext, members)
        members = _members(ext)
    unread = [m.name for m in members if m.missing]
    if unread:
        return "refused", f"{', '.join(unread)}: settings not read yet", {}
    reference = _reference(members)
    target = _target(rate, tx, reference)

    results = {
        str(m.system_id): {"written": "unchanged", "restarted": False} for m in members
    }
    undo: dict[int, list[tuple[str, Any]]] = {}

    async def write(member: _Member) -> None:
        entry, writes = (
            results[str(member.system_id)],
            undo.setdefault(member.system_id, []),
        )
        for name, value in _wanted(member, target).items():
            if member.params[name] == value:
                continue
            writes.append((name, member.params[name]))
            failure, not_stored = await _write(ext, member, name, value)
            if failure is None:
                entry["written"] = "changed"
                continue
            entry.update(written="failed", detail=failure)
            if not_stored:
                writes.pop()
            return

    async with trio.open_nursery() as nursery:
        for member in members:
            nursery.start_soon(write, member)
    if any(entry["written"] == "failed" for entry in results.values()):
        await _roll_back(ext, members, undo, results)
        return (
            "incomplete",
            "Not every device took the change; nothing was restarted",
            results,
        )

    fresh = _members(ext)
    reason = _refusal(ext, fresh)
    if reason is not None:
        return "incomplete", f"{reason}; nothing was restarted", results
    session = (
        {name: reference.params[name] for name in _SESSION_PARAMS} if reference else {}
    )
    to_reset = [m for m in fresh if _needs_reset(m, target, session)]
    if not await _restart(ext, to_reset, results):
        return "incomplete", "Not every device restarted", results
    return "applied", None, results


async def handle_cell_request(
    ext: "RtlsExtension", body: dict[str, Any]
) -> dict[str, Any]:
    """The X-RTLS-CELL response body. Raises ValueError for an invalid
    request."""
    action = body.get("action", "status")
    if action not in ("status", "apply"):
        raise ValueError(f"Invalid action: {action!r} (expected status or apply)")
    result, reason, results = action, None, {}
    if action == "apply":
        rate, tx = _request(body)
        if ext._cell_applying:
            result, reason = "refused", "An apply is running"
        else:
            ext._cell_applying = True
            try:
                result, reason, results = await _apply(ext, rate, tx)
            finally:
                ext._cell_applying = False
    return {
        "type": "X-RTLS-CELL",
        "action": action,
        "result": result,
        "reason": reason,
        "results": results,
        "cellSettings": cell_view(ext)[0],
    }
