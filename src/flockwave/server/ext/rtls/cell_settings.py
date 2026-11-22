"""Cell settings apply (X-RTLS-CELL): writes a ranging rate and the anchors'
TX level to every online device of the cell and reboots the running ones onto
them.

A request names a rate preset (block, slots and slot length) and/or a TX
level; what it leaves out stays the reference's (A0, see :mod:`.cell_state`,
which also derives the per-device state X-RTLS-INF carries). One apply runs at
a time. It:

1. reads the cell parameters the online devices have not reported yet, only
   for those devices (a role read in the first pass names what a second pass
   reads);
2. right before the first write, refuses with nothing written when no device
   is online, an anchor reports its lock, a show is authorized, a device's
   parameters are still not cached, or there is no reference;
3. writes each value to every online device that stores another, asleep and
   standby devices included, in an order that keeps every stored step a
   schedule the firmware boots; each old value is recorded before its write,
   so a write whose ack is lost is still undone;
4. when any write fails, writes every old value back, each restore attempted,
   and restarts nothing (``incomplete``);
5. from the state after the writes, restarts nothing when a lock or a show
   authorization now applies; otherwise reboots over SMP every running device
   that stores the target and the reference's channel and session but runs
   something else, A0 last. Asleep and standby devices apply it at their wake.
   A written device whose run state is not known ("restart needed") and a
   device storing another channel or session than A0 are not rebooted and
   leave the result ``incomplete``.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink import ota
from rtlslink.cell_settings import (
    RATE_PRESETS,
    TX_GAIN_PARAM,
    preset,
    tx_gain_range,
)

from flockwave.server.ext.show.extension import authorizes_flight

from .cell_state import CellMember, cell_members, cell_reference, cell_view

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("handle_cell_request",)

_PARAM_TYPES = {
    "UWB_BLOCK_MS": "uint16",
    "UWB_SLOTS_RR": "uint8",
    "UWB_SLOT_RSTU": "uint16",
    TX_GAIN_PARAM: "uint8",
}
_RATE_PARAMS = ("UWB_BLOCK_MS", "UWB_SLOTS_RR", "UWB_SLOT_RSTU")
_ROUND_PARAMS = ("UWB_SLOTS_RR", "UWB_SLOT_RSTU")
_SESSION_PARAMS = ("UWB_CHANNEL", "UWB_SESSION_ID")
_READ_TIMEOUT = 2.0

#: ``(result, reason, results by system id)`` of an apply
_Outcome = tuple[str, Optional[str], dict[str, dict[str, Any]]]


def _request(body: dict[str, Any]) -> tuple[Optional[int], Optional[int]]:
    """The requested ``(rateHz, txGain)``, each None when left out.

    Raises:
        ValueError: for a rate that is not a preset or a TX level out of range.
    """
    rate, tx = body.get("rateHz"), body.get("txGain")
    if rate is not None and rate not in RATE_PRESETS:
        raise ValueError(f"Unknown ranging rate: {rate!r}")
    if tx is not None:
        low, high, _ = tx_gain_range()
        if not isinstance(tx, int) or not low <= tx <= high:
            raise ValueError(f"TX level out of range: {tx!r}")
    return rate, tx


def _refusal(ext: "RtlsExtension", members: list[CellMember]) -> Optional[str]:
    """Why the cell must not change now: an anchor that reports its lock, or a
    live or rehearsal show that is authorized."""
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


def _preflight(ext: "RtlsExtension", members: list[CellMember]) -> Optional[str]:
    """Why nothing may be written to ``members`` now, if anything: no device,
    a refusal, a device whose cell parameters are still not cached, or no
    reference."""
    if not members:
        return "No online devices"
    reason = _refusal(ext, members)
    if reason is not None:
        return reason
    unread = [m.name for m in members if m.missing]
    if unread:
        return f"{', '.join(unread)}: settings not read yet"
    if cell_reference(members) is None:
        return "A0 is offline"
    return None


def _target(
    rate: Optional[int], tx: Optional[int], reference: CellMember
) -> dict[str, int]:
    """Every parameter value the request asks of the cell: the preset and the
    TX level, the reference's stored ones for what the request leaves out."""
    if rate is not None:
        target = preset(rate)
    else:
        target = {name: int(reference.params[name]) for name in _RATE_PARAMS}
    tx = tx if tx is not None else reference.tx_gain
    if tx is not None:
        target[TX_GAIN_PARAM] = tx
    return target


def _wanted(member: CellMember, target: dict[str, int]) -> dict[str, int]:
    """What ``member`` must store: the target, without the TX level on a
    tag."""
    return {k: v for k, v in target.items() if member.anchor or k != TX_GAIN_PARAM}


def _write_order(member: CellMember, wanted: dict[str, int]) -> list[tuple[str, int]]:
    """``wanted`` in the order to write it to ``member``.

    The firmware refuses to boot a round (slots times slot length) longer than
    its block, and a device can reboot between two writes. Writing the slot
    count and slot length that shrink first, then the block, then those that
    grow keeps every stored step within its block; the rollback, which undoes
    the writes in reverse, passes through the same steps.
    """

    def step(item: tuple[str, int]) -> int:
        name, value = item
        if name not in _ROUND_PARAMS:
            return 1
        return 0 if value < member.params[name] else 2

    return sorted(wanted.items(), key=step)


def _needs_reset(
    member: CellMember, target: dict[str, int], session: dict[str, Any]
) -> Optional[bool]:
    """Whether a reboot makes ``member`` run the target.

    Returns:
        True when it is running, stores the target and the cell's channel and
        session, and runs something else (its session or, an anchor, its TX
        level); False when it is asleep or in standby, stores something else
        or already runs what it stores; None when it stores the target but
        whether or what it runs is not known.
    """
    stores = {**session, **_wanted(member, target)}
    if member.operating is False or any(
        member.params.get(k) != v for k, v in stores.items()
    ):
        return False
    if member.operating is None or member.running is None:
        return None
    stale_tx = member.anchor and member.running_tx not in (None, member.tx_gain)
    return member.running != member.stored or stale_tx


async def _write(
    ext: "RtlsExtension", member: CellMember, name: str, value: int
) -> tuple[Optional[str], bool]:
    """Writes one parameter and checks the device's ack.

    Returns:
        ``(None, False)`` when the device took the value, else ``(why not,
        whether the device certainly did not store it)``: a refusal or a lost
        device did not store it, an unanswered write may have.
    """
    try:
        ack = await ext.set_param(member.system_id, name, value, _PARAM_TYPES[name])
    except trio.TooSlowError:
        return f"{member.name}: no answer to {name}={value}", False
    except KeyError:
        return f"{member.name}: offline", True
    if not ack["accepted"]:
        return f"{member.name} refused {name}={value}", True
    if int(ack["value"]) != value:
        return f"{member.name} stored {name}={ack['value']}", False
    return None, False


async def _read_missing(ext: "RtlsExtension", members: list[CellMember]) -> None:
    """Reads every cell parameter ``members`` have not reported yet, all
    devices at once; a read that fails leaves the parameter missing."""

    async def read(member: CellMember) -> None:
        for name in member.missing:
            with suppress(KeyError, trio.TooSlowError):
                await ext.get_param(member.system_id, name, timeout=_READ_TIMEOUT)

    async with trio.open_nursery() as nursery:
        for member in members:
            nursery.start_soon(read, member)


async def _roll_back(
    ext: "RtlsExtension",
    members: list[CellMember],
    undo: dict[int, list[tuple[str, Any]]],
    results: dict[str, dict[str, Any]],
) -> None:
    """Writes the old values back to every device this request wrote or may
    have written, newest write first. Every restore is attempted; a device
    whose restores all landed reads ``unchanged`` again, every failure is
    added to its ``detail``."""
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
    ext: "RtlsExtension",
    members: list[CellMember],
    results: dict[str, dict[str, Any]],
) -> bool:
    """Reboots ``members`` over SMP, the others at once and A0 after them, so
    the responders come back to an initiator that already runs the target.
    Marks each rebooted device ``restarted``; returns False when a reboot
    failed (its ``detail`` says why)."""
    failed = False

    async def restart(member: CellMember) -> None:
        nonlocal failed
        entry = results[str(member.system_id)]
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


async def _write_all(
    ext: "RtlsExtension",
    members: list[CellMember],
    target: dict[str, int],
    results: dict[str, dict[str, Any]],
) -> dict[int, list[tuple[str, Any]]]:
    """Writes the target to every member that stores another value, all
    devices at once, each device's writes in :func:`_write_order` and stopped
    at its first failure.

    Returns:
        the ``(name, old value)`` of every write that landed or may have, by
        system id, for :func:`_roll_back`.
    """
    undo: dict[int, list[tuple[str, Any]]] = {}

    async def write(member: CellMember) -> None:
        entry = results[str(member.system_id)]
        writes = undo.setdefault(member.system_id, [])
        for name, value in _write_order(member, _wanted(member, target)):
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
    return undo


async def _restart_written(
    ext: "RtlsExtension",
    reference: CellMember,
    target: dict[str, int],
    results: dict[str, dict[str, Any]],
) -> _Outcome:
    """Step 5 of the module docstring, from a fresh snapshot of the devices
    this request covered."""
    fresh = {m.system_id: m for m in cell_members(ext) if str(m.system_id) in results}
    reason = _refusal(ext, list(fresh.values()))
    if reason is not None:
        return "incomplete", f"{reason}; nothing was restarted", results
    session = {name: reference.params[name] for name in _SESSION_PARAMS}
    plan = {sid: _needs_reset(m, target, session) for sid, m in fresh.items()}
    unconfirmed = False
    for sid, entry in results.items():
        if entry["written"] == "changed" and plan.get(int(sid)) is None:
            entry["detail"], unconfirmed = "restart needed", True
    for sid, member in fresh.items():
        if any(member.params.get(k) != v for k, v in session.items()):
            detail = "different channel or session than A0"
            results[str(sid)]["detail"], unconfirmed = detail, True
    to_reset = [fresh[sid] for sid, needed in plan.items() if needed]
    if not await _restart(ext, to_reset, results) or unconfirmed:
        return "incomplete", "Not every device restarted", results
    return "applied", None, results


async def _apply(
    ext: "RtlsExtension", rate: Optional[int], tx: Optional[int]
) -> _Outcome:
    """One apply, steps 1 to 5 of the module docstring."""
    members = cell_members(ext)
    for _ in range(2):  # a role read in the first pass names what the second reads
        incomplete = [m for m in members if m.missing]
        if not incomplete:
            break
        await _read_missing(ext, incomplete)
        members = cell_members(ext)
    reason = _preflight(ext, members)
    if reason is not None:
        return "refused", reason, {}
    reference = cell_reference(members)
    assert reference is not None
    target = _target(rate, tx, reference)

    results = {
        str(m.system_id): {"written": "unchanged", "restarted": False} for m in members
    }
    undo = await _write_all(ext, members, target, results)
    if any(entry["written"] == "failed" for entry in results.values()):
        await _roll_back(ext, members, undo, results)
        reason = "Not every device took the change; nothing was restarted"
        return "incomplete", reason, results
    return await _restart_written(ext, reference, target, results)


async def handle_cell_request(
    ext: "RtlsExtension", body: dict[str, Any]
) -> dict[str, Any]:
    """The X-RTLS-CELL response body: the apply's result, its reason, the
    per-device results and the cell's settings after it.

    Raises:
        ValueError: for an invalid request (see :func:`_request`).
    """
    rate, tx = _request(body)
    if ext._cell_applying:
        result, reason, results = "refused", "An apply is running", {}
    else:
        ext._cell_applying = True
        try:
            result, reason, results = await _apply(ext, rate, tx)
        finally:
            ext._cell_applying = False
    return {
        "type": "X-RTLS-CELL",
        "result": result,
        "reason": reason,
        "results": results,
        "cellSettings": cell_view(ext)[0],
    }
