"""Cell settings (X-RTLS-CELL): the ranging rate every device of the cell runs
and the anchors' TX level.

Every anchor and tag reads its session settings once at boot and reports the
code of the session it runs as the ``cellcfg`` stat; the SDK computes the same
code from a device's parameters (``rtlslink.cell_settings``). The cell's
reference is the online initiator (A0): a device is ``ok`` when it stores and
runs A0's stored settings, ``restart`` when it stores them but runs others,
``differ`` when it stores others (or, an anchor, another TX level) and
``unknown`` while either code is missing. Nothing is stored here: the state is
derived from the cached parameters and stats. One server is one cell.

Apply writes ``UWB_BLOCK_MS`` (and ``UWB_TX_GAIN`` on anchors) to every online
device that differs, rolls every write back when one is refused or
unanswered, then reboots the running devices that need it over SMP, A0 last.
A device asleep or in standby is written, not rebooted: its wake reboots it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink.cell_settings import (
    RATE_PARAM,
    RATE_PRESETS,
    TX_GAIN_PARAM,
    code,
    rate_hz,
    tx_gain_range,
)

from flockwave.server.ext.show.config import AuthorizationScope

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "cell_json",
    "cell_rule",
    "cell_warnings",
    "device_states",
    "handle_cell_request",
)

_PARAM_TYPES = {RATE_PARAM: "uint16", TX_GAIN_PARAM: "uint8"}

#: how a lock warning, a Verify finding and Control name a device's state
_STATE_DETAILS = {
    "differ": "different cell settings",
    "restart": "restart needed",
    "unknown": "cell settings not reported",
}
_TX_DETAIL = "different TX level"


@dataclass(frozen=True)
class _Member:
    """An online anchor or tag as the cell settings see it."""

    system_id: int
    name: str
    role: str
    address: tuple[str, int]
    params: dict[str, Any]
    running: Optional[int]
    awake: bool

    @property
    def anchor(self) -> bool:
        return self.role != "tag"

    @property
    def stored(self) -> Optional[int]:
        return code(self.params)

    @property
    def tx_gain(self) -> Optional[int]:
        value = self.params.get(TX_GAIN_PARAM) if self.anchor else None
        return None if value is None else int(value)


def _members(ext: "RtlsExtension") -> list[_Member]:
    from .extension import ANCHOR_ROLES, _decoded_device_params, _device_name

    devices = ext._protocol.devices if ext._protocol else {}
    members = []
    for device in sorted(devices.values(), key=lambda d: d.system_id):
        identity = ext._identity_params(device)
        role = ext._device_role(device.system_id)
        if role not in ("tag", *ANCHOR_ROLES):
            continue
        stats = ext._fresh_stats(device.system_id) or {}
        running = stats.get("cellConfig")
        if role == "tag":
            awake = not getattr(device, "sleeping", False)
        else:
            awake = stats.get("powerState") in (None, "operating")
        members.append(
            _Member(
                system_id=device.system_id,
                name=_device_name(device.system_id, identity, role)
                or str(device.system_id),
                role=role,
                address=tuple(device.address),
                params=_decoded_device_params(device),
                running=None if running is None else int(running),
                awake=awake,
            )
        )
    return members


def _reference(members: list[_Member]) -> Optional[_Member]:
    return next((m for m in members if m.role == "anchor-initiator"), None)


def _state(member: _Member, reference: Optional[_Member]) -> tuple[str, Optional[str]]:
    target = reference.stored if reference is not None else None
    if target is None or member.stored is None:
        return "unknown", None
    if member.stored != target:
        return "differ", None
    reference_tx = reference.tx_gain if reference is not None else None
    if member.anchor and None not in (member.tx_gain, reference_tx):
        if member.tx_gain != reference_tx:
            return "differ", _TX_DETAIL
    if member.running is None:
        return "unknown", None
    return ("ok" if member.running == target else "restart"), None


def device_states(ext: "RtlsExtension") -> dict[int, tuple[str, Optional[str]]]:
    """``{system id: (state, detail)}`` of every online anchor and tag."""
    members = _members(ext)
    reference = _reference(members)
    return {m.system_id: _state(m, reference) for m in members}


def _summary(members: list[_Member]) -> dict[str, Any]:
    reference = _reference(members)
    states = {m.system_id: _state(m, reference)[0] for m in members}
    block = reference.params.get(RATE_PARAM) if reference is not None else None
    return {
        "rateHz": rate_hz(block),
        "blockMs": None if block is None else int(block),
        "txGain": reference.tx_gain if reference is not None else None,
        "reference": reference.system_id if reference is not None else None,
        **{
            key: [sid for sid, state in states.items() if state == key]
            for key in ("differ", "restart", "unknown")
        },
    }


def cell_json(ext: "RtlsExtension") -> dict[str, Any]:
    """``cellSettings`` as X-RTLS-INF and X-RTLS-CELL carry it."""
    return _summary(_members(ext))


def _not_ok(ext: "RtlsExtension") -> list[tuple[int, str, str]]:
    """``(system id, state, "<name>: <what>")`` of every online device that is not ok."""
    members = _members(ext)
    reference = _reference(members)
    found = []
    for member in members:
        state, detail = _state(member, reference)
        if state != "ok":
            text = f"{member.name}: {detail or _STATE_DETAILS[state]}"
            found.append((member.system_id, state, text))
    return found


def cell_warnings(ext: "RtlsExtension") -> list[dict[str, Any]]:
    """Lock preflight warnings: a device that differs or needs a restart."""
    return [
        {"code": "cell-settings", "device": system_id, "detail": text}
        for system_id, state, text in _not_ok(ext)
        if state != "unknown"
    ]


def cell_rule(ext: "RtlsExtension") -> dict[str, Any]:
    """The X-RTLS-VERIFY ``cell`` rule: an error while any online device is
    not running the cell's settings."""
    problems = [text for _, _, text in _not_ok(ext)]
    return {
        "id": "cell",
        "label": "Cell settings",
        "severity": "error",
        "status": "fail" if problems else "pass",
        "detail": "; ".join(problems) or "every device runs the cell's settings",
    }


def _show_authorized(ext: "RtlsExtension") -> bool:
    if ext.app is None:
        return False
    try:
        config = ext.app.import_api("show").get_configuration()
    except (AttributeError, KeyError, RuntimeError):
        return False
    return bool(config.authorized_to_start) and config.authorization_scope in (
        AuthorizationScope.LIVE,
        AuthorizationScope.REHEARSAL,
    )


def _refusal(ext: "RtlsExtension", members: list[_Member]) -> Optional[str]:
    if any(
        m.anchor and (ext._fresh_stats(m.system_id) or {}).get("locked")
        for m in members
    ):
        return "Unlock the anchors first"
    if _show_authorized(ext):
        return "A show is authorized"
    return None


def _targets(
    body: dict[str, Any], reference: Optional[_Member]
) -> tuple[int, Optional[int]]:
    """The requested ``(UWB_BLOCK_MS, UWB_TX_GAIN)``; the reference's for what
    the request leaves out. Raises ValueError for an invalid request."""
    rate, tx = body.get("rateHz"), body.get("txGain")
    if rate is not None:
        if rate not in RATE_PRESETS:
            raise ValueError(f"Unknown ranging rate: {rate!r}")
        block = RATE_PRESETS[rate]
    elif reference is not None and RATE_PARAM in reference.params:
        block = int(reference.params[RATE_PARAM])
    else:
        raise ValueError("rateHz is required while A0 is not online")
    if tx is not None:
        low, high, _ = tx_gain_range()
        if not isinstance(tx, int) or not low <= tx <= high:
            raise ValueError(f"TX level out of range: {tx!r}")
    elif reference is not None:
        tx = reference.tx_gain
    return block, tx


def _writes(member: _Member, block: int, tx: Optional[int]) -> list[tuple[str, int]]:
    writes = []
    if member.params.get(RATE_PARAM) != block:
        writes.append((RATE_PARAM, block))
    if member.anchor and tx is not None and member.tx_gain not in (None, tx):
        writes.append((TX_GAIN_PARAM, tx))
    return writes


async def _write(
    ext: "RtlsExtension", member: _Member, name: str, value: int
) -> Optional[str]:
    """None when the device took the value, else why not."""
    try:
        ack = await ext.set_param(member.system_id, name, value, _PARAM_TYPES[name])
    except (KeyError, ValueError, trio.TooSlowError) as ex:
        return str(ex) or "no answer"
    if ack["accepted"] and int(ack["value"]) == value:
        return None
    return f"{member.name} refused {name}={value}"


async def _reset_device(host: str, port: int) -> None:
    from rtlslink.ota import reset

    await trio.to_thread.run_sync(lambda: reset(host, port=port))


async def _apply(
    ext: "RtlsExtension",
    members: list[_Member],
    block: int,
    tx: Optional[int],
) -> tuple[str, Optional[str], dict[str, dict[str, Any]]]:
    from rtlslink.ota import SMP_PORT

    results: dict[str, dict[str, Any]] = {
        str(m.system_id): {"written": "unchanged", "restarted": False} for m in members
    }
    changed: dict[int, list[tuple[str, Any]]] = {}

    async def write(member: _Member) -> None:
        for name, value in _writes(member, block, tx):
            failure = await _write(ext, member, name, value)
            if failure is not None:
                results[str(member.system_id)].update(written="failed", detail=failure)
                return
            changed.setdefault(member.system_id, []).append(
                (name, member.params.get(name))
            )
            results[str(member.system_id)]["written"] = "changed"

    async with trio.open_nursery() as nursery:
        for member in members:
            nursery.start_soon(write, member)

    by_id = {m.system_id: m for m in members}
    if any(entry["written"] == "failed" for entry in results.values()):
        await _roll_back(ext, by_id, changed, results)
        return (
            "incomplete",
            "Not every device took the change; nothing was restarted",
            results,
        )

    def needs_restart(member: _Member) -> bool:
        if member.system_id in changed:
            return True
        written = {**member.params, RATE_PARAM: block}
        return member.running is not None and member.running != code(written)

    to_reset = [m for m in members if m.awake and needs_restart(m)]
    to_reset.sort(key=lambda m: m.role == "anchor-initiator")
    failed = False

    async def restart(member: _Member) -> None:
        nonlocal failed
        port = ext._smp_ports.get(member.address, SMP_PORT)
        try:
            await _reset_device(member.address[0], port)
        except Exception as ex:  # noqa: BLE001 - one device must not stop the rest
            failed = True
            results[str(member.system_id)]["detail"] = (
                f"restart failed: {ex or type(ex).__name__}"
            )
            return
        results[str(member.system_id)]["restarted"] = True

    async with trio.open_nursery() as nursery:
        for member in to_reset:
            if member.role != "anchor-initiator":
                nursery.start_soon(restart, member)
    for member in to_reset:
        if member.role == "anchor-initiator":
            await restart(member)
    if failed:
        return "incomplete", "Not every device restarted", results
    return "applied", None, results


async def _roll_back(
    ext: "RtlsExtension",
    by_id: dict[int, _Member],
    changed: dict[int, list[tuple[str, Any]]],
    results: dict[str, dict[str, Any]],
) -> None:
    """Writes the old values back to every device this request changed."""

    async def roll_back(system_id: int, writes: list[tuple[str, Any]]) -> None:
        member, entry = by_id[system_id], results[str(system_id)]
        for name, old in reversed(writes):
            failure = None if old is None else await _write(ext, member, name, int(old))
            if old is None or failure is not None:
                entry["detail"] = f"rollback failed: {failure or name + ' was unknown'}"
                return
        entry["written"] = "unchanged"

    async with trio.open_nursery() as nursery:
        for system_id, writes in changed.items():
            nursery.start_soon(roll_back, system_id, writes)


async def handle_cell_request(
    ext: "RtlsExtension", body: dict[str, Any]
) -> dict[str, Any]:
    """The X-RTLS-CELL response body. Raises ValueError for an invalid
    request."""
    action = body.get("action", "status")
    if action not in ("status", "apply"):
        raise ValueError(f"Invalid action: {action!r} (expected status or apply)")
    members = _members(ext)
    reason = _refusal(ext, members)
    result: str = action
    results: dict[str, dict[str, Any]] = {}
    if action == "apply":
        block, tx = _targets(body, _reference(members))
        if reason is not None:
            result = "refused"
        else:
            result, reason, results = await _apply(ext, members, block, tx)
    return {
        "type": "X-RTLS-CELL",
        "action": action,
        "result": result,
        "reason": reason,
        "results": results,
        "cellSettings": cell_json(ext),
    }
