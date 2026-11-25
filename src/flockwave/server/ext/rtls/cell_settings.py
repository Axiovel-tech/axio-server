"""Cell settings (X-RTLS-CELL): whether each online device runs the cell's
ranging rate and TX level, and the apply that writes them.

Every anchor and tag reports the code of the session it runs as ``cellcfg``
(``rtlslink.cell_settings.code``), an anchor also its running TX level as
``txgain``; both are null while no session runs. A write applies on the
device at once (the firmware restarts its UWB session), or is refused in the
ack when the session would not run. One server is one cell; its reference is
the online A0 once it reports both. A device is ``unknown`` while it reports
no session or there is no reference, ``differ`` while it runs another session
or (an anchor) another TX level than A0, and ``ok`` otherwise.

An apply writes every online tag first and, only when every tag took the
change, every anchor; nothing is rolled back, so a device left behind shows
``differ`` and the same apply again brings it in. The rules, refusals and
results: AxioStack ``contracts/rtls-client-api.md``, ``X-RTLS-CELL``.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING, Any, NamedTuple, Optional

import trio
from rtlslink import role_from_params
from rtlslink.cell_settings import RATE_PRESETS, TX_GAIN_PARAM, rate_hz, tx_gain_range

from flockwave.server.ext.show.extension import authorizes_flight

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "cell_rule",
    "cell_view",
    "cell_warnings",
    "handle_cell_request",
    "write_refusal",
)

_BLOCK_PARAM = "UWB_BLOCK_MS"
_PARAM_TYPES = {_BLOCK_PARAM: "uint16", TX_GAIN_PARAM: "uint8"}
_READ_TIMEOUT = 2.0

#: how a lock warning, a Verify finding and Control name a device's state
_STATE_DETAILS = {
    "differ": "different cell settings",
    "unknown": "cell settings not reported",
}
#: ``(state, detail)`` of a device
_State = tuple[str, Optional[str]]


class _Member(NamedTuple):
    """An online device as the cell settings see it."""

    system_id: int
    name: str
    #: from the device's own ``UWB_ROLE``; None until it is cached
    role: Optional[str]
    anchor: bool
    params: dict[str, Any]
    #: the code of the session it runs (``cellcfg``); None while none runs
    running: Optional[int]
    #: an anchor's running TX level (``txgain``); None while none runs
    running_tx: Optional[int]


def _members(ext: "RtlsExtension") -> list[_Member]:
    """The online devices by system id, disabled ones left out. The role is
    each device's own; retained parameters only fill in its display name."""
    from .extension import ANCHOR_ROLES, _decoded_device_params, _device_name

    devices = ext._protocol.devices if ext._protocol else {}
    members = []
    for device in sorted(devices.values(), key=lambda d: d.system_id):
        sid = device.system_id
        params = _decoded_device_params(device)
        role = role_from_params(params)
        if role == "disabled":
            continue
        identity = {**ext._retained_params.get(sid, {}), **params}
        name = _device_name(sid, identity, ext._device_role(sid)) or str(sid)
        stats = ext._fresh_stats(sid) or {}
        running, tx = stats.get("cellConfig"), stats.get("txGain")
        members.append(
            _Member(sid, name, role, role in ANCHOR_ROLES, params, running, tx)
        )
    return members


def _reference(members: list[_Member]) -> Optional[_Member]:
    """The online A0 once it reports the session and TX level it runs."""
    for m in members:
        if m.role == "anchor-initiator" and None not in (m.running, m.running_tx):
            return m
    return None


def _state(member: _Member, reference: Optional[_Member]) -> _State:
    if member.role is None:
        return "unknown", "role not read yet"
    tx_missing = member.anchor and member.running_tx is None
    if reference is None or member.running is None or tx_missing:
        return "unknown", None
    if member.running != reference.running:
        return "differ", None
    if member.anchor and member.running_tx != reference.running_tx:
        return "differ", "different TX level"
    return "ok", None


def cell_view(ext: "RtlsExtension") -> tuple[dict[str, Any], dict[int, _State]]:
    """The ``cellSettings`` summary (A0's rate and block from its parameters,
    the TX level it runs, its system id, the system ids not ``ok``) and every
    online device's state by system id, from one snapshot."""
    members = _members(ext)
    reference = _reference(members)
    states = {m.system_id: _state(m, reference) for m in members}
    params = reference.params if reference is not None else {}
    block = params.get(_BLOCK_PARAM)
    summary = {
        "rateHz": rate_hz(params),
        "blockMs": None if block is None else int(block),
        "txGain": reference.running_tx if reference is not None else None,
        "reference": reference.system_id if reference is not None else None,
        **{
            key: [sid for sid, (state, _) in states.items() if state == key]
            for key in ("differ", "unknown")
        },
    }
    return summary, states


def _not_ok(ext: "RtlsExtension") -> list[tuple[Optional[int], str]]:
    """``(system id, "<name>: <what>")`` of every online device that is not
    ``ok``; without a reference a single ``(None, why)`` for the cell."""
    members = _members(ext)
    reference = _reference(members)
    if reference is None:
        return [(None, "Cell settings: no A0 to compare")]
    found = []
    for member in members:
        state, detail = _state(member, reference)
        if state != "ok":
            text = detail or _STATE_DETAILS[state]
            found.append((member.system_id, f"{member.name}: {text}"))
    return found


def cell_warnings(ext: "RtlsExtension") -> list[dict[str, Any]]:
    """The lock preflight's acknowledgeable ``cell-settings`` warnings."""
    return [
        {"code": "cell-settings", "device": system_id, "detail": text}
        for system_id, text in _not_ok(ext)
    ]


def cell_rule(ext: "RtlsExtension") -> dict[str, Any]:
    """The X-RTLS-VERIFY ``cell`` rule: an error while any online device is
    not ``ok`` or there is no reference."""
    problems = [text for _, text in _not_ok(ext)]
    return {
        "id": "cell",
        "label": "Cell settings",
        "severity": "error",
        "status": "fail" if problems else "pass",
        "detail": "; ".join(problems) or "every device runs the cell's settings",
    }


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _request(body: dict[str, Any]) -> dict[str, int]:
    """What the request names: the preset's block, the TX level.

    Raises:
        ValueError: for a rate that is not a preset or a TX level out of range.
    """
    named = {}
    if "rateHz" in body:
        rate = body["rateHz"]
        if not _is_int(rate) or rate not in RATE_PRESETS:
            raise ValueError(f"Unknown ranging rate: {rate!r}")
        named[_BLOCK_PARAM] = RATE_PRESETS[rate]
    if "txGain" in body:
        tx = body["txGain"]
        low, high, _ = tx_gain_range()
        if not _is_int(tx) or not low <= tx <= high:
            raise ValueError(f"TX level out of range: {tx!r}")
        named[TX_GAIN_PARAM] = tx
    return named


def write_refusal(ext: "RtlsExtension") -> Optional[str]:
    """Why neither cell settings nor a survey may be written now: a live
    device reports its lock, or a show is authorized. The lock counts
    whatever else is known of the device: an anchor that was lost reports it
    again before its role is read."""
    devices = ext._protocol.devices if ext._protocol else {}
    if any((ext._fresh_stats(sid) or {}).get("locked") for sid in devices):
        return "Unlock the anchors first"
    if ext.app is not None:
        with suppress(AttributeError, KeyError, RuntimeError):
            if authorizes_flight(ext.app.import_api("show").get_configuration()):
                return "A show is authorized"
    return None


def _refusal(ext: "RtlsExtension", members: list[_Member]) -> Optional[str]:
    """Why nothing may be written now, if anything."""
    if not members:
        return "No online devices"
    reason = write_refusal(ext)
    if reason is not None:
        return reason
    if _reference(members) is None:
        a0_online = any(m.role == "anchor-initiator" for m in members)
        return "A0 is not ranging" if a0_online else "A0 is offline"
    unknown = [m.name for m in members if m.role is None]
    return f"{', '.join(unknown)}: role not read yet" if unknown else None


async def _write(
    ext: "RtlsExtension", member: _Member, target: dict[str, int]
) -> dict[str, Any]:
    """Writes what ``member`` takes of ``target`` (a tag: no TX level),
    stopping at the first failure; returns its ``results`` entry. Every
    value is written: the firmware ignores a write of the value it has."""
    for name, value in target.items():
        if name == TX_GAIN_PARAM and not member.anchor:
            continue
        try:
            ack = await ext.set_param(
                member.system_id, name, value, _PARAM_TYPES[name], exact=True
            )
        except trio.TooSlowError:
            failure = f"{member.name}: no answer to {name}={value}"
        except KeyError:
            failure = f"{member.name}: offline"
        else:
            failure = None
            if not ack["accepted"]:
                failure = f"{member.name} refused {name}={value}"
        if failure is not None:
            return {"written": "failed", "detail": failure}
    return {"written": "changed"}


async def _write_all(
    ext: "RtlsExtension", members: list[_Member], target: dict[str, int]
) -> tuple[dict[str, dict[str, Any]], bool]:
    """Writes ``members`` all at once: their results, whether every one took
    it."""
    results = {}

    async def write(member: _Member) -> None:
        results[str(member.system_id)] = await _write(ext, member, target)

    async with trio.open_nursery() as nursery:
        for member in members:
            nursery.start_soon(write, member)
    return results, all(entry["written"] != "failed" for entry in results.values())


async def _apply(
    ext: "RtlsExtension", named: dict[str, int]
) -> tuple[str, Optional[str], dict[str, dict[str, Any]]]:
    """One apply; ``(result, reason, results)``."""

    async def read(system_id: int, name: str) -> None:
        with suppress(KeyError, trio.TooSlowError):
            await ext.get_param(system_id, name, timeout=_READ_TIMEOUT)

    async def prepare(member: _Member) -> None:
        """Reads the role if it is missing, then A0's block if the request
        leaves the rate out and it is not cached."""
        if member.role is None:
            await read(member.system_id, "UWB_ROLE")
            member = next(
                (m for m in _members(ext) if m.system_id == member.system_id), member
            )
        if member.role == "anchor-initiator" and not (
            _BLOCK_PARAM in named or _BLOCK_PARAM in member.params
        ):
            await read(member.system_id, _BLOCK_PARAM)

    async with trio.open_nursery() as nursery:
        for member in _members(ext):
            nursery.start_soon(prepare, member)
    members = _members(ext)
    reason = _refusal(ext, members)
    if reason is not None:
        return "refused", reason, {}
    reference = _reference(members)
    assert reference is not None and reference.running_tx is not None
    # what the request leaves out is A0's
    block = named.get(_BLOCK_PARAM, reference.params.get(_BLOCK_PARAM))
    if block is None:
        return "refused", "A0's settings not read yet", {}
    tx = named.get(TX_GAIN_PARAM, reference.running_tx)
    target = {_BLOCK_PARAM: int(block), TX_GAIN_PARAM: tx}
    results, ok = await _write_all(ext, [m for m in members if not m.anchor], target)
    if not ok:
        reason = "Not every tag took the change; no anchor was written"
        return "incomplete", reason, results
    anchors, ok = await _write_all(ext, [m for m in members if m.anchor], target)
    if not ok:
        return "incomplete", "Not every anchor took the change", results | anchors
    return "applied", None, results | anchors


async def handle_cell_request(
    ext: "RtlsExtension", body: dict[str, Any]
) -> dict[str, Any]:
    """The X-RTLS-CELL response body: the apply's result, its reason, the
    per-device results and the cell's settings after it.

    Raises:
        ValueError: for an invalid request (see :func:`_request`).
    """
    named = _request(body)
    if ext._cell_applying:
        result, reason, results = "refused", "An apply is running", {}
    elif ext._survey.running:
        result, reason, results = "refused", "A survey is running", {}
    else:
        ext._cell_applying = True
        try:
            result, reason, results = await _apply(ext, named)
        finally:
            ext._cell_applying = False
    return {
        "type": "X-RTLS-CELL",
        "result": result,
        "reason": reason,
        "results": results,
        "cellSettings": cell_view(ext)[0],
    }
