"""Cell settings state: whether each online device runs the cell's ranging
rate and TX level.

Every anchor and tag reports the code of the session it booted with as the
``cellcfg`` stat, and an anchor also its TX level as ``txgain``. The SDK
computes the same code from a device's parameters (``rtlslink.cell_settings``),
so comparing the two tells whether a device runs what it stores. The cell's
reference is the online initiator (A0) once it reports ``cellcfg``. One server
is one cell.

A device's state is the first of these rules that matches:

1. ``unknown``: it has not reported what it runs (an anchor: or its TX
   level), or a cell parameter it needs, its role included, is not cached.
2. ``restart``: it stores other settings than it runs, and what it stores
   equals what A0 stores (with no reference: whenever the two differ). Its
   next reboot makes it run them.
3. ``differ``: it stores other settings than it runs, and not A0's.
4. ``unknown``: there is no reference to compare with.
5. ``differ``: it runs another session than A0 runs.
6. ``unknown``: an anchor, while A0 reports no TX level.
7. ``differ``, detail "different TX level": an anchor whose running TX level
   is not A0's.
8. ``ok`` otherwise.

Nothing is stored: the state is derived on each call from the cached
parameters and the fresh stats. X-RTLS-INF carries it per device and as the
``cellSettings`` summary; the lock warns and Verify fails on every device that
is not ``ok``, or once for the whole cell when there is no reference.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

from rtlslink.cell_settings import CODE_PARAMS, TX_GAIN_PARAM, code, rate_hz

from .cell_compat import role_from_params

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "CellMember",
    "cell_members",
    "cell_reference",
    "cell_rule",
    "cell_view",
    "cell_warnings",
)

#: how a lock warning, a Verify finding and Control name a device's state
_STATE_DETAILS = {
    "differ": "different cell settings",
    "restart": "restart needed",
    "unknown": "cell settings not reported",
}
_TX_DETAIL = "different TX level"
_NO_REFERENCE = "Cell settings: no A0 to compare"


@dataclass(frozen=True)
class CellMember:
    """An online device as the cell settings see it: its own cached
    parameters and the session it reports running."""

    system_id: int
    name: str
    #: the role from the device's own ``UWB_ROLE``; None until it is cached
    role: Optional[str]
    address: tuple[str, int]
    params: dict[str, Any]
    #: the code of the session it runs (``cellcfg``); None until reported
    running: Optional[int]
    #: an anchor's running TX level (``txgain``); None until reported
    running_tx: Optional[int]
    #: an operating anchor or an awake tag: the device a reboot applies to;
    #: None for an anchor that has not reported its power state
    operating: Optional[bool]

    @property
    def anchor(self) -> bool:
        """Whether the device is an anchor (initiator or responder)."""
        return self.role in ("anchor-initiator", "anchor-responder")

    @property
    def stored(self) -> Optional[int]:
        """The code of the session it stores; None while a parameter of it is
        not cached."""
        return code(self.params)

    @property
    def tx_gain(self) -> Optional[int]:
        """An anchor's stored TX level; None on a tag or while not cached."""
        value = self.params.get(TX_GAIN_PARAM) if self.anchor else None
        return None if value is None else int(value)

    @property
    def missing(self) -> list[str]:
        """The parameters the cell settings need and the cache lacks; only
        ``UWB_ROLE`` while the role is unknown, since the role decides the
        rest."""
        if self.role is None:
            return ["UWB_ROLE"]
        names = (*CODE_PARAMS, TX_GAIN_PARAM) if self.anchor else CODE_PARAMS
        return [name for name in names if name not in self.params]


def cell_members(ext: "RtlsExtension") -> list[CellMember]:
    """The online devices by system id, disabled ones left out.

    Each carries the role its own cache reports, never one retained from an
    earlier device with that system id; the retained parameters only fill in
    its display name.
    """
    from .extension import _decoded_device_params, _device_name

    devices = ext._protocol.devices if ext._protocol else {}
    members = []
    for device in sorted(devices.values(), key=lambda d: d.system_id):
        params = _decoded_device_params(device)
        role = role_from_params(params)
        if role == "disabled":
            continue
        identity = {**ext._retained_params.get(device.system_id, {}), **params}
        stats = ext._fresh_stats(device.system_id) or {}
        power = ext._power_state(device.system_id)
        if role == "tag":
            operating = not getattr(device, "sleeping", False)
        else:
            operating = None if power is None else power == "operating"
        running, running_tx = stats.get("cellConfig"), stats.get("txGain")
        members.append(
            CellMember(
                system_id=device.system_id,
                name=_device_name(
                    device.system_id, identity, ext._device_role(device.system_id)
                )
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


def cell_reference(members: list[CellMember]) -> Optional[CellMember]:
    """The cell's reference: the online A0 once it reports what it runs."""
    return next(
        (m for m in members if m.role == "anchor-initiator" and m.running is not None),
        None,
    )


def _state(
    member: CellMember, reference: Optional[CellMember]
) -> tuple[str, Optional[str]]:
    """The device's ``(state, detail)`` against the reference, by the rules in
    the module docstring, in their order."""
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
    if reference is None:
        return "unknown", None
    if member.running != reference.running:
        return "differ", None
    if member.anchor and reference.running_tx is None:
        return "unknown", None
    if member.anchor and member.running_tx != reference.running_tx:
        return "differ", _TX_DETAIL
    return "ok", None


def cell_view(
    ext: "RtlsExtension",
) -> tuple[dict[str, Any], dict[int, tuple[str, Optional[str]]]]:
    """The cell's settings and every online device's state, from one
    snapshot.

    Returns:
        ``cellSettings`` as X-RTLS-INF and X-RTLS-CELL carry it (the
        reference's stored rate, block and TX level, its system id, and the
        system ids in each state but ``ok``), and the ``(state, detail)`` of
        every online device by system id.
    """
    members = cell_members(ext)
    reference = cell_reference(members)
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


def _not_ok(ext: "RtlsExtension") -> list[tuple[Optional[int], str]]:
    """What the lock and Verify report.

    Returns:
        ``(system id, "<name>: <what>")`` of every online device that is not
        ``ok``; without a reference, a single ``(None, why)`` for the whole
        cell instead of an ``unknown`` per device.
    """
    members = cell_members(ext)
    reference = cell_reference(members)
    if reference is None:
        return [(None, _NO_REFERENCE)]
    found = []
    for member in members:
        state, detail = _state(member, reference)
        if state != "ok":
            found.append(
                (member.system_id, f"{member.name}: {detail or _STATE_DETAILS[state]}")
            )
    return found


def cell_warnings(ext: "RtlsExtension") -> list[dict[str, Any]]:
    """The lock preflight's ``cell-settings`` warnings: one per device that is
    not ``ok`` (``device`` null for the missing reference)."""
    return [
        {"code": "cell-settings", "device": system_id, "detail": text}
        for system_id, text in _not_ok(ext)
    ]


def cell_rule(ext: "RtlsExtension") -> dict[str, Any]:
    """The X-RTLS-VERIFY ``cell`` rule: an error while any online device is
    not ``ok`` or there is no reference to compare with."""
    problems = [text for _, text in _not_ok(ext)]
    return {
        "id": "cell",
        "label": "Cell settings",
        "severity": "error",
        "status": "fail" if problems else "pass",
        "detail": "; ".join(problems) or "every device runs the cell's settings",
    }
