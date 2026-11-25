"""The group of an anchor survey: which anchors can take part, and the order
in which they lead its steps."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

from rtlslink.cell_settings import RATE_PRESETS
from rtlslink.survey import MAX_GROUP

from flockwave.server.model.uav import UAV

from .anchor_lock import AnchorView, anchor_views, lock_refusal

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("Anchor", "block_ms", "select_group", "step_order")

_BLOCK_PARAM = "UWB_BLOCK_MS"


@dataclass(frozen=True)
class Anchor:
    """An anchor of a survey group."""

    system_id: int
    name: str
    mac: int
    #: it boots as a responder, so its ``rnghz`` says it hears its initiator
    boot_responder: bool


def _params(ext: "RtlsExtension", system_id: int) -> dict[str, Any]:
    from .extension import _decoded_device_params

    device = ext._require_protocol().devices.get(system_id)
    return {} if device is None else _decoded_device_params(device)


def _slot(ext: "RtlsExtension", system_id: int) -> int:
    """An anchor's row in its own stored table; past every row without one."""
    from .extension import _anchor_index_for_device

    index = _anchor_index_for_device(_params(ext, system_id))
    return MAX_GROUP if index is None else index


def _must_take_part(view: AnchorView) -> bool:
    """Whether a survey of its cell needs this anchor in the group: it is
    online and reports no other power state than operating. Asleep, in
    standby or off it does not range; with no report yet it may well."""
    return view["online"] and view["stats"].get("powerState") in (None, "operating")


def select_group(
    ext: "RtlsExtension", requested: Optional[list[int]]
) -> tuple[list[Anchor], Optional[str]]:
    """The group a start names, or why no survey can run with it. By default
    the anchors that must take part, in slot order."""
    views = {view["systemId"]: view for view in anchor_views(ext)}
    ids = requested
    if ids is None:
        ranging = [
            system_id for system_id in views if _must_take_part(views[system_id])
        ]
        ids = sorted(ranging, key=lambda system_id: _slot(ext, system_id))
    if not 3 <= len(ids) <= MAX_GROUP:
        return [], f"A survey takes three to eight anchors, not {len(ids)}"
    unknown = [system_id for system_id in ids if system_id not in views]
    if unknown:
        return [], f"Device {unknown[0]} is not a known anchor"
    group = [views[system_id] for system_id in ids]
    reason = _state_refusal(group) or _left_out(ext, views, set(ids))
    if reason:
        return [], reason
    anchors = []
    for view in group:
        mac = _params(ext, view["systemId"]).get("UWB_MAC")
        if mac is None:
            return [], f"{view['name']}: UWB_MAC not read yet"
        responder = view["role"] == "anchor-responder"
        anchors.append(Anchor(view["systemId"], view["name"], int(mac), responder))
    if len({anchor.mac for anchor in anchors}) != len(anchors):
        return [], "Two anchors of the survey have the same UWB_MAC"
    return anchors, None


def _state_refusal(group: list[AnchorView]) -> Optional[str]:
    """Why an anchor of the group cannot take a survey step now."""
    offline = [view["name"] for view in group if not view["online"]]
    if offline:
        return f"{offline[0]} is offline"
    reason = lock_refusal(group)
    if reason:
        return reason
    # the session comes with the status report, every 10 s: until the first
    # one, an anchor's stats do not say whether it ranges
    unreported = [view["name"] for view in group if "cellConfig" not in view["stats"]]
    if unreported:
        return f"Still waiting for {unreported[0]} to report its session"
    idle = [view["name"] for view in group if view["stats"]["cellConfig"] is None]
    return f"{idle[0]} is not ranging" if idle else None


def armed_refusal(ext: "RtlsExtension") -> Optional[str]:
    """Why no survey may start or be committed now: a drone reports itself
    armed. Each step moves the initiator and a commit restarts every anchor's
    radio, so the positions of the tags jump. An arming state that is not
    known does not refuse: the server cannot tell a drone that is off."""
    registry = getattr(ext.app, "object_registry", None)
    for uav_id in sorted(registry.ids_by_type(UAV)) if registry else ():
        if registry.find_by_id(uav_id).status.armed:
            return f"Drone {uav_id} is armed"
    return None


def _left_out(
    ext: "RtlsExtension", views: dict[int, AnchorView], group: set[int]
) -> Optional[str]:
    """Why the group cannot be surveyed without the rest of its cell. An
    operating anchor of the cell that is left out loses its initiator at
    every step and keeps its old survey ID after the commit, so every tag
    then leaves it out."""
    from .extension import _cell_id_from_params

    cells = {_cell_id_from_params(_params(ext, system_id)) for system_id in group}
    names = [
        view["name"]
        for system_id, view in views.items()
        if system_id not in group
        and _must_take_part(view)
        and _cell_id_from_params(_params(ext, system_id)) in cells
    ]
    if not names:
        return None
    them = "it" if len(names) == 1 else "them"
    return f"Include {', '.join(names)} in the survey, or put {them} to sleep"


def step_order(ext: "RtlsExtension", anchors: list[Anchor]) -> list[Anchor]:
    """The stored slot order when the group is exactly one cell, so that the
    first step is the session the anchors already run: it restarts no radio
    and its acks say every anchor is in the survey before any role moves.
    Any other group keeps the order it was given in."""
    initiators = [anchor for anchor in anchors if not anchor.boot_responder]
    if len(initiators) != 1:
        return anchors
    params = _params(ext, initiators[0].system_id)
    count = int(params.get("UWB_AN_COUNT", 0))
    table = [int(params.get(f"UWB_AN{i}_MAC", 0)) for i in range(count)]
    by_mac = {anchor.mac: anchor for anchor in anchors}
    if sorted(table) != sorted(by_mac):
        return anchors
    return [by_mac[mac] for mac in table]


def block_ms(ext: "RtlsExtension", anchors: list[Anchor]) -> int:
    """The longest block an anchor of the group stores, which sets how long
    a step may take; the slowest preset while none is read."""
    blocks = [_params(ext, anchor.system_id).get(_BLOCK_PARAM) for anchor in anchors]
    known = [int(block) for block in blocks if block is not None]
    return max(known) if known else max(RATE_PRESETS.values())
