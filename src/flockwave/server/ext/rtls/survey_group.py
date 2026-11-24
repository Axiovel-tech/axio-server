"""The group of an anchor survey: which anchors can take part, and the order
in which they lead its steps."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

from rtlslink.cell_settings import RATE_PRESETS
from rtlslink.survey import MAX_GROUP

from .anchor_lock import anchor_views, lock_refusal

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


def select_group(
    ext: "RtlsExtension", requested: Optional[list[int]]
) -> tuple[list[Anchor], Optional[str]]:
    """The group a start names (by default every online anchor, in slot
    order), or why no survey can run with it."""
    views = {view["systemId"]: view for view in anchor_views(ext)}
    ids = requested
    if ids is None:
        online = [system_id for system_id, view in views.items() if view["online"]]
        ids = sorted(online, key=lambda system_id: _slot(ext, system_id))
    if not 3 <= len(ids) <= MAX_GROUP:
        return [], f"A survey takes three to eight anchors, not {len(ids)}"
    unknown = [system_id for system_id in ids if system_id not in views]
    if unknown:
        return [], f"Device {unknown[0]} is not a known anchor"
    group = [views[system_id] for system_id in ids]
    offline = [view["name"] for view in group if not view["online"]]
    if offline:
        return [], f"{offline[0]} is offline"
    reason = lock_refusal(group)
    if reason:
        return [], reason
    idle = [view["name"] for view in group if view["stats"].get("cellConfig") is None]
    if idle:
        return [], f"{idle[0]} is not ranging"
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
