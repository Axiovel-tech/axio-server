"""What a survey commit writes: the survey ID, each anchor's ``UWB_LOC``
and the write itself.

The anchors are the only store. The survey ID continues the sequence the
known anchors hold, so a new server at a venue goes on where the last one
stopped, and every anchor of one commit gets the same ID. Nothing is rolled
back when an anchor misses the write: it keeps advertising the old ID or
none, the tags exclude it, and the same commit again brings it in.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from contextlib import suppress
from functools import reduce
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink import anchor_location
from rtlslink.anchor_location import AnchorLocation, StoredLocation

from .anchor_lock import anchor_views
from .cell_settings import write_refusal
from .survey_fit import DirectedRange, SurveyFit
from .survey_group import Anchor

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "CONFIRM_TIMEOUT",
    "commit_refusal",
    "known_survey_ids",
    "location_values",
    "next_survey_id",
    "read_params",
    "running_ids",
    "stored_positions",
    "write_locations",
)

#: how long a commit waits for the anchors to report the new survey ID, in
#: seconds. An anchor restarts its ranging once (about 0.3 s) and reports the
#: ID at once when it changes.
CONFIRM_TIMEOUT = 10.0

LOC_PARAM = anchor_location.PARAM

_READ_TIMEOUT = 2.0

#: row 0 of an anchor's stored table: its stored initiator
_STORED_INITIATOR = "UWB_AN0_MAC"

#: what ``UWB_LOC`` carries of a coordinate: 24 bits of millimetres
_MAX_COORDINATE_M = 8388.0

Point = tuple[float, float, float]


def next_survey_id(stored: Iterable[int]) -> int:
    """One more than the newest of the stored IDs in 16-bit serial order,
    skipping 0 and 0xFFFF, which no anchor advertises; 1 without any."""
    ids = list(stored)
    if not ids:
        return 1
    newest = reduce(
        lambda newest, other: other if (other - newest) & 0xFFFF < 0x8000 else newest,
        ids,
    )
    return 1 if newest >= 0xFFFE else newest + 1


def _stored_location(device: Any) -> Optional[StoredLocation]:
    """The location in a device's cached ``UWB_LOC``, None without one."""
    from .extension import _decoded_device_params

    text = _decoded_device_params(device, (LOC_PARAM,)).get(LOC_PARAM, "")
    with suppress(ValueError):
        return anchor_location.decode_param(str(text))
    return None


def stored_positions(ext: "RtlsExtension", ids: Sequence[int]) -> dict[int, Point]:
    """The position each of the live anchors ``ids`` stores, where it does."""
    devices = ext._require_protocol().devices
    positions = {}
    for system_id in ids:
        device = devices.get(system_id)
        stored = None if device is None else _stored_location(device)
        if stored is not None:
            positions[system_id] = stored.location.ned_m
    return positions


def known_survey_ids(ext: "RtlsExtension") -> list[int]:
    """The survey ID of every known anchor that stores one; an offline
    anchor's as it was last read."""
    devices = ext._require_protocol().devices
    ids = []
    for view in anchor_views(ext):
        system_id = view["systemId"]
        device = devices.get(system_id) or ext._lost.get(system_id)
        stored = None if device is None else _stored_location(device)
        if stored is not None:
            ids.append(stored.location.survey_id)
    return ids


async def read_params(ext: "RtlsExtension", ids: Sequence[int], *, fresh: bool) -> None:
    """Reads what a fit and a commit need of the live anchors ``ids`` into
    their parameter caches: the stored location, again when ``fresh`` asks,
    and the stored initiator where it is not cached. An anchor that does not
    answer keeps what was cached."""
    devices = ext._require_protocol().devices

    async def read(system_id: int, name: str) -> None:
        with suppress(KeyError, trio.TooSlowError):
            await ext.get_param(system_id, name, timeout=_READ_TIMEOUT)

    async with trio.open_nursery() as nursery:
        for system_id in ids:
            device = devices.get(system_id)
            if device is None:
                continue
            for name in (LOC_PARAM, _STORED_INITIATOR):
                if name not in device.params or (fresh and name == LOC_PARAM):
                    nursery.start_soon(read, system_id, name)


def commit_refusal(
    ext: "RtlsExtension", anchors: Sequence[Anchor], fit: SurveyFit, stale: bool
) -> Optional[str]:
    """Why a held fit cannot be written now, if it cannot."""
    devices = ext._require_protocol().devices
    if stale or any(anchor.system_id not in devices for anchor in anchors):
        return "Survey again"
    if ext._cell_applying:
        return "An apply is running"
    if any(
        abs(coordinate) >= _MAX_COORDINATE_M
        for report in fit.anchors.values()
        for coordinate in report.position
    ):
        return "A fitted position is farther than an anchor can store"
    return write_refusal(ext)


def location_values(
    ext: "RtlsExtension",
    anchors: Sequence[Anchor],
    fit: SurveyFit,
    measurements: Mapping[tuple[int, int], DirectedRange],
    survey_id: int,
) -> dict[int, str]:
    """The ``UWB_LOC`` value of each anchor: its fitted position, the
    survey ID, the planar flag and its reference, the range it measured in
    the step its stored initiator led (0 for the initiator itself, and when
    that range is missing or the stored initiator is not in the group)."""
    from .extension import _decoded_device_params

    devices = ext._require_protocol().devices
    by_mac = {anchor.mac: anchor.system_id for anchor in anchors}
    values = {}
    for anchor in anchors:
        device = devices.get(anchor.system_id)
        params = _decoded_device_params(device, (_STORED_INITIATOR,)) if device else {}
        initiator = by_mac.get(params.get(_STORED_INITIATOR, 0), 0)
        measured = measurements.get((initiator, anchor.system_id))
        reference_mm = 0
        if measured is not None and measured.usable:
            reference_mm = round(measured.range_m * 1000)
        north, east, down = fit.anchors[anchor.system_id].position
        location = AnchorLocation.from_metres(
            north, east, down, survey_id, planar=fit.planar
        )
        values[anchor.system_id] = anchor_location.encode_param(
            StoredLocation(location, reference_mm)
        )
    return values


async def write_locations(
    ext: "RtlsExtension", anchors: Sequence[Anchor], values: Mapping[int, str]
) -> dict[str, dict[str, Any]]:
    """Writes every anchor its value at once; the ``results`` of the commit.
    A write counts when its ack is accepted and echoes the value."""
    results: dict[str, dict[str, Any]] = {}

    async def write(anchor: Anchor) -> None:
        value = values[anchor.system_id]
        try:
            ack = await ext.set_param(
                anchor.system_id, LOC_PARAM, value, "custom", exact=True
            )
        except trio.TooSlowError:
            detail = f"{anchor.name}: no answer"
        except KeyError:
            detail = f"{anchor.name} is offline"
        else:
            if ack["accepted"]:
                results[str(anchor.system_id)] = {"written": "changed"}
                return
            detail = f"{anchor.name} refused its position"
        results[str(anchor.system_id)] = {"written": "failed", "detail": detail}

    async with trio.open_nursery() as nursery:
        for anchor in anchors:
            nursery.start_soon(write, anchor)
    return results


def running_ids(
    ext: "RtlsExtension", anchors: Sequence[Anchor], survey_id: int
) -> list[int]:
    """The anchors whose ``sid`` stat reports ``survey_id``."""
    return [
        anchor.system_id
        for anchor in anchors
        if (ext._fresh_stats(anchor.system_id) or {}).get("surveyId") == survey_id
    ]
