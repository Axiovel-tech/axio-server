"""Fleet cell-geometry agreement (X-RTLS-GEOM).

Since rtls-link-zephyr#210 every tag fits the anchor table itself at boot
from the initiator distances it receives (stacked-rectangle deployment
convention) and streams the fit as health stats: the geometry state, why the last fit was
rejected, the rectangle-diagonal residual, the largest live drift since
calibration and the seven fitted AN0-ANi distances. A rig outside the
convention (two anchors swapped, one missing) is rejected on the tag, never
fitted; the reason is reported here. The fitted table is a deterministic
function of those distances, so "did every drone converge on the same
cell?" reduces to comparing the distances across the fleet against a
per-distance median reference.

This module is a pure function over the extension's cached stats: no
device I/O, no parameter reads, so it scales to a swarm and can run as
often as a UI likes.
"""

from __future__ import annotations

import math
import time
from statistics import median_low
from typing import TYPE_CHECKING, Any, Optional

from .cell_compat import role_from_params
from .extension import _cell_id_from_params, _decoded_device_params

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "DEFAULT_TOLERANCE_M",
    "GEOMETRY_REASON_NAMES",
    "GEOMETRY_STATE_NAMES",
    "STATS_FRESH_S",
    "run_agreement",
)

#: a fitted distance farther than this from the fleet's median reference
#: marks the drone as deviating, metres; a live initiator distance that
#: drifted farther than this from its calibrated value marks the drone as
#: drifted (a tripod moved after the fit). Bench: independent fits of the
#: same cell agree within ~4 mm; a moved tripod shows as centimetres.
DEFAULT_TOLERANCE_M = 0.02

#: a stats snapshot older than this many seconds is STALE: the stream
#: went silent and the tag's geometry cannot be certified on it
STATS_FRESH_S = 10.0

#: firmware ``geom`` stat codes
GEOMETRY_STATE_NAMES = {
    0: "manual",
    1: "waiting",
    2: "calibrating",
    3: "calibrated",
    4: "failed",
}

#: firmware ``gfail`` stat codes: why the last fit was rejected
GEOMETRY_REASON_NAMES = {
    0: "",
    1: "the provisioned anchor count is not 4 or 8",
    2: "an anchor was never heard",
    3: "a side or the height is under 1 m",
    4: "not a rectangle: the diagonal disagrees with the sides (anchors swapped?)",
    5: "the top plane is not above the bottom one (anchors swapped?)",
}

STATE_MANUAL = 0
STATE_CALIBRATED = 3
STATE_FAILED = 4


def _is_tag(ext: "RtlsExtension", system_id: int, params: dict[str, Any]) -> bool:
    """Whether a device is (or may still turn out to be) a tag. The state
    advertisement names the role before the parameter cache does, so it
    is consulted first, as X-RTLS-INF does; a device of unknown role is
    graded (and reported as ``unknown``) rather than silently skipped."""
    role = ext._adv.get(system_id, {}).get("role") or role_from_params(params)
    return role is None or role == "tag"


def _fit_state(
    system_id: int, cell: str, stats: Optional[dict[str, Any]], stats_age: float
) -> dict[str, Any]:
    """What the tag reports about its own fit, with a status for every tag
    that cannot take part in the comparison; ``candidate`` for a fresh,
    calibrated tag."""
    entry: dict[str, Any] = {"id": system_id, "cell": cell}
    if not stats or "geometryState" not in stats:
        entry["status"] = "unknown"
        entry["detail"] = "no geometry telemetry (firmware without automatic geometry?)"
        return entry
    state = int(stats["geometryState"])
    entry["state"] = state
    entry["stateName"] = GEOMETRY_STATE_NAMES.get(state, "?")
    for key, field in (
        ("geometryResidualM", "residualM"),
        ("geometryDriftM", "driftM"),
        ("geometryReason", "reason"),
    ):
        if key in stats:
            entry[field] = stats[key]
    if stats_age > STATS_FRESH_S:
        entry["status"] = "stale"
        entry["detail"] = f"telemetry went silent ({stats_age:.0f} s ago)"
    elif state == STATE_MANUAL:
        entry["status"] = "manual"
        entry["detail"] = "uses its provisioned UWB_AN* table"
    elif state == STATE_CALIBRATED:
        entry["status"] = "candidate"
    elif state == STATE_FAILED:
        reason = GEOMETRY_REASON_NAMES.get(int(stats.get("geometryReason", 0)), "?")
        entry["status"] = "failed"
        entry["detail"] = (
            f"the tag rejected the rig: {reason or 'no reason reported'}; it retries"
        )
    else:
        entry["status"] = "calibrating"
        entry["detail"] = "still fitting the cell"
    return entry


def _fitted_distances(entry: dict[str, Any], stats: dict[str, Any], tolerance: float) -> None:
    """Attaches a candidate's fit or downgrades the entry. The stats arrive
    one field at a time, so only a complete, finite fit — the three
    bottom-plane distances plus all or none of the top plane — with its
    live drift is graded; a drift over ``tolerance`` means a tripod moved
    since the fit, and the table is stale even if every tag agrees on it."""
    distances = list(stats.get("geometryDistancesM") or [])
    distances += [None] * (7 - len(distances))
    bottom_complete = all(d is not None for d in distances[:3])
    top = [d is not None for d in distances[3:7]]
    if not bottom_complete or (any(top) and not all(top)):
        entry["status"] = "calibrating"
        entry["detail"] = "calibrated but the fitted distances have not all arrived yet"
        return
    entry["distancesM"] = distances
    if "geometryDriftM" not in stats:
        entry["status"] = "calibrating"
        entry["detail"] = "calibrated but the live drift has not arrived yet"
        return
    drift = float(stats["geometryDriftM"])
    if not math.isfinite(drift) or not all(
        d is None or math.isfinite(d) for d in distances
    ):
        entry["status"] = "calibrating"
        entry["detail"] = "fitted distances are not finite yet"
        return
    if drift > tolerance:
        entry["status"] = "drifted"
        entry["detail"] = (
            f"an initiator distance moved {drift * 100:.1f} cm since the fit "
            "(a tripod moved?) — recalibrate"
        )


def _entry_for(
    system_id: int,
    cell: str,
    stats: Optional[dict[str, Any]],
    stats_age: float,
    tolerance: float,
) -> dict[str, Any]:
    """The per-device entry before the fleet comparison."""
    entry = _fit_state(system_id, cell, stats, stats_age)
    if entry["status"] == "candidate":
        _fitted_distances(entry, stats or {}, tolerance)
    return entry


def _reference(candidates: list[dict[str, Any]]) -> Optional[list[Optional[float]]]:
    """Per-distance lower median across the calibrated tags, so the
    reference is always a value some tag actually reported (two tags
    never both deviate from their own midpoint); ``None`` where no tag
    reports that distance (a four-anchor cell has no top plane)."""
    if not candidates:
        return None
    reference: list[Optional[float]] = []
    for index in range(7):
        values = [
            entry["distancesM"][index]
            for entry in candidates
            if index < len(entry["distancesM"])
            and entry["distancesM"][index] is not None
        ]
        reference.append(round(median_low(values), 4) if values else None)
    return reference


def _compare(
    entry: dict[str, Any], reference: list[Optional[float]], tolerance: float
) -> None:
    """Grades one candidate against the reference in place."""
    distances = entry["distancesM"]
    deviations: dict[str, float] = {}
    missing: list[str] = []
    max_deviation = 0.0
    for index, ref in enumerate(reference):
        value = distances[index] if index < len(distances) else None
        name = f"AN0-AN{index + 1}"
        if ref is None:
            if value is not None:
                missing.append(name)  # this tag fitted a plane the others lack
            continue
        if value is None:
            missing.append(name)
            continue
        deviation = abs(value - ref)
        max_deviation = max(max_deviation, deviation)
        if deviation > tolerance:
            deviations[name] = round(deviation, 4)
    entry["maxDeviationM"] = round(max_deviation, 4)
    if deviations or missing:
        entry["status"] = "deviates"
        parts = []
        if deviations:
            parts.append(
                "off the fleet reference by "
                + ", ".join(
                    f"{name} {dev * 100:.1f} cm" for name, dev in deviations.items()
                )
            )
        if missing:
            parts.append("anchor set differs (" + ", ".join(missing) + ")")
        entry["deviations"] = deviations
        if missing:
            entry["missing"] = missing
        entry["detail"] = "; ".join(parts)
    else:
        entry["status"] = "agree"


#: parameters that place the fitted cell in the world and bind the fitted
#: slots to physical anchors: the origin the NED frame hangs off, the
#: show-frame yaw and the slot->anchor MAC table. Identical distances are
#: not the same cell if these differ (a symmetric rig with two slots
#: swapped fits the same distances onto different anchors), so the graded
#: tags of one cell must agree on them too — as the removed X-RTLS-GEO
#: check compared them.
FRAME_PARAMS = (
    "ORIGIN_LAT_E7",
    "ORIGIN_LON_E7",
    "ORIGIN_ALT_MM",
    "POS_YAW_DEG",
    # how many rows of the anchor table the solver consumes
    "UWB_AN_COUNT",
)


def _frame_names(entry: dict[str, Any]) -> tuple[str, ...]:
    """The frame parameters that matter for this tag's fit: the world
    placement, plus the MAC of every slot the fit includes."""
    distances = entry.get("distancesM") or []
    slots = 8 if len(distances) >= 7 and distances[3] is not None else 4
    # the MAC binds a slot to an anchor; the bias is the range correction
    # the solver applies to that anchor — both change where a tag solves
    return FRAME_PARAMS + tuple(
        name for i in range(slots) for name in (f"UWB_AN{i}_MAC", f"UWB_AN{i}_BIAS_M")
    )


def _frame_value(name: str, value: Any) -> Any:
    """A frame parameter as compared across the cell: a non-finite real
    is not a value (``None``, so the tag stays ``incomplete`` rather than
    certified on an invalid yaw or bias); the show yaw is one revolution
    modulo, 0 and 360 being the same rotation."""
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        if name == "POS_YAW_DEG":
            return round(value % 360.0, 6) % 360.0
    return value


def _frame_of(params: dict[str, Any], names: tuple[str, ...]) -> tuple[Any, ...]:
    return tuple(_frame_value(name, params.get(name)) for name in names)


def _cell_known(device: Any, params: dict[str, Any]) -> bool:
    """Whether the cell a tag belongs to can be trusted: ``CELL_ID`` is
    cached, or the parameter listing is complete and lacks it (older
    firmware; the default cell)."""
    if params.get("CELL_ID") or params.get("RTLS_CELL_ID"):
        return True
    return device.param_count is not None and len(device.params) >= device.param_count


def _check_frames(protocol: Any, graded: list[dict[str, Any]]) -> None:
    """Downgrades graded tags whose coordinate-frame parameters differ from
    the majority of their cell to ``frame`` (blocking), and tags whose
    frame is not fully known yet (identity refill still running, or a
    lossy dump) to ``incomplete`` (blocking): a fit cannot be certified
    without knowing where it stands."""
    if not graded:
        return
    names = _frame_names(graded[0])
    frames = {
        entry["id"]: _frame_of(
            _decoded_device_params(protocol.devices[entry["id"]]), names
        )
        for entry in graded
    }
    complete = [f for f in frames.values() if all(v is not None for v in f)]
    majority = max(set(complete), key=complete.count) if complete else None
    for entry in graded:
        frame = frames[entry["id"]]
        missing = [name for name, value in zip(names, frame) if value is None]
        if missing:
            entry["status"] = "incomplete"
            entry["missingParams"] = missing
            entry["detail"] = "coordinate frame not fully known yet: " + ", ".join(
                missing
            )
            continue
        if majority is None:
            continue
        differing = [
            name for name, value, ref in zip(names, frame, majority) if value != ref
        ]
        if differing:
            entry["status"] = "frame"
            entry["frame"] = differing
            entry["detail"] = (
                "coordinate frame differs from the rest of the cell: "
                + ", ".join(differing)
            )


BLOCKING = frozenset(
    {
        "deviates",
        "drifted",
        "frame",
        "incomplete",
        "missing",
        "calibrating",
        "failed",
        "stale",
        "unknown",
    }
)


def _tag_entries(
    ext: "RtlsExtension",
    protocol: Any,
    ids: Optional[list[int]],
    tolerance: float,
    now: float,
) -> dict[str, dict[str, Any]]:
    """One entry per graded device: a requested id that is not online is a
    verdict too (``missing``: the caller asked, so silence would certify
    it), anchors carry no fit and are skipped, and a tag whose cell is not
    known yet is ``incomplete`` (a partial parameter cache without CELL_ID
    would file it under the default cell and certify it against the wrong
    reference)."""
    entries: dict[str, dict[str, Any]] = {}
    if ids is not None:
        for system_id in sorted(set(ids) - set(protocol.devices)):
            entries[str(system_id)] = {
                "id": system_id,
                "status": "missing",
                "detail": "device is not online",
            }
    for system_id, device in sorted(protocol.devices.items()):
        if ids is not None and system_id not in ids:
            continue
        params = _decoded_device_params(device)
        if not _is_tag(ext, system_id, params):
            continue
        stats_age = now - ext._stats_at.get(system_id, float("-inf"))
        entry = _entry_for(
            system_id,
            _cell_id_from_params(params),
            ext._stats.get(system_id),
            stats_age,
            tolerance,
        )
        if entry["status"] == "candidate" and not _cell_known(device, params):
            entry["status"] = "incomplete"
            entry["missingParams"] = ["CELL_ID"]
            entry["detail"] = "cell not known yet: CELL_ID"
        entries[str(system_id)] = entry
    return entries


def _grade_cells(
    protocol: Any, entries: dict[str, dict[str, Any]], tolerance: float
) -> dict[str, list[Optional[float]]]:
    """Grades the candidates of every cell against that cell's reference
    (in place) and returns the references."""
    references: dict[str, list[Optional[float]]] = {}
    for cell in sorted({entry["cell"] for entry in entries.values() if "cell" in entry}):
        candidates = [
            e for e in entries.values() if e["status"] == "candidate" and e["cell"] == cell
        ]
        reference = _reference(candidates)
        if reference is None:
            continue
        references[cell] = reference
        for entry in candidates:
            _compare(entry, reference, tolerance)
        _check_frames(protocol, candidates)
    return references


def run_agreement(
    ext: "RtlsExtension",
    *,
    ids: Optional[list[int]] = None,
    tolerance: float = DEFAULT_TOLERANCE_M,
    now: Optional[float] = None,
) -> dict[str, Any]:
    """Compares the fleet's fitted geometries; returns the X-RTLS-GEOM
    response body.

    Tags (role 1, or unknown role) among the live devices are graded per
    cell (``CELL_ID``): ``agree`` / ``deviates`` against the per-distance
    lower median of that cell's calibrated tags, ``drifted`` when the live
    distances moved away from the fit, ``frame`` when the origin, show-yaw
    or anchor-MAC parameters differ from the cell's majority,
    ``incomplete`` while those parameters are not all known, ``missing``
    for a requested id that is not online, or ``manual`` / ``calibrating`` /
    ``failed`` (with the tag's ``reason``) / ``stale`` / ``unknown`` when
    they cannot take part. ``references`` holds one reference per cell;
    ``reference`` is that of the only cell (``None`` with none or several).
    ``consistent`` is true when at least one tag agrees and no tag
    deviates, drifted, is still calibrating, failed, stale or unknown —
    manual tags are reported but do not block, their table is the
    operator's deliberate choice."""
    protocol = ext._require_protocol()
    if now is None:
        now = time.monotonic()
    entries = _tag_entries(ext, protocol, ids, tolerance, now)
    references = _grade_cells(protocol, entries, tolerance)
    consistent = bool(references) and not any(
        entry["status"] in BLOCKING for entry in entries.values()
    )
    return {
        "type": "X-RTLS-GEOM",
        "tolerance": tolerance,
        "reference": next(iter(references.values())) if len(references) == 1 else None,
        "references": references,
        "consistent": consistent,
        "devices": entries,
    }
