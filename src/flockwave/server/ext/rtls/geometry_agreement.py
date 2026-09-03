"""Fleet cell-geometry agreement (X-RTLS-GEOM).

Since rtls-link-zephyr#208 every tag fits the anchor table itself at boot
from the initiator distances it receives (stacked-rectangle deployment
convention) and streams the fit as health stats: the geometry state, the
rectangle-diagonal residual, the largest live drift since calibration and
the seven fitted AN0-ANi distances. The fitted table is a deterministic
function of those distances, so "did every drone converge on the same
cell?" reduces to comparing the distances across the fleet against a
per-distance median reference.

This module is a pure function over the extension's cached stats: no
device I/O, no parameter reads, so it scales to a swarm and can run as
often as a UI likes.
"""

from __future__ import annotations

import time
from statistics import median_low
from typing import TYPE_CHECKING, Any, Optional

from .cell_compat import role_from_params
from .extension import _cell_id_from_params, _decoded_device_params

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "DEFAULT_TOLERANCE_M",
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

#: firmware UWB_GEOM_STATE / ``geom`` stat codes
GEOMETRY_STATE_NAMES = {
    0: "manual",
    1: "waiting",
    2: "calibrating",
    3: "calibrated",
    4: "failed",
}

STATE_MANUAL = 0
STATE_CALIBRATED = 3


def _is_tag(ext: "RtlsExtension", system_id: int, params: dict[str, Any]) -> bool:
    """Whether a device is (or may still turn out to be) a tag. The state
    advertisement names the role before the parameter cache does, so it
    is consulted first, as X-RTLS-INF does; a device of unknown role is
    graded (and reported as ``unknown``) rather than silently skipped."""
    role = ext._adv.get(system_id, {}).get("role") or role_from_params(params)
    return role is None or role == "tag"


def _entry_for(
    system_id: int,
    cell: str,
    stats: Optional[dict[str, Any]],
    stats_age: float,
    tolerance: float,
) -> dict[str, Any]:
    """The per-device entry before the fleet comparison: everything the
    tag reports about its own fit, plus a status for the tags that
    cannot take part in the comparison."""
    entry: dict[str, Any] = {"id": system_id, "cell": cell}
    if not stats or "geometryState" not in stats:
        entry["status"] = "unknown"
        entry["detail"] = "no geometry telemetry (firmware without automatic geometry?)"
        return entry
    state = int(stats["geometryState"])
    entry["state"] = state
    entry["stateName"] = GEOMETRY_STATE_NAMES.get(state, "?")
    if "geometryResidualM" in stats:
        entry["residualM"] = stats["geometryResidualM"]
    if "geometryDriftM" in stats:
        entry["driftM"] = stats["geometryDriftM"]
    if stats_age > STATS_FRESH_S:
        entry["status"] = "stale"
        entry["detail"] = f"telemetry went silent ({stats_age:.0f} s ago)"
        return entry
    if state == STATE_MANUAL:
        entry["status"] = "manual"
        entry["detail"] = "uses its provisioned UWB_AN* table"
        return entry
    if state != STATE_CALIBRATED:
        entry["status"] = "calibrating" if state in (1, 2) else "failed"
        entry["detail"] = (
            "still fitting the cell"
            if state in (1, 2)
            else "could not fit the cell (AN1..AN3 not all heard); retrying"
        )
        return entry
    distances = stats.get("geometryDistancesM")
    if not distances or not any(d is not None for d in distances):
        entry["status"] = "calibrating"
        entry["detail"] = "calibrated but the fitted distances have not arrived yet"
        return entry
    entry["distancesM"] = list(distances)
    drift = float(stats.get("geometryDriftM", 0.0))
    if drift > tolerance:
        # the fit still describes the cell as it was at boot: the live
        # distances say an anchor has moved since, so the table is stale
        # even if every tag agrees on it
        entry["status"] = "drifted"
        entry["detail"] = (
            f"an initiator distance moved {drift * 100:.1f} cm since the fit "
            "(a tripod moved?) — recalibrate"
        )
        return entry
    entry["status"] = "candidate"
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
    distances moved away from the fit, or ``manual`` / ``calibrating`` /
    ``failed`` / ``stale`` / ``unknown`` when they cannot take part.
    ``references`` holds one reference per cell; ``reference`` is that of
    the only cell (``None`` with none or several). ``consistent`` is true
    when at least one tag agrees and no tag deviates, drifted, is still
    calibrating, failed, stale or unknown — manual tags are reported but
    do not block, their table is the operator's deliberate choice."""
    protocol = ext._require_protocol()
    if now is None:
        now = time.monotonic()
    entries: dict[str, dict[str, Any]] = {}
    for system_id, device in sorted(protocol.devices.items()):
        if ids is not None and system_id not in ids:
            continue
        params = _decoded_device_params(device)
        if not _is_tag(ext, system_id, params):
            continue  # anchors carry no fit
        stats = ext._stats.get(system_id)
        stats_age = now - ext._stats_at.get(system_id, float("-inf"))
        entries[str(system_id)] = _entry_for(
            system_id, _cell_id_from_params(params), stats, stats_age, tolerance
        )

    references: dict[str, list[Optional[float]]] = {}
    for cell in sorted({entry["cell"] for entry in entries.values()}):
        candidates = [
            e
            for e in entries.values()
            if e["status"] == "candidate" and e["cell"] == cell
        ]
        reference = _reference(candidates)
        if reference is None:
            continue
        references[cell] = reference
        for entry in candidates:
            _compare(entry, reference, tolerance)

    blocking = {"deviates", "drifted", "calibrating", "failed", "stale", "unknown"}
    consistent = bool(references) and not any(
        entry["status"] in blocking for entry in entries.values()
    )
    return {
        "type": "X-RTLS-GEOM",
        "tolerance": tolerance,
        "reference": next(iter(references.values())) if len(references) == 1 else None,
        "references": references,
        "consistent": consistent,
        "devices": entries,
    }
