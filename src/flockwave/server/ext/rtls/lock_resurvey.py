"""The lock's resurvey check (X-RTLS-LOCK ``resurvey``).

The live range check sees only the distance to the initiator, so an anchor
carried sideways hardly shows. On request the lock first measures the cell
as a survey does, fits it, lays the positions the anchors store onto the
fitted ones (turned about z and moved, never mirrored, on the anchors that
did not move) and warns of every anchor off its stored position, at the
range check's two bounds. Nothing is held or written; a check that cannot
run, and what it could not check, are warnings too.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

from .survey_fit import Frame, GateEntry, SurveyFit, off_layout
from .survey_view import RANGE_DEVIATION_WARNING_M, range_level

if TYPE_CHECKING:
    from .anchor_lock import AnchorView, LockWarning
    from .survey import Check

__all__ = ("stored_frame", "with_resurvey")

#: ``UWB_LOC`` stores millimetres: the origin and the +x anchor of the
#: frame a commit wrote are within half of one of the axes
_ON_AXIS_M = 0.0015

#: anchors whose stored heights are closer than this, one after the other,
#: are one height group, metres
_HEIGHT_GAP_M = 0.25

#: per anchor, the range check's and the resurvey check's warnings by level
_LEVELS = {
    "range-deviation": 1,
    "resurvey-moved": 1,
    "range-deviation-danger": 2,
    "resurvey-moved-danger": 2,
}
_RANGE = ("range-deviation", "range-deviation-danger")


def stored_frame(
    stored: Mapping[int, Sequence[float]],
) -> tuple[Optional[Frame], Optional[str]]:
    """The frame the stored positions were fitted in, or why there is none:
    anchors at about one height are a height group; the origin is the anchor
    at x = y = 0 of the lowest group that has one (an anchor above it may be
    at x = y = 0 too), the groups above it are upper, and the nearest anchor
    on +x, of the origin's group where it has one, is the +x anchor."""
    groups: list[list[int]] = []
    for anchor in sorted(stored, key=lambda a: stored[a][2]):
        if groups and stored[anchor][2] - stored[groups[-1][-1]][2] < _HEIGHT_GAP_M:
            groups[-1].append(anchor)
        else:
            groups.append([anchor])
    # down is +z: the last group is the lowest
    level = next(
        (
            group
            for group in reversed(groups)
            if any(_on_axis(stored[a][0]) and _on_axis(stored[a][1]) for a in group)
        ),
        None,
    )
    if level is None:
        return None, "no anchor stores the origin of a survey"
    origins = [a for a in level if _on_axis(stored[a][0]) and _on_axis(stored[a][1])]
    if len(origins) > 1:
        return None, "two anchors store the origin of a survey"
    on_x = [a for a, p in stored.items() if _on_axis(p[1]) and p[0] > _ON_AXIS_M]
    if not on_x:
        return None, "no anchor stores the +x of a survey"
    upper = [a for group in groups[: groups.index(level)] for a in group]
    return (
        Frame(
            origin=origins[0],
            x_axis=min(on_x, key=lambda a: (a not in level, stored[a][0])),
            height_groups=tuple(tuple(sorted(group)) for group in groups),
            upper=tuple(sorted(upper)),
        ),
        None,
    )


def _on_axis(value: float) -> bool:
    return abs(value) <= _ON_AXIS_M


def _warning(code: str, device: Optional[int], detail: str) -> "LockWarning":
    return {"code": code, "device": device, "detail": detail}


def _failed(device: Optional[int], detail: str) -> "LockWarning":
    return _warning("resurvey-failed", device, detail)


def _unchecked(device: Optional[int], detail: str) -> "LockWarning":
    return _warning("resurvey-unchecked", device, detail)


def _measurement(entry: GateEntry, names: Mapping[int, str]) -> "LockWarning":
    """A warning of the fit itself: the measurement may be what is off."""
    who = [names[d] for d in (entry.device, entry.peer) if d is not None]
    where = f"{' and '.join(who)}: " if who else ""
    return _failed(entry.device, f"Resurvey measurement poor: {where}{entry.detail}")


def _fit_warnings(
    fit: SurveyFit, stored: Mapping[int, Sequence[float]], names: Mapping[int, str]
) -> list["LockWarning"]:
    """The fit's own warnings, an anchor it cannot locate or that stores no
    position, and each anchor off its stored one. A pair the fit lacks
    blocks a commit, not the check."""
    warnings = [_measurement(entry, names) for entry in fit.warnings]
    unlocated = {e.device: e.detail for e in fit.blocked if e.code == "not-located"}
    placed = []
    for system_id in fit.anchors:
        name = names[system_id]
        if system_id in unlocated:
            warnings.append(
                _failed(system_id, f"{name} not located: {unlocated[system_id]}")
            )
        elif system_id not in stored:
            warnings.append(_unchecked(system_id, f"{name} not checked: no survey"))
        else:
            placed.append(system_id)
    # a flat cell's fit lies in one plane: it cannot see a height change
    heights = not fit.planar
    if not heights:
        warnings.append(_unchecked(None, "Heights not checked: flat cell"))
    if len(placed) < 3:
        warnings.append(_failed(None, "Resurvey check failed: too few anchors located"))
        return warnings
    apart = off_layout(
        np.array([stored[a][:3] for a in placed], dtype=float),
        np.array([fit.anchors[a].position for a in placed], dtype=float),
        RANGE_DEVIATION_WARNING_M,
        heights=heights,
    )
    if apart is None:
        warnings.append(_failed(None, "Resurvey check failed: too many anchors moved"))
        return warnings
    for system_id, metres in zip(placed, apart):
        cm = float(metres) * 100
        level = range_level(float(metres))
        name = names[system_id]
        if level == "danger":
            detail = f"{name} moved: {cm:.0f} cm off its surveyed position"
            warnings.append(_warning("resurvey-moved-danger", system_id, detail))
        elif level == "warning":
            detail = f"{name} is {cm:.0f} cm off its surveyed position"
            warnings.append(_warning("resurvey-moved", system_id, detail))
    return warnings


def _check_warnings(
    check: "Check", views: Sequence["AnchorView"]
) -> list["LockWarning"]:
    """What the check found, and the anchors the lock takes that it did not
    measure as they are now: online since, or restarted while it ran."""
    if check.fit is None:
        return [_failed(None, f"Resurvey check failed: {check.reason}")]
    names = {view["systemId"]: view["name"] for view in views}
    for system_id in check.ids:
        names.setdefault(system_id, f"RTLS anchor {system_id}")
    warnings = _fit_warnings(check.fit, check.stored, names)
    for view in views:
        system_id, name = view["systemId"], view["name"]
        if not view["online"]:
            continue
        if system_id not in check.ids:
            warnings.append(_unchecked(system_id, f"{name} not checked"))
        elif system_id in check.rebooted:
            detail = f"{name} not checked: restarted during the check"
            warnings.append(_unchecked(system_id, detail))
    return warnings


def with_resurvey(
    warnings: list["LockWarning | dict[str, Any]"],
    live: list["LockWarning"],
    check: "Check",
    views: Sequence["AnchorView"],
) -> list["LockWarning | dict[str, Any]"]:
    """The lock's ``warnings`` after a check, with what it found. The range
    check warnings of ``live``, from before the check, stand for an anchor
    without a fresh reading yet; of the range check's and the check's
    warnings, an anchor keeps the graver, the check's at one level."""
    ranged = {w["device"] for w in warnings if w["code"] in _RANGE}
    warnings = warnings + [
        w for w in live if w["code"] in _RANGE and w["device"] not in ranged
    ]
    found = _check_warnings(check, views)
    level = {w["device"]: _LEVELS[w["code"]] for w in found if w["code"] in _LEVELS}
    warnings = [
        w
        for w in warnings
        if w["code"] not in _RANGE or level.get(w["device"], 0) < _LEVELS[w["code"]]
    ]
    level = {w["device"]: _LEVELS[w["code"]] for w in warnings if w["code"] in _RANGE}
    return warnings + [
        w
        for w in found
        if w["code"] not in _LEVELS or level.get(w["device"], 0) <= _LEVELS[w["code"]]
    ]
