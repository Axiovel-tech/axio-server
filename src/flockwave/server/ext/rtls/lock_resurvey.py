"""The lock's resurvey check (X-RTLS-LOCK ``resurvey``).

The live range check sees only the distance to the initiator, so an anchor
carried sideways hardly shows. On request the lock first measures the cell
as a survey does, fits it in the frame of the positions the anchors store
and warns of every anchor whose fitted position is off the stored one, at
the range check's two bounds. Nothing is held or written; a check that
cannot run is a warning too.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Optional

from .survey_fit import Frame, SurveyFit
from .survey_view import range_level

if TYPE_CHECKING:
    from .anchor_lock import AnchorView, LockWarning
    from .extension import RtlsExtension

__all__ = ("resurvey_warnings", "stored_frame")

#: ``UWB_LOC`` stores millimetres: the origin and the +x anchor of the
#: frame a commit wrote are within half of one of the axes
_ON_AXIS_M = 0.0015

#: anchors whose stored heights are closer than this, one after the other,
#: are one height group, metres
_HEIGHT_GAP_M = 0.25


def stored_frame(stored: Mapping[int, Sequence[float]]) -> Optional[Frame]:
    """The frame the stored positions were fitted in: the anchor at x = y =
    0 is its origin, the nearest on +x its +x anchor, anchors at about one
    height a height group and the groups above the origin's upper. None
    without an origin and a +x anchor."""
    origin = next(
        (a for a, p in stored.items() if max(abs(p[0]), abs(p[1])) <= _ON_AXIS_M),
        None,
    )
    on_x = [
        a for a, p in stored.items() if abs(p[1]) <= _ON_AXIS_M and p[0] > _ON_AXIS_M
    ]
    if origin is None or not on_x:
        return None
    groups: list[list[int]] = []
    for anchor in sorted(stored, key=lambda a: stored[a][2]):
        if groups and stored[anchor][2] - stored[groups[-1][-1]][2] < _HEIGHT_GAP_M:
            groups[-1].append(anchor)
        else:
            groups.append([anchor])
    level = next(group for group in groups if origin in group)
    # down is +z: a group before the origin's stands above it
    upper = [a for group in groups[: groups.index(level)] for a in group]
    return Frame(
        origin=origin,
        x_axis=min(on_x, key=lambda a: stored[a][0]),
        height_groups=tuple(tuple(sorted(group)) for group in groups),
        upper=tuple(sorted(upper)),
    )


def _fit_warnings(fit: SurveyFit, names: Mapping[int, str]) -> list["LockWarning"]:
    """An anchor the fit cannot locate, and one off its stored position. A
    pair the fit lacks blocks a commit, not the check."""
    warnings: list["LockWarning"] = []
    unlocated = {e.device: e.detail for e in fit.blocked if e.code == "not-located"}
    for system_id, report in fit.anchors.items():
        name = names.get(system_id, f"RTLS anchor {system_id}")
        cm = (report.moved_m or 0.0) * 100
        level = range_level(report.moved_m)
        if system_id in unlocated:
            warnings.append(
                _failed(system_id, f"{name} not located: {unlocated[system_id]}")
            )
        elif level == "danger":
            detail = f"{name} moved: {cm:.0f} cm off its surveyed position"
            warnings.append(_warning("resurvey-moved-danger", system_id, detail))
        elif level == "warning":
            detail = f"{name} is {cm:.0f} cm off its surveyed position"
            warnings.append(_warning("resurvey-moved", system_id, detail))
    return warnings


def _warning(code: str, device: Optional[int], detail: str) -> "LockWarning":
    return {"code": code, "device": device, "detail": detail}


def _failed(device: Optional[int], detail: str) -> "LockWarning":
    return _warning("resurvey-failed", device, detail)


async def resurvey_warnings(
    ext: "RtlsExtension", views: Sequence["AnchorView"]
) -> list["LockWarning"]:
    """Measures the cell and returns the check's lock warnings."""
    fit, reason = await ext._survey.check()
    if fit is None:
        return [_failed(None, f"Resurvey check failed: {reason}")]
    return _fit_warnings(fit, {view["systemId"]: view["name"] for view in views})
