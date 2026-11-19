"""Anchor positions from a tag's own geometry fit.

With automatic geometry (rtls-link-zephyr#210) a tag positions on the cell it
fitted at boot, not on its provisioned ``UWB_AN*_X/Y/Z`` table, which may be
stale or never set. The tag streams the fitted AN0-ANi distances; the
positions follow from them by the deployment convention the tag fits
(``docs/reference/auto-geometry.md`` in the firmware):

    AN0 origin, AN1 (d1, 0, 0), AN2 (0, d2, 0), AN3 (d1, d2, 0),
    AN4..AN7 the top plane at -h (h = d4), each leg the remaining side of its
    right triangle.

Pure functions over the cached stats, like the agreement check.
"""

from __future__ import annotations

import math
from typing import Any, Optional

__all__ = ("fitted_anchor_positions",)

#: firmware ``geom`` state of an applied fit
STATE_CALIBRATED = 3

Ned = tuple[float, float, float]


def _leg(hypotenuse: float, known: float) -> Optional[float]:
    square = hypotenuse * hypotenuse - known * known
    return math.sqrt(square) if square > 0.0 else None


def fitted_anchor_positions(stats: Optional[dict[str, Any]]) -> Optional[list[Ned]]:
    """NED positions (north, east, down) of AN0..AN3 or AN0..AN7 from the
    tag's fit, or ``None`` unless the tag reports a complete applied fit."""
    if not stats or stats.get("geometryState") != STATE_CALIBRATED:
        return None
    distances = list(stats.get("geometryDistancesM") or [])
    distances += [None] * (7 - len(distances))
    d1, d2, d3, h, d5, d6, d7 = distances[:7]
    if d1 is None or d2 is None or d3 is None:
        return None
    bottom: list[Ned] = [(0.0, 0.0, 0.0), (d1, 0.0, 0.0), (0.0, d2, 0.0), (d1, d2, 0.0)]
    top = (h, d5, d6, d7)
    if all(d is None for d in top):
        return bottom
    if any(d is None for d in top):
        return None
    x5 = _leg(d5, h)
    y6 = _leg(d6, h)
    x7 = _leg(_leg(d7, h) or 0.0, y6) if y6 is not None else None
    if x5 is None or y6 is None or x7 is None:
        return None
    return bottom + [(0.0, 0.0, -h), (x5, 0.0, -h), (0.0, y6, -h), (x7, y6, -h)]
