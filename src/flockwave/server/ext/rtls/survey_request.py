"""The members of an X-RTLS-SURVEY request as the survey uses them, and the
frame and placement as the state object echoes them."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any, Optional

from .survey_fit import Frame

__all__ = (
    "DEFAULT_ROUNDS",
    "Placement",
    "frame_json",
    "parse_frame",
    "parse_placement",
    "parse_start",
    "placement_json",
)

DEFAULT_ROUNDS = 100
#: what a step command carries
ROUNDS = range(20, 1001)

#: rough ``(x, y)`` or ``(x, y, z)`` per anchor, metres in the declared frame
Placement = dict[int, tuple[float, ...]]


def _is_id(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 1 <= value <= 255


def _ids(value: Any, what: str) -> list[int]:
    if not isinstance(value, list) or not all(_is_id(entry) for entry in value):
        raise ValueError(f"Invalid {what}: expected a list of system IDs")
    return value


def parse_start(body: dict[str, Any]) -> tuple[Optional[list[int]], int, Any]:
    """The anchors a start names (None: every online anchor), its rounds
    and its frame as sent.

    Raises:
        ValueError: for a malformed member.
    """
    anchors = body.get("anchors")
    if anchors is not None and len(set(_ids(anchors, "anchors"))) != len(anchors):
        raise ValueError("Invalid anchors: an anchor is listed twice")
    rounds = body.get("rounds", DEFAULT_ROUNDS)
    if isinstance(rounds, float) and rounds.is_integer():
        rounds = int(rounds)
    if type(rounds) is not int or rounds not in ROUNDS:
        raise ValueError(f"Invalid rounds: {rounds!r} (expected 20 to 1000)")
    return anchors, rounds, body.get("frame")


def parse_frame(raw: Any, anchors: Sequence[int]) -> Frame:
    """The frame a request declares: spelled out, or the preset filled in
    from ``anchors`` in slot order.

    Raises:
        ValueError: for a malformed frame.
        SurveyFitError: when the preset does not fit the group.
    """
    if not isinstance(raw, dict):
        raise ValueError("Missing or invalid frame")
    origin, x_axis = raw.get("origin"), raw.get("xAxis")
    if origin is None and x_axis is None:
        if raw.get("preset") != "two-tiers":
            raise ValueError("Invalid frame: no origin and +x anchor, no known preset")
        return Frame.two_tiers(anchors)
    y_positive = raw.get("yPositive")
    if not (_is_id(origin) and _is_id(x_axis)) or not (
        y_positive is None or _is_id(y_positive)
    ):
        raise ValueError("Invalid frame: origin, xAxis and yPositive are system IDs")
    groups = raw.get("heightGroups") or []
    if not isinstance(groups, list):
        raise ValueError("Invalid frame: heightGroups is a list of lists")
    return Frame(
        origin=origin,
        x_axis=x_axis,
        height_groups=tuple(tuple(_ids(group, "height group")) for group in groups),
        upper=tuple(_ids(raw.get("upper") or [], "upper anchors")),
        y_positive=y_positive,
    )


def frame_json(frame: Frame) -> dict[str, Any]:
    spelled: dict[str, Any] = {
        "origin": frame.origin,
        "xAxis": frame.x_axis,
        "heightGroups": [list(group) for group in frame.height_groups],
        "upper": list(frame.upper),
    }
    if frame.y_positive is not None:
        spelled["yPositive"] = frame.y_positive
    return spelled


def _coordinate(point: Any, axis: str) -> float:
    value = point.get(axis) if isinstance(point, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Invalid placement: {axis} is a number of metres")
    if not math.isfinite(value):
        raise ValueError(f"Invalid placement: {axis} is not finite")
    return float(value)


def parse_placement(raw: Any) -> Optional[Placement]:
    """The rough placement of a request, None when it places no anchor.

    Raises:
        ValueError: for a malformed placement.
    """
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("Invalid placement: expected an object by system ID")
    placement: Placement = {}
    for key, point in raw.items():
        if not str(key).isdecimal():
            raise ValueError(f"Invalid placement: {key!r} is not a system ID")
        axes = "xyz" if isinstance(point, dict) and point.get("z") is not None else "xy"
        placement[int(key)] = tuple(_coordinate(point, axis) for axis in axes)
    return placement or None


def placement_json(placement: Optional[Placement]) -> Optional[dict[str, Any]]:
    if placement is None:
        return None
    return {str(key): dict(zip("xyz", point)) for key, point in placement.items()}
