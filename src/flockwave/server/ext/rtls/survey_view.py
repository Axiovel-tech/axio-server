"""The survey the cell is on, as its anchors and tags report it.

The anchors own the geometry: each stores its position and a survey ID
(``UWB_LOC``) and advertises them, and a tag positions on the anchors of one
survey ID that it hears. An anchor is on the survey ID its ranging session
advertises (the ``sid`` stat); while it runs no session, on the one it
stores. The cell is on the ID most online anchors are on. From that follow
the ``survey`` member of X-RTLS-INF with its fleet warning, two lock
warnings and two Verify rules.

One server is one cell, as for the cell settings.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
from contextlib import suppress
from dataclasses import dataclass
from functools import reduce
from typing import TYPE_CHECKING, Any, Optional

from rtlslink import anchor_location
from rtlslink.anchor_location import StoredLocation

from .tag_table import IN_USE

if TYPE_CHECKING:
    from .anchor_lock import AnchorView, LockWarning
    from .extension import RtlsExtension

__all__ = (
    "RANGE_DEVIATION_WARNING_M",
    "SURVEY_DRIFT_WARNING_M",
    "anchors_rule",
    "location_from_params",
    "newest_survey_id",
    "stored_location",
    "survey_inputs",
    "survey_json",
    "survey_rule",
    "survey_warnings",
)

#: an anchor warns while the range to its stored initiator is farther than
#: this from the one its survey measured, metres, either sign. The firmware's
#: own bound (rtls-link-zephyr ``docs/reference/anchor-health.md``, "Range
#: check"), which its ring shows.
RANGE_DEVIATION_WARNING_M = 0.08

#: a tag's live initiator ranges farther than this from the distances between
#: the advertised positions are worth a look, metres. The stat includes the
#: survey fit's own residual on each pair, so it informs and blocks nothing.
SURVEY_DRIFT_WARNING_M = 0.10

#: a tag's snapshot older than this many seconds says nothing of its table
STATS_FRESH_S = 10.0

_TABLE_STATS = {"anchorsUsed", "anchorsExcluded"}


def location_from_params(params: Mapping[str, Any]) -> Optional[StoredLocation]:
    """The location in an anchor's decoded ``UWB_LOC``, None without one."""
    with suppress(ValueError):
        return anchor_location.decode_param(str(params.get(anchor_location.PARAM, "")))
    return None


def stored_location(device: Any) -> Optional[StoredLocation]:
    """The location in a device's cached ``UWB_LOC``, None without one."""
    from .extension import _decoded_device_params

    return location_from_params(
        _decoded_device_params(device, (anchor_location.PARAM,))
    )


def newest_survey_id(ids: Sequence[int]) -> int:
    """The newest of some survey IDs in 16-bit serial order, in which the
    commits allocate them and a tag settles a tie."""
    return reduce(
        lambda newest, other: other if (other - newest) & 0xFFFF < 0x8000 else newest,
        ids,
    )


@dataclass(frozen=True)
class _Anchor:
    """An online anchor and the survey it is on."""

    system_id: int
    name: str
    survey_id: Optional[int]
    range_deviation_m: Optional[float]
    moved: bool

    @property
    def deviates(self) -> bool:
        deviation = self.range_deviation_m
        return deviation is not None and abs(deviation) > RANGE_DEVIATION_WARNING_M


@dataclass(frozen=True)
class _Tag:
    """An online, awake tag and what it reports of its table."""

    system_id: int
    #: None while it reports no table: an old snapshot, or stats it lacks
    stats: Optional[dict[str, Any]]

    @property
    def excluded(self) -> list[dict[str, Any]]:
        """The anchors its table names as left out."""
        table = (self.stats or {}).get("anchorTable") or []
        return [entry for entry in table if entry["state"] != IN_USE]


def _anchors(ext: "RtlsExtension", views: Sequence["AnchorView"]) -> list[_Anchor]:
    devices = ext._protocol.devices if ext._protocol else {}
    anchors = []
    for view in views:
        device = devices.get(view["systemId"])
        if not view["online"] or device is None:
            continue
        stats = view["stats"]
        survey_id = stats.get("surveyId")
        if survey_id is None and stats.get("cellConfig") is None:
            stored = stored_location(device)
            survey_id = None if stored is None else stored.location.survey_id
        anchors.append(
            _Anchor(
                view["systemId"],
                view["name"],
                survey_id,
                stats.get("rangeDeviationM"),
                bool(stats.get("moved")),
            )
        )
    return anchors


def survey_inputs(stats: Mapping[str, Any]) -> tuple[Any, ...]:
    """What of a device's stats the ``survey`` member of X-RTLS-INF is made
    from: it is pushed when this changes."""
    deviation = stats.get("rangeDeviationM")
    table = stats.get("anchorTable")
    return (
        stats.get("surveyId"),
        stats.get("cellConfig") is None,
        deviation is not None and abs(deviation) > RANGE_DEVIATION_WARNING_M,
        stats.get("anchorsExcluded"),
        table and [entry for entry in table if entry["state"] != IN_USE],
        stats.get("sleeping"),
    )


def _cell_survey_id(anchors: Sequence[_Anchor]) -> Optional[int]:
    """The survey ID most anchors are on; of several, the newest."""
    counts = Counter(a.survey_id for a in anchors if a.survey_id is not None)
    if not counts:
        return None
    most = max(counts.values())
    return newest_survey_id([sid for sid, count in counts.items() if count == most])


def _tags(ext: "RtlsExtension", now: float) -> list[_Tag]:
    """The online tags that are awake; a device whose role is not read yet
    counts as one, so that it is named instead of passed over."""
    devices = ext._protocol.devices if ext._protocol else {}
    tags = []
    for system_id, device in sorted(devices.items()):
        stats = ext._fresh_stats(system_id)
        asleep = bool(
            getattr(device, "sleeping", False) or (stats or {}).get("sleeping")
        )
        if ext._device_role(system_id) not in (None, "tag") or asleep:
            continue
        old = now - ext._stats_at.get(system_id, float("-inf")) > STATS_FRESH_S
        reports = stats is not None and _TABLE_STATS <= stats.keys() and not old
        tags.append(_Tag(system_id, stats if reports else None))
    return tags


def survey_json(
    ext: "RtlsExtension", views: Sequence["AnchorView"], now: float
) -> dict[str, Any]:
    """The ``survey`` member of X-RTLS-INF: the ID the cell is on, how many
    online anchors are on it, the online anchors that are not, the anchors
    each tag leaves out and the anchors whose range check warns."""
    anchors = _anchors(ext, views)
    cell = _cell_survey_id(anchors)
    excluding = {}
    for tag in _tags(ext, now):
        if tag.stats is not None and (tag.excluded or tag.stats["anchorsExcluded"]):
            excluding[str(tag.system_id)] = [
                {key: entry[key] for key in ("mac", "systemId", "state")}
                for entry in tag.excluded
            ]
    return {
        "id": cell,
        "anchors": sum(1 for a in anchors if cell is not None and a.survey_id == cell),
        "notSurveyed": [a.system_id for a in anchors if a.survey_id is None],
        "otherSurvey": [
            a.system_id for a in anchors if a.survey_id not in (None, cell)
        ],
        "excluding": excluding,
        "rangeDeviation": [a.system_id for a in anchors if a.deviates],
    }


def _off_survey(anchor: _Anchor, cell: Optional[int]) -> Optional[str]:
    """How an anchor is off the cell's survey, None while it is on it."""
    if anchor.survey_id is None:
        return f"{anchor.name} is not surveyed"
    if anchor.survey_id != cell:
        return f"{anchor.name} is on survey {anchor.survey_id}, the cell on {cell}"
    return None


def _deviation(anchor: _Anchor) -> str:
    deviation = anchor.range_deviation_m or 0.0
    where = "closer to" if deviation < 0 else "farther from"
    return (
        f"{anchor.name} is {abs(deviation) * 100:.0f} cm {where} its initiator "
        "than when surveyed"
    )


def survey_warnings(
    ext: "RtlsExtension", views: Sequence["AnchorView"]
) -> list["LockWarning"]:
    """The lock preflight's acknowledgeable ``not-surveyed`` and
    ``range-deviation`` warnings."""
    anchors = _anchors(ext, views)
    cell = _cell_survey_id(anchors)
    warnings: list["LockWarning"] = []
    for anchor in anchors:
        off = _off_survey(anchor, cell)
        if off is not None:
            warnings.append(
                {"code": "not-surveyed", "device": anchor.system_id, "detail": off}
            )
        if anchor.deviates:
            warnings.append(
                {
                    "code": "range-deviation",
                    "device": anchor.system_id,
                    "detail": _deviation(anchor),
                }
            )
    return warnings


def _tag_problems(tag: _Tag, cell: int, surveyed: int) -> list[str]:
    """Why a tag does not position on the cell's survey with every anchor."""
    name = f"tag {tag.system_id}"
    stats = tag.stats
    if stats is None:
        return [f"{name}: no survey telemetry"]
    following = stats.get("surveyId")
    if following != cell:
        on = "no survey" if following is None else f"survey {following}"
        return [f"{name} follows {on}, the cell is on {cell}"]
    problems = []
    excluded = stats["anchorsExcluded"]
    if excluded:
        named = ", ".join(
            f"{_anchor_label(entry)} ({entry['state']})" for entry in tag.excluded
        )
        problems.append(
            f"{name} leaves out {excluded} anchor(s)" + (f": {named}" if named else "")
        )
    if stats["anchorsUsed"] != surveyed:
        problems.append(
            f"{name} uses {stats['anchorsUsed']} of {surveyed} surveyed anchors"
        )
    return problems


def _anchor_label(entry: Mapping[str, Any]) -> str:
    if entry["systemId"] is not None:
        return f"anchor {entry['systemId']}"
    return f"MAC {entry['mac']:#06x}"


def _rule(rule_id: str, label: str, severity: str, problems: list[str], ok: str):
    return {
        "id": rule_id,
        "label": label,
        "severity": severity,
        "status": "fail" if problems else "pass",
        "detail": "; ".join(problems) or ok,
    }


def survey_rule(
    ext: "RtlsExtension", views: Sequence["AnchorView"], now: float
) -> dict[str, Any]:
    """The X-RTLS-VERIFY ``survey`` rule: an error unless every online
    anchor is on one survey and every online, awake tag follows it, leaves
    out no anchor and uses as many anchors as are on it."""
    anchors = _anchors(ext, views)
    cell = _cell_survey_id(anchors)
    problems: list[str] = []
    if not anchors:
        problems.append("no online anchors")
    elif cell is None:
        problems.append("no anchor is surveyed")
    else:
        surveyed = sum(1 for anchor in anchors if anchor.survey_id == cell)
        problems += [off for a in anchors if (off := _off_survey(a, cell)) is not None]
        for tag in _tags(ext, now):
            problems += _tag_problems(tag, cell, surveyed)
    ok = f"every anchor is on survey {cell} and every tag uses all of them"
    return _rule("survey", "Anchor survey", "error", problems, ok)


def anchors_rule(
    ext: "RtlsExtension", views: Sequence["AnchorView"], now: float
) -> dict[str, Any]:
    """The X-RTLS-VERIFY ``anchors`` rule: a warning while an anchor's range
    check warns or it reports ``moved``, or a tag's live ranges drifted from
    the advertised positions."""
    problems = []
    for anchor in _anchors(ext, views):
        if anchor.deviates:
            problems.append(_deviation(anchor))
        if anchor.moved:
            problems.append(f"{anchor.name} moved since it was placed")
    for tag in _tags(ext, now):
        drift = (tag.stats or {}).get("surveyDriftM")
        if drift is not None and drift > SURVEY_DRIFT_WARNING_M:
            problems.append(
                f"tag {tag.system_id}: live ranges are {drift * 100:.0f} cm off "
                "the surveyed positions"
            )
    ok = "no anchor moved and no tag sees its ranges drift"
    return _rule("anchors", "Anchor positions", "warning", problems, ok)
