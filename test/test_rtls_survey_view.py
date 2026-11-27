"""The survey the cell is on (rtls/survey_view.py): the ``survey`` member of
X-RTLS-INF with its fleet warning, the lock warnings and the Verify rules
``survey``, ``frame`` and ``anchors``."""

import time

import numpy as np
import pytest
import trio
from anchor_lock_support import _lock, location
from rtls_support import FakeDevice, discover, set_fake_param
from rtlslink.dialect import load_dialect
from survey_support import ASLEEP
from survey_view_support import (
    ANCHORS,
    NO_POSITION,
    OTHER_SURVEY,
    TAG,
    TAG_STATS,
    anchors_report,
    antbl,
    give_origin,
    inf,
    surveyed_cell,
    tag_reports,
)

from flockwave.server.ext.rtls.survey_view import positioned
from flockwave.server.ext.rtls.verify import run_verify
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder


@pytest.fixture(scope="module")
def dialect():
    return load_dialect()


@pytest.fixture
def hub():
    return MessageHub()


@pytest.fixture
def builder():
    return FlockwaveMessageBuilder()


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


async def _survey(ext, builder, hub):
    return (await inf(ext, builder, hub))["survey"]


async def _rule(ext, rule_id):
    body = await run_verify(ext)
    return next(rule for rule in body["rules"] if rule["id"] == rule_id)


async def _positioned(dialect, **cell):
    """A surveyed cell whose anchors and tag all report survey 1."""
    ext, devices = await surveyed_cell(dialect, **cell)
    await anchors_report(ext)
    await tag_reports(ext)
    return ext, devices


async def _second_tag(ext, dialect, system_id=43, cell=None, **origin):
    tag = FakeDevice(dialect, system_id=system_id)
    tag.address = (f"192.168.4.{system_id}", 3333)
    set_fake_param(tag, "UWB_ROLE", 1, "uint8")
    if cell is not None:
        set_fake_param(tag, "CELL_ID", cell, "custom")
    give_origin(tag, **origin)
    ext.routes[tag.address] = tag
    await discover(ext, tag)
    return tag


# ---- X-RTLS-INF survey ------------------------------------------------------


async def test_the_cell_is_on_the_survey_its_anchors_advertise(dialect, builder, hub):
    ext, _ = await _positioned(dialect)

    assert await _survey(ext, builder, hub) == {
        "id": 1,
        "anchors": 4,
        "notSurveyed": [],
        "otherSurvey": [],
        "excluding": {},
        "rangeDeviation": [],
        "rangeDanger": [],
    }


async def test_anchors_off_the_cells_survey_are_named(dialect, builder, hub):
    ext, _ = await _positioned(dialect)

    await anchors_report(ext, [203], sid=0.0)
    await anchors_report(ext, [204], sid=9.0)

    survey = await _survey(ext, builder, hub)
    assert (survey["id"], survey["anchors"]) == (1, 2)
    assert (survey["notSurveyed"], survey["otherSurvey"]) == ([203], [204])


async def test_no_survey_when_no_anchor_advertises_one(dialect, builder, hub):
    ext, _ = await surveyed_cell(dialect, blank=(201, 202, 203, 204))
    await anchors_report(ext, sid=0.0)

    survey = await _survey(ext, builder, hub)

    assert (survey["id"], survey["anchors"]) == (None, 0)
    assert survey["notSurveyed"] == [201, 202, 203, 204]


@pytest.mark.parametrize(("older", "newer"), [(1, 2), (0xFFFE, 1), (0x7000, 0x9000)])
async def test_of_two_surveys_held_equally_the_newer_is_the_cells(
    dialect, builder, hub, older, newer
):
    ext, _ = await surveyed_cell(dialect)
    await anchors_report(ext, [201, 204], sid=float(newer))
    await anchors_report(ext, [202, 203], sid=float(older))

    survey = await _survey(ext, builder, hub)

    assert (survey["id"], survey["otherSurvey"]) == (newer, [202, 203])


async def test_an_anchor_without_a_session_is_on_the_survey_it_stores(
    dialect, builder, hub
):
    ext, _ = await surveyed_cell(dialect, blank=(203,))
    # before any report, and asleep or restarting after one
    assert (await _survey(ext, builder, hub))["anchors"] == 3

    await anchors_report(ext, **ASLEEP)

    survey = await _survey(ext, builder, hub)
    assert (survey["id"], survey["anchors"]) == (1, 3)
    assert survey["notSurveyed"] == [203]


async def test_a_ranging_anchor_that_advertises_nothing_is_not_surveyed(
    dialect, builder, hub
):
    # its radio refused the location it stores: it ranges, sid reads 0
    ext, _ = await _positioned(dialect)

    await anchors_report(ext, [202], sid=0.0)

    assert (await _survey(ext, builder, hub))["notSurveyed"] == [202]


async def test_an_offline_anchor_is_in_no_list(dialect, builder, hub):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [204], sid=9.0)

    ext._protocol.devices[204].last_seen = -1e9
    ext._expire_devices(time.monotonic())

    survey = await _survey(ext, builder, hub)
    assert (survey["anchors"], survey["otherSurvey"]) == (3, [])


@pytest.mark.parametrize(
    ("deviation", "warns"),
    [
        (0.08, False),
        (-0.08, False),
        (0.08004, True),
        (-0.08004, True),
        (0.0801, True),
        (-0.12, True),
        (None, False),
    ],
)
async def test_the_range_check_warns_strictly_above_its_bound(
    dialect, builder, hub, deviation, warns
):
    ext, _ = await _positioned(dialect)

    stats = {} if deviation is None else {"rdev": deviation}
    await anchors_report(ext, [203], **stats)

    survey = await _survey(ext, builder, hub)
    assert survey["rangeDeviation"] == ([203] if warns else [])


@pytest.mark.parametrize("bound", [0.08, 0.2])
@pytest.mark.parametrize("sign", [1, -1])
async def test_a_float32_reading_at_a_bound_is_the_level_below_it(
    dialect, builder, hub, bound, sign
):
    # the wire carries 0.2F as 0.2000000030: the ring compares it with
    # 0.2F and shows a warning, not a danger
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [203], rdev=sign * float(np.float32(bound)))

    survey = await _survey(ext, builder, hub)
    assert (survey["rangeDeviation"], survey["rangeDanger"]) == (
        [203] if bound == 0.2 else [],
        [],
    )


async def test_a_deviation_just_over_the_bound_warns_everywhere(dialect, builder, hub):
    # the firmware's ring warns at 0.08004 m: so must the server, which
    # used to round the value to 0.0800 before it compared
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [203], rdev=0.08004)

    assert ext._stats[203]["rangeDeviationM"] == 0.08004
    assert (await _survey(ext, builder, hub))["rangeDeviation"] == [203]
    warnings = (await _lock(ext, builder, hub, "status"))["warnings"]
    assert [(w["code"], w["device"]) for w in warnings] == [("range-deviation", 203)]
    rule = await _rule(ext, "anchors")
    assert (rule["status"], rule["detail"]) == (
        "fail",
        "RTLS anchor A2 is 8 cm off its surveyed range",
    )


async def test_the_anchors_a_tag_leaves_out_are_the_fleet_warning(
    dialect, builder, hub
):
    ext, _ = await _positioned(dialect)

    await tag_reports(
        ext,
        antbl(1, row=0),
        antbl(4, state=OTHER_SURVEY),
        antbl(0x0777, state=NO_POSITION),
        anexc=2.0,
    )

    assert (await _survey(ext, builder, hub))["excluding"] == {
        str(TAG): [
            {"mac": 4, "systemId": 204, "state": "other-survey"},
            {"mac": 0x0777, "systemId": None, "state": "no-position"},
        ]
    }


async def test_a_tag_that_counts_an_anchor_out_before_naming_it_is_listed(
    dialect, builder, hub
):
    ext, _ = await _positioned(dialect)

    await tag_reports(ext, antbl(1, row=0), anexc=1.0)

    assert (await _survey(ext, builder, hub))["excluding"] == {str(TAG): []}


async def test_a_tag_that_is_asleep_or_silent_warns_of_nothing(dialect, builder, hub):
    ext, _ = await _positioned(dialect)
    left_out = (antbl(4, state=OTHER_SURVEY),)

    await tag_reports(ext, *left_out, anexc=1.0, slp=1.0)
    assert (await _survey(ext, builder, hub))["excluding"] == {}

    await tag_reports(ext, *left_out, anexc=1.0)
    assert str(TAG) in (await _survey(ext, builder, hub))["excluding"]
    ext._stats_at[TAG] = time.monotonic() - 11.0
    assert (await _survey(ext, builder, hub))["excluding"] == {}


async def test_a_change_of_the_survey_is_pushed(dialect):
    ext, _ = await _positioned(dialect)

    async def pushes(report, *args, **stats):
        ext._inf_pending = False
        await report(ext, *args, **stats)
        return ext._inf_pending

    assert not await pushes(anchors_report)
    assert not await pushes(tag_reports)
    # a table that fills with rows in use is no warning
    assert not await pushes(tag_reports, antbl(2, row=1), antbl(3, row=2))

    assert await pushes(anchors_report, [203], sid=2.0)
    assert not await pushes(anchors_report, [203], sid=2.0)
    assert await pushes(anchors_report, [204], rdev=0.2)
    assert not await pushes(anchors_report, [204], rdev=-0.19)
    assert await pushes(anchors_report, [204], rdev=0.21)
    assert not await pushes(anchors_report, [204], rdev=-0.3)
    assert await pushes(tag_reports, antbl(3, state=OTHER_SURVEY), anexc=1.0)
    assert not await pushes(tag_reports, antbl(3, state=OTHER_SURVEY), anexc=1.0)
    assert await pushes(tag_reports, antbl(3, row=2))


# ---- X-RTLS-LOCK warnings ---------------------------------------------------


async def test_lock_warns_of_anchors_off_the_survey_and_of_a_range_deviation(
    dialect, builder, hub
):
    ext, devices = await _positioned(dialect, blank=(203,))
    await anchors_report(ext, [203], sid=0.0)
    await anchors_report(ext, [204], sid=9.0)
    await anchors_report(ext, [202], rdev=-0.12)

    body = await _lock(ext, builder, hub)

    assert body["result"] == "warnings"
    assert [(w["code"], w["device"], w["detail"]) for w in body["warnings"]] == [
        (
            "range-deviation",
            202,
            "RTLS anchor A1 is 12 cm off its surveyed range",
        ),
        ("not-surveyed", 203, "RTLS anchor A2 is not surveyed"),
        ("not-surveyed", 204, "RTLS anchor A3 is on survey 9, the cell on 1"),
    ]
    assert devices[201].params["LOCK"][0][0] == 0

    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"


@pytest.mark.parametrize(
    "deviation,code,detail",
    [
        (0.094, "range-deviation", "RTLS anchor A2 is 9 cm off its surveyed range"),
        (0.2, "range-deviation", "RTLS anchor A2 is 20 cm off its surveyed range"),
        (
            -0.2001,
            "range-deviation-danger",
            "RTLS anchor A2 moved: 20 cm off its surveyed range",
        ),
        (
            0.35,
            "range-deviation-danger",
            "RTLS anchor A2 moved: 35 cm off its surveyed range",
        ),
    ],
)
async def test_lock_warns_at_the_range_checks_two_levels(
    dialect, builder, hub, deviation, code, detail
):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [203], rdev=deviation)

    body = await _lock(ext, builder, hub)

    assert [(w["code"], w["device"], w["detail"]) for w in body["warnings"]] == [
        (code, 203, detail)
    ]
    survey = await _survey(ext, builder, hub)
    danger = code == "range-deviation-danger"
    assert (survey["rangeDeviation"], survey["rangeDanger"]) == (
        [203],
        [203] if danger else [],
    )
    # a warning at either level: the operator may still lock
    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"


async def test_a_surveyed_cell_locks_without_a_warning(dialect, builder, hub):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [202], rdev=0.08)

    body = await _lock(ext, builder, hub)

    assert (body["result"], body["warnings"]) == ("locked", [])


# ---- X-RTLS-VERIFY survey ---------------------------------------------------


async def test_verify_passes_a_fleet_positioned_on_one_survey(dialect):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [202, 203, 204], rdev=0.01)

    body = await run_verify(ext)

    rules = {rule["id"]: rule for rule in body["rules"]}
    assert list(rules)[:3] == ["survey", "frame", "anchors"]
    assert rules["survey"] == {
        "id": "survey",
        "label": "Anchor survey",
        "severity": "error",
        "status": "pass",
        "detail": "every anchor is on survey 1 and every tag uses all of them",
    }
    assert (rules["frame"]["status"], rules["frame"]["severity"]) == ("pass", "error")
    assert (rules["anchors"]["status"], rules["anchors"]["severity"]) == (
        "pass",
        "warning",
    )
    assert "geometry" not in body and "geometry" not in rules


async def test_verify_fails_without_an_online_anchor(dialect):
    ext, _ = await surveyed_cell(dialect, anchors=())
    await tag_reports(ext)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == ("fail", "no online anchors")


async def test_verify_fails_when_no_anchor_is_surveyed(dialect):
    ext, _ = await surveyed_cell(dialect, blank=(201, 202, 203, 204))
    await anchors_report(ext, sid=0.0)
    await tag_reports(ext, sid=0.0, anuse=0.0)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == ("fail", "no anchor is surveyed")


async def test_verify_fails_on_an_anchor_off_the_survey(dialect):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [203], sid=0.0)
    await anchors_report(ext, [204], sid=9.0)
    await tag_reports(ext, anuse=2.0)

    rule = await _rule(ext, "survey")

    assert rule["status"] == "fail"
    assert rule["detail"] == (
        "RTLS anchor A2 is not surveyed; "
        "RTLS anchor A3 is on survey 9, the cell on 1; "
        f"tag {TAG} uses 2 anchor(s), a position needs 4"
    )


async def test_verify_fails_on_a_tag_that_follows_another_survey(dialect):
    ext, _ = await _positioned(dialect)
    await tag_reports(ext, sid=9.0)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        f"tag {TAG} follows survey 9, the cell is on 1",
    )


async def test_a_tag_is_positioned_by_its_survey_not_by_its_clock(dialect):
    # an initiator the tag leaves out still times the round: the show clock
    # is valid in a cell the tag has no survey of
    ext, _ = await _positioned(dialect)
    await tag_reports(ext, sid=0.0, anuse=0.0, clkok=1.0)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        f"tag {TAG} follows no survey, the cell is on 1",
    )


async def test_verify_fails_on_a_tag_that_leaves_anchors_out(dialect):
    ext, _ = await _positioned(dialect)
    await tag_reports(
        ext,
        antbl(4, state=OTHER_SURVEY),
        antbl(0x0777, state=NO_POSITION),
        anexc=3.0,
    )

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        f"tag {TAG} leaves out 3 anchor(s): RTLS anchor A3 (other-survey), "
        "MAC 0x0777 (no-position)",
    )


async def test_verify_names_a_left_out_anchor_the_server_lost(dialect):
    ext, _ = await _positioned(dialect, planar=True)
    ext._protocol.devices[204].last_seen = -1e9
    ext._expire_devices(time.monotonic())
    await tag_reports(ext, antbl(4, state=OTHER_SURVEY), anuse=3.0, anexc=1.0)

    rule = await _rule(ext, "survey")

    assert rule["detail"] == (
        f"tag {TAG} leaves out 1 anchor(s): RTLS anchor A3 (other-survey)"
    )


async def test_verify_fails_on_a_tag_that_uses_fewer_anchors_than_are_surveyed(dialect):
    ext, _ = await _positioned(dialect, planar=True)
    await tag_reports(ext, anuse=3.0)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        f"tag {TAG} uses 3 of 4 surveyed anchors",
    )


async def test_a_tag_on_three_anchors_is_positioned_only_in_a_planar_survey(dialect):
    # three anchors are all there are: the tag uses every one of them, and
    # still solves nothing unless the survey lets it lock its height
    ext, _ = await _positioned(dialect, anchors=ANCHORS[:3])
    await tag_reports(ext, anuse=3.0)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        f"tag {TAG} uses 3 anchor(s), a position needs 4",
    )

    ext, _ = await _positioned(dialect, anchors=ANCHORS[:3], planar=True)
    await tag_reports(ext, anuse=3.0)
    assert (await _rule(ext, "survey"))["status"] == "pass"
    await tag_reports(ext, anuse=2.0)
    assert (await _rule(ext, "survey"))["detail"] == (
        f"tag {TAG} uses 2 anchor(s), a position needs 3"
    )


@pytest.mark.parametrize(
    ("survey_id", "used", "planar", "is_positioned"),
    [
        (1, 4, False, True),
        (1, 3, False, False),
        (1, 3, True, True),
        (1, 2, True, False),
        (2, 8, False, False),
        (None, 4, False, False),
    ],
)
def test_positioned_is_the_survey_and_enough_anchors(
    survey_id, used, planar, is_positioned
):
    stats = {"surveyId": survey_id, "anchorsUsed": used}

    assert positioned(stats, 1, planar) is is_positioned
    assert positioned(stats, None, planar) is False


async def test_a_tag_is_expected_to_use_the_anchors_that_advertise_now(dialect):
    ext, _ = await _positioned(dialect, planar=True)

    # one anchor sleeps: it stores the survey and transmits nothing
    await anchors_report(ext, [204], **ASLEEP)
    await tag_reports(ext, anuse=3.0)

    rule = await _rule(ext, "survey")
    assert (rule["status"], rule["detail"]) == (
        "pass",
        "every anchor is on survey 1 and every tag uses all of them",
    )
    # what a tag misses of the anchors on air is still a fault
    await tag_reports(ext, anuse=2.0)
    assert (await _rule(ext, "survey"))["status"] == "fail"


async def test_a_sleeping_anchor_with_an_old_survey_is_its_own_finding(dialect):
    def prepare(devices):
        stored = location(0.0, 3.0, -2.5, 9, planar=True)
        set_fake_param(devices[204], "UWB_LOC", stored, "custom")

    ext, _ = await _positioned(dialect, planar=True, prepare=prepare)
    await anchors_report(ext, [204], **ASLEEP)
    await tag_reports(ext, anuse=3.0)

    rule = await _rule(ext, "survey")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        "RTLS anchor A3 is on survey 9, the cell on 1",
    )


async def test_verify_fails_on_a_tag_whose_survey_stats_are_old_or_missing(dialect):
    ext, _ = await _positioned(dialect)
    missing = f"tag {TAG}: no survey telemetry"

    ext._stats_at[TAG] = time.monotonic() - 11.0
    assert (await _rule(ext, "survey"))["detail"] == missing

    # a tag image from before the switch reports none of them
    old = {k: v for k, v in TAG_STATS.items() if k not in ("anuse", "anexc", "antbl")}
    await ext._on_stats(TAG, old, time.monotonic())
    assert (await _rule(ext, "survey"))["detail"] == missing

    ext._stats[TAG] = {**ext._stats[TAG], "stale": True}
    assert (await _rule(ext, "survey"))["detail"] == missing


async def test_verify_does_not_judge_a_sleeping_tag(dialect):
    ext, _ = await _positioned(dialect)
    await tag_reports(ext, 0.0, sid=0.0, anuse=0.0, slp=1.0)

    assert (await _rule(ext, "survey"))["status"] == "pass"


# ---- X-RTLS-VERIFY frame ----------------------------------------------------


async def test_the_frame_rule_is_skipped_without_a_tag(dialect):
    ext, _ = await _positioned(dialect)
    ext._protocol.devices[TAG].last_seen = -1e9
    ext._expire_devices(time.monotonic())

    rule = await _rule(ext, "frame")

    assert (rule["status"], rule["detail"]) == ("skipped", "no online tags")


@pytest.mark.parametrize(
    ("name", "value", "param_type"),
    [
        ("ORIGIN_LAT_E7", 413900001, "int32"),
        ("ORIGIN_LON_E7", 21500001, "int32"),
        ("ORIGIN_ALT_MM", 10500, "int32"),
        ("POS_YAW_DEG", 91.0, "real32"),
    ],
)
async def test_verify_fails_on_tags_that_hold_two_frames(
    dialect, name, value, param_type
):
    ext, _ = await _positioned(dialect)
    await _second_tag(ext, dialect, 43)
    odd = await _second_tag(ext, dialect, 44)
    assert (await _rule(ext, "frame"))["status"] == "pass"

    await ext.set_param(odd.system_id, name, value, param_type)

    rule = await _rule(ext, "frame")
    assert (rule["status"], rule["detail"]) == (
        "fail",
        f"tag 44: {name} differ from the rest of the cell",
    )


async def test_of_two_frames_held_equally_the_lowest_tags_is_the_reference(dialect):
    ext, _ = await _positioned(dialect)
    await _second_tag(ext, dialect, 43, ORIGIN_ALT_MM=0)

    rule = await _rule(ext, "frame")

    assert rule["detail"] == "tag 43: ORIGIN_ALT_MM differ from the rest of the cell"


async def test_a_yaw_is_one_revolution_modulo(dialect):
    ext, _ = await _positioned(dialect)
    second = await _second_tag(ext, dialect)

    await ext.set_param(second.system_id, "POS_YAW_DEG", 450.0, "real32")

    assert (await _rule(ext, "frame"))["status"] == "pass"


async def test_the_frame_rule_waits_for_a_parameter_not_read_yet(dialect):
    ext, _ = await _positioned(dialect)
    second = await _second_tag(ext, dialect, cell="default")
    del ext._protocol.devices[second.system_id].params["POS_YAW_DEG"]

    rule = await _rule(ext, "frame")

    assert (rule["status"], rule["detail"]) == (
        "fail",
        "tag 43: POS_YAW_DEG not read yet",
    )


async def test_the_frame_rule_waits_for_the_cell_of_a_partial_listing(dialect):
    ext, _ = await _positioned(dialect)
    second = await _second_tag(ext, dialect)
    ext._protocol.devices[second.system_id].param_count += 1

    rule = await _rule(ext, "frame")

    assert rule["detail"] == "tag 43: CELL_ID not read yet"


async def test_tags_of_two_cells_hold_their_own_frames(dialect):
    ext, _ = await _positioned(dialect)
    await _second_tag(ext, dialect, cell="hall-b", ORIGIN_LAT_E7=1)

    assert (await _rule(ext, "frame"))["status"] == "pass"


async def test_an_anchor_table_or_bias_a_tag_still_holds_is_not_its_frame(dialect):
    ext, _ = await _positioned(dialect)
    second = await _second_tag(ext, dialect)
    for name, value, param_type in (
        ("UWB_AN_COUNT", 8, "uint8"),
        ("UWB_AN1_MAC", 77, "uint16"),
        ("UWB_AN1_BIAS_M", 0.25, "real32"),
    ):
        set_fake_param(second, name, value, param_type)
    await ext.get_param_list(second.system_id)

    assert (await _rule(ext, "frame"))["status"] == "pass"
    assert (await _rule(ext, "survey"))["status"] == "fail"  # 43 reports nothing


# ---- X-RTLS-VERIFY anchors --------------------------------------------------


async def test_verify_warns_of_both_range_check_levels_and_drift(dialect):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [202], rdev=0.0801)
    await anchors_report(ext, [203], rdev=-0.25)
    # the retired motion watch of older firmware says nothing
    await anchors_report(ext, [204], moved=1.0)
    await tag_reports(ext, sdrift=0.1234)

    rule = await _rule(ext, "anchors")

    assert (rule["severity"], rule["status"]) == ("warning", "fail")
    assert rule["detail"] == (
        "RTLS anchor A1 is 8 cm off its surveyed range; "
        "RTLS anchor A2 moved: 25 cm off its surveyed range; "
        f"tag {TAG}: live ranges are 12 cm off the surveyed positions"
    )
    # judgment material: it does not fail the survey itself
    assert (await _rule(ext, "survey"))["status"] == "pass"


async def test_the_anchors_rule_passes_at_its_bounds(dialect):
    ext, _ = await _positioned(dialect)
    await anchors_report(ext, [202, 203, 204], rdev=-0.08)
    await tag_reports(ext, sdrift=0.10)

    rule = await _rule(ext, "anchors")

    # the initiator A0 is what the others range to: it has no check
    assert (rule["status"], rule["detail"]) == (
        "pass",
        "every anchor is on its surveyed range; no tag sees its ranges drift",
    )


async def test_the_anchors_rule_names_the_anchors_it_cannot_check(dialect):
    ext, _ = await _positioned(dialect)

    rule = await _rule(ext, "anchors")
    assert (rule["status"], rule["detail"]) == (
        "skipped",
        "no anchor reports its range check; no tag sees its ranges drift",
    )

    await anchors_report(ext, [203], rdev=0.01)
    rule = await _rule(ext, "anchors")
    assert (rule["status"], rule["detail"]) == (
        "pass",
        "range not checked: RTLS anchor A1, RTLS anchor A3; "
        "no tag sees its ranges drift",
    )
