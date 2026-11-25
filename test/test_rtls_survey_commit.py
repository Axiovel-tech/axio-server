"""The fit an operator asks again and the commit (X-RTLS-SURVEY ``fit`` and
``commit``, rtls/survey.py and survey_commit.py): the gate, the survey ID,
what each anchor is written, and an anchor that misses it."""

import math

import pytest
import trio
from anchor_lock_support import _lock
from cell_settings_support import _record_writes
from rtlslink.anchor_location import AnchorLocation, StoredLocation, encode_param
from rtlslink.dialect import load_dialect
from rtlslink.protocol import PARAM_TYPE_CUSTOM
from survey_support import (
    FLAT,
    LOC_PARAM,
    TIERS,
    TWO_TIERS,
    authorize_show,
    commit,
    measure,
    request,
    survey_cell,
    until,
)

from flockwave.server.ext.rtls import survey as survey_module
from flockwave.server.ext.rtls.extension import _stats_json
from flockwave.server.ext.rtls.survey_commit import next_survey_id, write_locations
from flockwave.server.ext.rtls.survey_group import Anchor
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

IDS = list(FLAT)
INCOMPLETE = ("incomplete", "Not every anchor took the survey")


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
async def cell(dialect, nursery, autojump_clock):
    return await survey_cell(dialect, nursery)


def _mm(metres):
    return round(metres * 1000)


def _range_mm(a, b, positions=FLAT):
    return _mm(math.dist(positions[a], positions[b]))


def _store(cell, system_id, survey_id):
    """The anchor holds a position of an earlier survey."""
    stored = StoredLocation(AnchorLocation(1000, 2000, -3000, survey_id))
    value = encode_param(stored).encode()
    cell.devices[system_id].params[LOC_PARAM] = (value, PARAM_TYPE_CUSTOM)


def _codes(entries):
    return [(entry["code"], entry["device"], entry["peer"]) for entry in entries]


@pytest.mark.parametrize(
    "stored, expected",
    [
        ([], 1),
        ([5], 6),
        ([5, 7, 6], 8),
        ([0xFFFD], 0xFFFE),
        # 0xFFFF and 0 are never advertised
        ([0xFFFE], 1),
        # past the wrap the small ID is the newer one
        ([0xFFFE, 2, 0xFFF0], 3),
        ([2, 0xFFFE], 3),
    ],
)
def test_the_survey_id_is_one_more_than_the_newest_stored(stored, expected):
    assert next_survey_id(stored) == expected


async def test_commit_writes_every_anchor_its_position_and_confirms(
    cell, builder, hub, autojump_clock
):
    writes = _record_writes(cell.ext)
    await measure(cell, builder, hub)

    answer = await commit(cell, builder, hub)

    assert (answer["action"], answer["result"], answer["reason"]) == (
        "commit",
        "committed",
        None,
    )
    survey = answer["survey"]
    assert (survey["state"], survey["reason"]) == ("committed", None)
    assert survey["commit"] == {
        "surveyId": 1,
        "results": {str(system_id): {"written": "changed"} for system_id in IDS},
        "running": IDS,
    }
    for system_id, (north, east, down) in FLAT.items():
        assert cell.stored(system_id).location == AnchorLocation(
            _mm(north), _mm(east), _mm(down), survey_id=1, planar=True
        )
    # the reference is each responder's own range to the stored initiator
    assert [cell.stored(system_id).reference_mm for system_id in IDS] == [
        0,
        _range_mm(201, 202),
        _range_mm(201, 203),
        _range_mm(201, 204),
    ]
    # the commit does not lock: Control does, with the lock's own warnings
    written = {name for device in writes.values() for name, _ in device}
    assert written == {"UWB_SURVEY", LOC_PARAM}
    assert (await _lock(cell.ext, builder, hub, "status"))["anchorLock"]["locked"] is (
        False
    )


async def test_a_two_tier_cell_is_not_planar(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await survey_cell(dialect, nursery, TIERS)
    lower, upper = list(TIERS)[:4], list(TIERS)[4:]
    frame = {
        "origin": 201,
        "xAxis": 202,
        "heightGroups": [lower, upper],
        "upper": upper,
        "yPositive": 203,
    }
    survey = await measure(cell, builder, hub, frame=frame)
    assert survey["frame"] == frame
    assert survey["fit"]["planar"] is False
    assert survey["steps"] == 8 and len(survey["pairs"]) == 28

    answer = await commit(cell, builder, hub)

    assert answer["result"] == "committed"
    for system_id, (north, east, down) in TIERS.items():
        stored = cell.stored(system_id)
        assert stored.location == AnchorLocation(
            _mm(north), _mm(east), _mm(down), survey_id=1, planar=False
        )
        expected = 0 if system_id == 201 else _range_mm(201, system_id, TIERS)
        assert stored.reference_mm == expected


async def test_the_reference_is_the_range_the_anchor_itself_measured(
    cell, builder, hub, autojump_clock
):
    cell.error[201, 202] = 0.02
    cell.share[201, 203] = 0.3

    survey = await measure(cell, builder, hub)
    assert _codes(survey["gate"]["warnings"]) == [("low-share", 203, 201)]
    answer = await commit(cell, builder, hub, acknowledgeWarnings=True)

    assert answer["result"] == "committed"
    # not the pair's mean, nor the fitted distance
    assert cell.stored(202).reference_mm == 4020
    # too few valid ranges to its initiator: no reference
    assert cell.stored(203).reference_mm == 0
    assert cell.stored(204).reference_mm == _range_mm(201, 204)


async def test_the_reference_is_unknown_without_the_stored_initiator(
    cell, builder, hub, autojump_clock
):
    await cell.asleep(201)
    await measure(cell, builder, hub, anchors=[202, 203, 204])
    answer = await commit(cell, builder, hub, reporting=[202, 203, 204])

    assert answer["result"] == "committed"
    assert [cell.stored(system_id).reference_mm for system_id in IDS[1:]] == [0, 0, 0]
    assert cell.stored(201) is None


async def test_warnings_need_the_operators_acknowledgement(
    cell, builder, hub, autojump_clock
):
    cell.error[201, 202] = 0.05
    survey = await measure(cell, builder, hub)
    assert _codes(survey["gate"]["warnings"]) == [("asymmetry", 201, 202)]

    for body in ({}, {"acknowledgeWarnings": "yes"}):
        answer = await request(cell, builder, hub, "commit", **body)
        assert (answer["result"], answer["reason"]) == ("warnings", None)
        assert answer["survey"]["state"] == "ready"
        assert answer["survey"]["commit"] is None
    assert cell.locations == []

    answer = await commit(cell, builder, hub, acknowledgeWarnings=True)
    assert answer["result"] == "committed"


async def test_a_block_cannot_be_acknowledged(cell, builder, hub, autojump_clock):
    cell.share[201, 203] = cell.share[203, 201] = 0.1
    survey = await measure(cell, builder, hub)
    assert ("pair-missing", 201, 203) in _codes(survey["gate"]["blocked"])

    answer = await request(cell, builder, hub, "commit", acknowledgeWarnings=True)

    assert (answer["result"], answer["reason"]) == ("blocked", None)
    assert answer["survey"]["state"] == "ready"
    assert cell.locations == []


async def test_an_anchor_that_misses_the_write_is_brought_in_by_a_retry(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_silent.add(204)

    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == INCOMPLETE
    survey = answer["survey"]
    assert (survey["state"], survey["reason"]) == ("ready", INCOMPLETE[1])
    assert survey["commit"]["surveyId"] == 1
    assert survey["commit"]["running"] == [201, 202, 203]
    assert survey["commit"]["results"] == {
        "201": {"written": "changed"},
        "202": {"written": "changed"},
        "203": {"written": "changed"},
        "204": {"written": "failed", "detail": "RTLS anchor A3: no answer"},
    }
    # nothing is rolled back
    assert cell.stored(201).location.survey_id == 1
    assert cell.stored(204) is None

    first = dict(cell.locations)
    cell.locations.clear()
    cell.location_silent.clear()
    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == ("committed", None)
    assert answer["survey"]["commit"]["surveyId"] == 1
    assert answer["survey"]["commit"]["running"] == IDS
    assert dict(cell.locations) == first and len(first) == 4


def _while_the_commit_reads(monkeypatch, happen):
    """``happen`` runs when the commit has read the anchors and has not
    written yet."""
    plan_commit = survey_module.plan_commit

    async def planned(*args):
        plan = await plan_commit(*args)
        await happen()
        return plan

    monkeypatch.setattr(survey_module, "plan_commit", planned)


@pytest.mark.parametrize(
    "change, reason",
    [
        ("moved", "Survey again"),
        ("lost", "Survey again"),
        ("locked", "Unlock the anchors first"),
        ("authorized", "A show is authorized"),
    ],
)
async def test_what_changes_while_the_commit_reads_still_refuses_it(
    cell, builder, hub, autojump_clock, monkeypatch, change, reason
):
    await measure(cell, builder, hub)

    async def happen():
        if change == "moved":
            await cell.report([203], moved=1.0)
        elif change == "lost":
            cell.lose(203)
        elif change == "locked":
            await cell.report([203], lock=1.0)
        else:
            authorize_show(cell.ext)

    _while_the_commit_reads(monkeypatch, happen)
    answer = await commit(cell, builder, hub, reporting=())

    assert (answer["result"], answer["reason"]) == ("refused", reason)
    assert (answer["survey"]["state"], answer["survey"]["reason"]) == ("ready", None)
    assert answer["survey"]["commit"] is None
    assert cell.locations == []


async def test_a_commit_refused_before_its_writes_is_not_a_retry_later(
    cell, builder, hub, autojump_clock, monkeypatch
):
    await measure(cell, builder, hub)

    async def lock():
        await cell.report([203], lock=1.0)

    _while_the_commit_reads(monkeypatch, lock)
    assert (await commit(cell, builder, hub, reporting=()))["result"] == "refused"
    monkeypatch.undo()

    # nothing was written: an anchor the server loses now needs a new survey
    cell.lose(204)
    await cell.ext._process_datagram(
        cell.devices[204].heartbeat(), cell.devices[204].address, 0.0
    )
    await cell.report()
    assert await _commit_refusal(cell, builder, hub) == "Survey again"


async def test_a_write_to_an_anchor_that_just_dropped_fails(cell, autojump_clock):
    anchors = [Anchor(202, "A1", 2, True), Anchor(203, "A2", 3, True)]
    cell.lose(203)

    results = await write_locations(cell.ext, anchors, {202: "", 203: ""})

    assert results == {
        "202": {"written": "changed"},
        "203": {"written": "failed", "detail": "A2 is offline"},
    }


async def test_the_survey_id_is_not_given_without_every_anchors_stored_one(
    cell, builder, hub, autojump_clock
):
    # 204 holds survey 1 of another server; this one never read it and its
    # read gets no answer
    await cell.asleep(204)
    await measure(cell, builder, hub, anchors=[201, 202, 203])
    _store(cell, 204, 1)
    del cell.ext._protocol.devices[204].params[LOC_PARAM]
    cell.devices[204].respond_to_read = False

    answer = await commit(cell, builder, hub, reporting=())

    assert (answer["result"], answer["reason"]) == (
        "refused",
        "RTLS anchor A3: its stored survey could not be read",
    )
    assert answer["survey"]["state"] == "ready"
    assert cell.locations == []

    cell.devices[204].respond_to_read = True
    answer = await commit(cell, builder, hub)
    assert answer["result"] == "committed"
    assert answer["survey"]["commit"]["surveyId"] == 2


async def test_an_anchor_shown_to_have_no_position_parameter_holds_no_survey(
    cell, builder, hub, autojump_clock
):
    # an anchor image from before UWB_LOC: its complete list has no such name
    await cell.asleep(204)
    await measure(cell, builder, hub, anchors=[201, 202, 203])
    old = cell.ext._protocol.devices[204]
    del old.params[LOC_PARAM]
    old.param_count -= 1
    cell.devices[204].read_requests.clear()

    answer = await commit(cell, builder, hub)

    assert answer["result"] == "committed"
    assert LOC_PARAM not in cell.devices[204].read_requests


async def test_a_retry_does_not_write_over_a_newer_survey(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_silent.add(204)
    incomplete = await commit(cell, builder, hub)
    assert incomplete["survey"]["commit"]["surveyId"] == 1
    cell.location_silent.clear()
    cell.locations.clear()

    # another server commits survey 2 to the cell
    for system_id in IDS:
        _store(cell, system_id, 2)
    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == ("refused", "Survey again")
    assert answer["survey"] == incomplete["survey"]
    assert cell.locations == []
    assert {cell.stored(system_id).location.survey_id for system_id in IDS} == {2}

    # a new fit is a new survey: it takes the next ID
    sketch = {str(k): {"x": x, "y": y} for k, (x, y, _) in FLAT.items()}
    await request(cell, builder, hub, "fit", placement=sketch)
    answer = await commit(cell, builder, hub)
    assert answer["result"] == "committed"
    assert answer["survey"]["commit"]["surveyId"] == 3


async def test_a_retry_needs_the_stored_value_of_every_anchor_of_the_group(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_silent.add(204)
    await commit(cell, builder, hub)
    cell.locations.clear()

    cell.devices[202].respond_to_read = False
    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == (
        "refused",
        "RTLS anchor A1: its stored survey could not be read",
    )
    assert cell.locations == []


async def test_a_retry_brings_in_an_anchor_the_server_lost_and_found_again(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_silent.add(204)
    await commit(cell, builder, hub)
    first = dict(cell.locations)
    cell.locations.clear()
    cell.location_silent.clear()

    cell.lose(204)
    answer = await request(cell, builder, hub, "commit")
    assert (answer["result"], answer["reason"]) == (
        "refused",
        "RTLS anchor A3 is offline",
    )
    await cell.ext._process_datagram(
        cell.devices[204].heartbeat(), cell.devices[204].address, 0.0
    )
    await cell.report()

    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == ("committed", None)
    assert answer["survey"]["commit"]["surveyId"] == 1
    assert dict(cell.locations) == first


async def test_a_retry_is_refused_once_an_anchor_moved(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_silent.add(204)
    await commit(cell, builder, hub)
    cell.locations.clear()
    cell.location_silent.clear()

    await cell.report([202], moved=1.0)

    assert await _commit_refusal(cell, builder, hub) == "Survey again"


async def test_a_position_an_anchor_cannot_store_is_not_committed(
    cell, builder, hub, autojump_clock
):
    cell.positions = {k: (x * 3000, y * 3000, z) for k, (x, y, z) in FLAT.items()}
    survey = await measure(cell, builder, hub)
    assert survey["fit"]["anchors"]["203"]["x"] == pytest.approx(12600, abs=0.1)
    assert await _commit_refusal(cell, builder, hub) == (
        "A fitted position is farther than an anchor can store"
    )


async def test_an_unreadable_stored_position_is_no_position(
    cell, builder, hub, autojump_clock
):
    cell.devices[202].params[LOC_PARAM] = (b"not a location", PARAM_TYPE_CUSTOM)
    await cell.ext.get_param(202, LOC_PARAM)

    survey = await measure(cell, builder, hub)
    assert "movedM" not in survey["fit"]["anchors"]["202"]
    answer = await commit(cell, builder, hub)
    assert answer["survey"]["commit"]["surveyId"] == 1


async def test_a_write_without_an_ack_is_not_waited_for_longer(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    # 204 takes the write but its ack is lost: it reports the new ID
    cell.location_silent.add(204)
    cell.stats[204] = {"sid": 1.0}
    started = trio.current_time()

    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == INCOMPLETE
    assert answer["survey"]["commit"]["running"] == IDS
    # the write's own timeout and its wait for a late ack, nothing after
    assert trio.current_time() - started == pytest.approx(6.0, abs=0.2)


async def test_a_refused_write_leaves_the_commit_incomplete(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_refusing.add(203)

    answer = await commit(cell, builder, hub)

    assert (answer["result"], answer["reason"]) == INCOMPLETE
    assert answer["survey"]["commit"]["results"]["203"] == {
        "written": "failed",
        "detail": "RTLS anchor A2 refused its position",
    }


async def test_an_anchor_that_does_not_report_the_new_id_is_not_done(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    started = trio.current_time()

    answer = await commit(cell, builder, hub, reporting=[201, 202, 203])

    assert (answer["result"], answer["reason"]) == INCOMPLETE
    assert trio.current_time() - started == pytest.approx(10.0, abs=0.2)
    results = answer["survey"]["commit"]["results"]
    assert {entry["written"] for entry in results.values()} == {"changed"}
    assert answer["survey"]["commit"]["running"] == [201, 202, 203]

    answer = await commit(cell, builder, hub)
    assert answer["result"] == "committed"


async def test_a_running_session_is_not_taken_for_the_new_id(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    # 204 restarts its ranging: no session, then the old survey, then the new
    cell.stats[204] = {"sid": -1.0}
    answer = await commit(cell, builder, hub)
    assert answer["result"] == "incomplete"
    assert answer["survey"]["commit"]["running"] == [201, 202, 203]

    cell.stats[204] = {}
    answer = await commit(cell, builder, hub)
    assert answer["result"] == "committed"


async def test_the_survey_id_continues_what_the_anchors_store(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    # written by another server since this one last read the anchor
    _store(cell, 202, 0x0102)

    answer = await commit(cell, builder, hub)

    assert answer["survey"]["commit"]["surveyId"] == 0x0103
    assert {cell.stored(system_id).location.survey_id for system_id in IDS} == {0x0103}


async def test_the_survey_id_wraps_past_the_ids_no_anchor_sends(
    cell, builder, hub, autojump_clock
):
    _store(cell, 203, 0xFFFE)
    await measure(cell, builder, hub)
    answer = await commit(cell, builder, hub)
    assert answer["survey"]["commit"]["surveyId"] == 1


async def test_an_offline_anchor_counts_with_the_id_it_last_stored(
    cell, builder, hub, autojump_clock
):
    _store(cell, 204, 9)
    await cell.ext.get_param(204, LOC_PARAM)
    cell.lose(204)
    await measure(cell, builder, hub, anchors=[201, 202, 203])

    answer = await commit(cell, builder, hub, reporting=[201, 202, 203])

    assert answer["result"] == "committed"
    assert answer["survey"]["commit"]["surveyId"] == 10


async def test_a_second_survey_says_how_far_each_anchor_moved(
    cell, builder, hub, autojump_clock
):
    survey = await measure(cell, builder, hub)
    assert "movedM" not in survey["fit"]["anchors"]["202"]
    await commit(cell, builder, hub)

    cell.positions = FLAT | {202: (4.5, 0.0, 0.0)}
    survey = await measure(cell, builder, hub)

    moved = {k: v["movedM"] for k, v in survey["fit"]["anchors"].items()}
    assert moved == pytest.approx({"201": 0, "202": 0.5, "203": 0, "204": 0}, abs=1e-3)
    assert (await commit(cell, builder, hub))["survey"]["commit"]["surveyId"] == 2


async def test_fit_refits_the_held_measurement_without_ranging(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    await trio.sleep(1)
    await trio.sleep(1)
    commands, pushed = len(cell.commands), len(cell.broadcasts)
    frame = {
        "origin": 202,
        "xAxis": 201,
        "heightGroups": [IDS],
        "upper": [],
        "yPositive": 204,
    }
    placement = {
        "201": {"x": 4, "y": 0},
        "202": {"x": 0, "y": 0},
        "203": {"x": 0, "y": 3},
        "204": {"x": 4, "y": 3, "z": 0},
    }

    answer = await request(cell, builder, hub, "fit", frame=frame, placement=placement)

    await trio.sleep(0.01)
    assert (answer["action"], answer["result"], answer["reason"]) == (
        "fit",
        "fitted",
        None,
    )
    survey = answer["survey"]
    assert (survey["state"], survey["frame"]) == ("ready", frame)
    assert survey["placement"] == {
        "201": {"x": 4.0, "y": 0.0},
        "202": {"x": 0.0, "y": 0.0},
        "203": {"x": 0.0, "y": 3.0},
        "204": {"x": 4.0, "y": 3.0, "z": 0.0},
    }
    anchors = survey["fit"]["anchors"]
    assert (anchors["202"]["x"], anchors["202"]["y"]) == (0.0, 0.0)
    assert (anchors["201"]["x"], anchors["201"]["y"]) == pytest.approx((4.0, 0.0))
    assert anchors["204"]["y"] == pytest.approx(2.8, abs=1e-3)
    assert len(cell.commands) == commands
    assert cell.broadcasts[pushed:] == [survey]

    # without a frame the held one is used; without a placement there is none
    answer = await request(cell, builder, hub, "fit")
    assert answer["result"] == "fitted"
    assert answer["survey"]["frame"] == frame
    assert answer["survey"]["placement"] is None


async def test_a_fit_that_cannot_run_is_refused_and_the_held_one_stays(
    cell, builder, hub, autojump_clock
):
    cell.failing.add(203)
    placement = {str(k): {"x": x, "y": y} for k, (x, y, _) in FLAT.items()}
    held = await measure(cell, builder, hub, placement=placement)

    answer = await request(cell, builder, hub, "fit")
    assert (answer["result"], answer["reason"]) == (
        "refused",
        "Place the anchors roughly first",
    )
    assert answer["survey"] == held

    answer = await request(cell, builder, hub, "fit", frame={"origin": 201, "xAxis": 9})
    assert (answer["result"], answer["reason"]) == (
        "refused",
        "The frame names anchor 9, which is not in the survey",
    )
    assert answer["survey"] == held


async def test_fit_needs_a_measurement(cell, builder, hub, autojump_clock):
    answer = await request(cell, builder, hub, "fit", frame=TWO_TIERS)
    assert (answer["result"], answer["reason"]) == (
        "refused",
        "There is no measurement to fit",
    )

    cell.collect_s = 1.2
    await request(cell, builder, hub, "start", anchors=IDS, frame=TWO_TIERS)
    for action in ("fit", "commit"):
        answer = await request(cell, builder, hub, action)
        assert (answer["result"], answer["reason"]) == (
            "refused",
            "A survey is running",
        )


async def test_a_refit_after_a_missed_commit_is_a_new_survey(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    cell.location_silent.add(204)
    await commit(cell, builder, hub)
    cell.location_silent.clear()

    answer = await request(cell, builder, hub, "fit")
    assert (answer["survey"]["commit"], answer["survey"]["reason"]) == (None, None)
    answer = await commit(cell, builder, hub)

    # three anchors hold survey 1 with the positions of the fit before
    assert answer["result"] == "committed"
    assert answer["survey"]["commit"]["surveyId"] == 2


async def test_a_missing_stored_initiator_is_read_before_the_fit(
    cell, builder, hub, autojump_clock
):
    devices = cell.ext._protocol.devices
    for system_id in IDS:
        del devices[system_id].params["UWB_AN0_MAC"]
        del devices[system_id].params[LOC_PARAM]

    await measure(cell, builder, hub)
    await commit(cell, builder, hub)

    assert cell.stored(202).reference_mm == _range_mm(201, 202)


async def test_a_stored_initiator_that_cannot_be_read_costs_only_the_reference(
    cell, builder, hub, autojump_clock
):
    del cell.ext._protocol.devices[203].params["UWB_AN0_MAC"]
    del cell.devices[203].params["UWB_AN0_MAC"]

    await measure(cell, builder, hub)
    # the commit asks once more, and waits two seconds for the answer
    answer = await commit(cell, builder, hub, after=3)

    assert answer["result"] == "committed"
    assert cell.stored(203).reference_mm == 0
    assert cell.stored(202).reference_mm == _range_mm(201, 202)


async def _commit_refusal(cell, builder, hub):
    answer = await request(cell, builder, hub, "commit", acknowledgeWarnings=True)
    assert answer["result"] == "refused"
    assert cell.locations == []
    return answer["reason"]


async def test_commit_needs_a_fit(cell, builder, hub, autojump_clock):
    assert await _commit_refusal(cell, builder, hub) == "There is no fit to commit"
    await measure(cell, builder, hub)
    await commit(cell, builder, hub)
    cell.locations.clear()
    assert await _commit_refusal(cell, builder, hub) == "There is no fit to commit"


async def test_commit_is_refused_when_the_cell_changed_since_the_survey(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    await cell.report([203], moved=1.0)
    assert await _commit_refusal(cell, builder, hub) == "Survey again"

    await measure(cell, builder, hub)
    cell.lose(203)
    assert await _commit_refusal(cell, builder, hub) == "Survey again"


async def test_an_anchor_that_came_back_since_the_survey_needs_a_new_one(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await survey_cell(dialect, nursery)
    await measure(cell, builder, hub)
    cell.lose(203)
    await cell.ext._process_datagram(
        cell.devices[203].heartbeat(), cell.devices[203].address, 0.0
    )
    await cell.report()
    assert 203 in cell.ext._protocol.devices
    assert await _commit_refusal(cell, builder, hub) == "Survey again"


async def test_an_anchor_marked_moved_before_the_survey_can_be_committed(
    cell, builder, hub, autojump_clock
):
    cell.stats[203] = {"moved": 1.0}
    await cell.report()
    await measure(cell, builder, hub)
    assert (await commit(cell, builder, hub))["result"] == "committed"


async def test_commit_is_refused_while_the_cell_may_not_be_written(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)

    cell.ext._cell_applying = True
    assert await _commit_refusal(cell, builder, hub) == "An apply is running"
    cell.ext._cell_applying = False
    authorize_show(cell.ext)
    assert await _commit_refusal(cell, builder, hub) == "A show is authorized"
    await cell.report([202], lock=1.0)
    assert await _commit_refusal(cell, builder, hub) == "Unlock the anchors first"


async def test_commit_is_refused_by_a_lock_of_a_device_of_unknown_role(
    cell, builder, hub, autojump_clock
):
    await cell.asleep(204)
    await measure(cell, builder, hub, anchors=[201, 202, 203])
    del cell.ext._protocol.devices[204].params["UWB_ROLE"]
    await cell.report([204], lock=1.0)
    assert await _commit_refusal(cell, builder, hub) == "Unlock the anchors first"


async def test_a_commit_runs_alone_and_tells_its_clients(
    cell, builder, hub, nursery, autojump_clock
):
    await measure(cell, builder, hub)
    await trio.sleep(1)
    await trio.sleep(1)
    pushed = len(cell.broadcasts)
    cell.location_silent.add(204)

    nursery.start_soon(request, cell, builder, hub, "commit")
    await trio.sleep(0.1)

    assert cell.ext._survey.state == "committing"
    for action, reason in (
        ("start", "A survey is running"),
        ("commit", "A survey is running"),
        ("cancel", "A commit cannot be cancelled"),
    ):
        body = {"anchors": IDS, "frame": TWO_TIERS} if action == "start" else {}
        answer = await request(cell, builder, hub, action, **body)
        assert (answer["result"], answer["reason"]) == ("refused", reason)
    assert (await _lock(cell.ext, builder, hub))["result"] == "refused"
    await cell.report()
    await until(cell, "ready")
    await trio.sleep(0.5)

    states = [(s["state"], s["commit"] is not None) for s in cell.broadcasts[pushed:]]
    assert states == [("committing", False), ("ready", True)]


async def test_each_push_of_a_commit_says_what_was_known_then(
    cell, builder, hub, nursery, autojump_clock
):
    await measure(cell, builder, hub)
    await trio.sleep(1)
    pushed = len(cell.broadcasts)

    nursery.start_soon(request, cell, builder, hub, "commit")
    await trio.sleep(0.1)
    await cell.report([201, 202])
    await trio.sleep(1)
    await cell.report()
    await until(cell, "committed")
    await trio.sleep(0.5)

    running = [s["commit"] and s["commit"]["running"] for s in cell.broadcasts[pushed:]]
    assert running == [None, [201, 202], IDS]


def test_the_survey_id_stat_reads_null_without_a_survey():
    assert _stats_json(201, {"sid": 12.0}, "anchor-responder")["surveyId"] == 12
    assert _stats_json(201, {"sid": 0.0}, "anchor-responder")["surveyId"] is None
    assert _stats_json(201, {"sid": -1.0}, "anchor-responder")["surveyId"] is None
    assert "surveyId" not in _stats_json(201, {}, "anchor-responder")
