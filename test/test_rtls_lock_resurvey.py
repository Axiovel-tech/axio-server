"""The lock's resurvey check (X-RTLS-LOCK ``resurvey``, rtls/lock_resurvey.py
and Survey.check): it measures the cell, warns of anchors off their stored
positions or of a check that could not run, and holds or writes nothing."""

import pytest
import trio
from anchor_lock_support import _lock
from rtlslink.dialect import load_dialect
from survey_support import (
    FLAT,
    TIERS,
    commit,
    fly_drone,
    held,
    measure,
    request,
    survey_cell,
)

from flockwave.server.ext.rtls.lock_resurvey import stored_frame
from flockwave.server.ext.rtls.survey_fit import Frame
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


async def _surveyed(dialect, nursery, builder, hub, positions=FLAT):
    """A cell whose anchors store a committed survey of ``positions``."""
    cell = await survey_cell(dialect, nursery, positions)
    await measure(cell, builder, hub)
    assert (await commit(cell, builder, hub))["result"] == "committed"
    await cell.report()
    return cell


@pytest.fixture
async def cell(dialect, nursery, builder, hub, autojump_clock):
    return await _surveyed(dialect, nursery, builder, hub)


def _codes(body):
    return [(w["code"], w["device"], w["detail"]) for w in body["warnings"]]


def _steps_sent(cell):
    return len([value for value in cell.steps() if value != "0000"])


def test_the_frame_is_read_from_the_stored_positions():
    assert stored_frame(FLAT) == Frame(201, 202, ((201, 202, 203, 204),))
    stored = {k: (n, e, d + 0.02 * (k % 2)) for k, (n, e, d) in TIERS.items()}
    stored[205] = (0.0, 0.0, -2.4)
    stored[201] = (0.0005, 0.0, 0.0)
    assert stored_frame({k: v for k, v in stored.items() if k != 205}) == Frame(
        201,
        202,
        ((206, 207, 208), (201, 202, 203, 204)),
        upper=(206, 207, 208),
    )
    assert stored_frame({202: (4.0, 0.0, 0.0), 203: (4.2, 3.1, 0.0)}) is None
    assert stored_frame({201: (0.0, 0.0, 0.0), 203: (4.2, 3.1, 0.0)}) is None


async def test_a_cell_where_it_was_surveyed_locks_at_once(cell, builder, hub):
    before = _steps_sent(cell)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert (body["result"], body["warnings"]) == ("locked", [])
    assert _steps_sent(cell) == before + len(FLAT)


@pytest.mark.parametrize(
    "moved, code, detail",
    [
        (0.12, "resurvey-moved", "RTLS anchor A2 is 12 cm off its surveyed position"),
        (
            0.3,
            "resurvey-moved-danger",
            "RTLS anchor A2 moved: 30 cm off its surveyed position",
        ),
    ],
)
async def test_an_anchor_carried_sideways_warns_at_the_two_bounds(
    cell, builder, hub, moved, code, detail
):
    north, east, down = FLAT[203]
    cell.positions[203] = (north, east + moved, down)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert body["result"] == "warnings"
    assert _codes(body) == [(code, 203, detail)]
    # the operator locks anyway; the check is not run again
    steps = _steps_sent(cell)
    body = await _lock(cell.ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"
    assert _steps_sent(cell) == steps


async def test_without_the_option_nothing_is_measured(cell, builder, hub):
    cell.positions[203] = (4.2, 3.5, 0.0)
    before = _steps_sent(cell)

    assert (await _lock(cell.ext, builder, hub))["result"] == "locked"
    assert _steps_sent(cell) == before


async def test_a_check_that_cannot_run_is_a_warning(cell, builder, hub):
    fly_drone(cell.ext, armed=True)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert body["result"] == "warnings"
    assert _codes(body) == [
        ("resurvey-failed", None, "Resurvey check failed: Drone 17 is armed")
    ]
    body = await _lock(cell.ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"


async def test_a_measurement_that_fails_is_a_warning(cell, builder, hub):
    cell.refusing.add(203)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert body["result"] == "warnings"
    assert _codes(body) == [
        (
            "resurvey-failed",
            None,
            "Resurvey check failed: RTLS anchor A2 refused the survey step: its "
            "radio is not running, or the step does not fit its slots",
        )
    ]


async def test_an_anchor_storing_no_frame_fails_the_check(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await survey_cell(dialect, nursery)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert ("resurvey-failed", None) in [
        (w["code"], w["device"]) for w in body["warnings"]
    ]
    assert body["warnings"][-1]["detail"] == (
        "Resurvey check failed: no anchor stores the origin and +x of a survey"
    )


async def test_the_held_survey_is_left_as_it_was(cell, builder, hub):
    # a fit the operator has not committed
    await measure(cell, builder, hub)
    await trio.sleep(1)
    survey = cell.ext._survey
    before, revision = survey.json, survey.revision
    pushed = len(cell.broadcasts)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert body["result"] == "locked"
    assert (survey.json, survey.revision) == (before, revision)
    assert len(cell.broadcasts) == pushed
    assert held(survey.json)["state"] == "ready"


async def test_while_the_check_measures_a_survey_and_a_lock_are_refused(
    cell, builder, hub
):
    cell.collect_s = 1.0
    answers = {}

    async def lock():
        answers["lock"] = await _lock(cell.ext, builder, hub, resurvey=True)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(lock)
        await trio.sleep(0.5)
        assert cell.ext._survey.involves(203)
        start = await request(cell, builder, hub, "start", anchors=list(FLAT))
        assert (start["result"], start["reason"]) == (
            "refused",
            "A survey is running",
        )
        second = await _lock(cell.ext, builder, hub, resurvey=True)
        assert (second["result"], second["reason"]) == (
            "refused",
            "A survey is running",
        )
        assert cell.ext._survey.json["state"] == "committed"

    assert answers["lock"]["result"] == "locked"
    assert not cell.ext._survey.involves(203)


async def test_resurvey_must_be_a_flag(cell, builder, hub):
    body = await _lock(cell.ext, builder, hub, resurvey="yes")

    assert body["reason"] == "Invalid resurvey: expected true or false"


async def test_an_upper_anchor_of_the_standard_cell_is_checked_in_its_tier(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await _surveyed(dialect, nursery, builder, hub, TIERS)
    north, east, down = TIERS[206]
    cell.positions[206] = (north - 0.1, east, down)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [
        ("resurvey-moved", 206, "RTLS anchor A5 is 10 cm off its surveyed position")
    ]
