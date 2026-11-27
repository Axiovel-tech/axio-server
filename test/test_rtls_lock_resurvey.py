"""The lock's resurvey check (X-RTLS-LOCK ``resurvey``, rtls/lock_resurvey.py
and Survey.check): it measures the cell, warns of anchors off their stored
positions or of a check that could not run, and holds or writes nothing."""

import pytest
import trio
from anchor_lock_support import _lock
from rtls_support import discover
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


FLAT_HEIGHTS = ("resurvey-unchecked", None, "Heights not checked: flat cell")


def _surveyed_moved(cell, system_id, north=0.0, east=0.0, down=0.0):
    n, e, d = cell.positions[system_id]
    cell.positions[system_id] = (n + north, e + east, d + down)


def test_the_frame_is_read_from_the_stored_positions():
    assert stored_frame(FLAT) == (Frame(201, 202, ((201, 202, 203, 204),)), None)
    stored = {k: (n, e, d + 0.02 * (k % 2)) for k, (n, e, d) in TIERS.items()}
    stored[201] = (0.0005, 0.0, 0.0)
    tiers = Frame(
        201,
        202,
        ((205, 206, 207, 208), (201, 202, 203, 204)),
        upper=(205, 206, 207, 208),
    )
    assert stored_frame(stored) == (tiers, None)
    # the bench cell: an upper anchor straight above the origin, listed first
    assert stored_frame({205: (0.0, 0.0, -2.4)} | stored) == (tiers, None)
    assert stored_frame({202: (4.0, 0.0, 0.0), 203: (4.2, 3.1, 0.0)}) == (
        None,
        "no anchor stores the origin of a survey",
    )
    assert stored_frame({201: (0.0, 0.0, 0.0), 203: (4.2, 3.1, 0.0)}) == (
        None,
        "no anchor stores the +x of a survey",
    )
    assert stored_frame(FLAT | {205: (0.0, 0.001, 0.1)}) == (
        None,
        "two anchors store the origin of a survey",
    )


async def test_a_cell_where_it_was_surveyed_locks_at_once(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await _surveyed(dialect, nursery, builder, hub, TIERS)
    before = _steps_sent(cell)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert (body["result"], body["warnings"]) == ("locked", [])
    assert body["resurvey"] == {"ran": True}
    assert _steps_sent(cell) == before + len(TIERS)


async def test_a_flat_cell_says_it_cannot_check_heights(cell, builder, hub):
    _surveyed_moved(cell, 203, down=-0.3)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [FLAT_HEIGHTS]


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
    _surveyed_moved(cell, 203, east=moved)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert body["result"] == "warnings"
    assert _codes(body) == [FLAT_HEIGHTS, (code, 203, detail)]
    # the operator locks anyway; the check is not run again
    steps = _steps_sent(cell)
    body = await _lock(cell.ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"
    assert body["resurvey"] == {"ran": False}
    assert _steps_sent(cell) == steps


@pytest.mark.parametrize(
    "system_id, name, north, east",
    [(201, "A0", 0.15, 0.0), (202, "A1", 0.0, 0.15), (202, "A1", 0.15, 0.0)],
)
async def test_a_moved_origin_or_x_anchor_is_the_one_blamed(
    dialect, nursery, builder, hub, autojump_clock, system_id, name, north, east
):
    cell = await _surveyed(dialect, nursery, builder, hub, TIERS)
    _surveyed_moved(cell, system_id, north, east)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [
        (
            "resurvey-moved",
            system_id,
            f"RTLS anchor {name} is 15 cm off its surveyed position",
        )
    ]


async def test_too_many_moved_anchors_fail_the_check(cell, builder, hub):
    _surveyed_moved(cell, 203, east=0.5)
    _surveyed_moved(cell, 204, north=0.5)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [
        FLAT_HEIGHTS,
        ("resurvey-failed", None, "Resurvey check failed: too many anchors moved"),
    ]


async def test_a_poor_measurement_is_named_as_such(cell, builder, hub):
    cell.error[(201, 203)] = cell.error[(203, 201)] = 0.3

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    # a measurement this poor locates too few anchors to blame any of them
    codes = _codes(body)
    poor = "Resurvey measurement poor: fit rms "
    assert [c for c, _, d in codes if d.startswith(poor)] == ["resurvey-failed"]
    assert not [code for code, _, _ in codes if code.startswith("resurvey-moved")]


async def test_without_the_option_nothing_is_measured(cell, builder, hub):
    cell.positions[203] = (4.2, 3.5, 0.0)
    before = _steps_sent(cell)

    body = await _lock(cell.ext, builder, hub)
    assert (body["result"], body["resurvey"]) == ("locked", {"ran": False})
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


async def test_a_check_waits_for_no_anchor_being_updated(cell, builder, hub):
    cell.ext._ota_jobs[203] = {"id": 203, "status": "running", "progress": 0.5}

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [
        (
            "resurvey-failed",
            None,
            "Resurvey check failed: RTLS anchor A2 is being updated",
        )
    ]


async def test_an_update_cannot_start_on_an_anchor_a_check_took_meanwhile(
    cell, builder, hub, tmp_path
):
    image = tmp_path / "anchor.bin"
    image.write_bytes(b"image")
    cell.collect_s = 1.0
    checking = trio.Event()

    async def verify_species(system_id, species):
        # the role read takes a while: the lock starts its check meanwhile
        checking.set()
        await trio.sleep(0.5)

    cell.ext.verify_species = verify_species
    async with trio.open_nursery() as nursery:

        async def lock():
            await checking.wait()
            await _lock(cell.ext, builder, hub, resurvey=True)

        nursery.start_soon(lock)
        with pytest.raises(RuntimeError, match="A resurvey check is running"):
            await cell.ext.start_ota(203, str(image), role="anchor")
    assert not cell.ext.ota_running(203)


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
        "Resurvey check failed: no anchor stores the origin of a survey"
    )


async def test_the_held_survey_is_left_as_it_was(cell, builder, hub):
    # a fit the operator has not committed
    await measure(cell, builder, hub)
    await trio.sleep(1)
    survey = cell.ext._survey
    before, revision = survey.json, survey.revision
    pushed = len(cell.broadcasts)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [FLAT_HEIGHTS]
    assert (survey.json, survey.revision) == (before, revision)
    assert len(cell.broadcasts) == pushed
    assert held(survey.json)["state"] == "ready"


async def test_while_the_check_measures_a_survey_and_a_lock_are_refused(
    cell, builder, hub
):
    cell.collect_s = 1.0
    answers = {}

    async def lock():
        answers["lock"] = await _lock(
            cell.ext, builder, hub, resurvey=True, acknowledgeWarnings=True
        )

    async with trio.open_nursery() as nursery:
        nursery.start_soon(lock)
        await trio.sleep(0.5)
        assert cell.ext._survey.involves(203)
        refused = ("refused", "A resurvey check is running")
        start = await request(cell, builder, hub, "start", anchors=list(FLAT))
        assert (start["result"], start["reason"]) == refused
        revision = cell.ext._survey.revision
        body = await request(cell, builder, hub, "commit", revision=revision)
        assert (body["result"], body["reason"]) == refused
        second = await _lock(cell.ext, builder, hub, resurvey=True)
        assert (second["result"], second["reason"]) == refused
        assert second["resurvey"] == {"ran": False}
        assert cell.ext._survey.json["state"] == "committed"

    assert answers["lock"]["result"] == "locked"
    assert not cell.ext._survey.involves(203)


async def test_a_cancelled_check_still_returns_the_anchors(cell, builder, hub):
    cell.collect_s = 1.0
    ended = len(cell.sent(203, "0000"))

    with trio.move_on_after(0.5):
        await _lock(cell.ext, builder, hub, resurvey=True)

    assert len(cell.sent(203, "0000")) == ended + 1
    assert cell.ext._survey.busy is None


async def test_a_range_check_warning_from_before_the_check_is_kept(cell, builder, hub):
    await cell.report([204], rdev=0.25)
    cell.collect_s = 1.0
    answers = {}

    async def lock():
        answers["lock"] = await _lock(cell.ext, builder, hub, resurvey=True)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(lock)
        await trio.sleep(0.5)
        # in a survey step an anchor reports no range check
        await cell.report()

    assert _codes(answers["lock"]) == [
        (
            "range-deviation-danger",
            204,
            "RTLS anchor A3 moved: 25 cm off its surveyed range",
        ),
        FLAT_HEIGHTS,
    ]


async def test_one_anchor_warns_once_of_the_graver_finding(cell, builder, hub):
    await cell.report([203], rdev=0.1)
    _surveyed_moved(cell, 203, east=0.3)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert [(code, device) for code, device, _ in _codes(body)] == [
        ("resurvey-unchecked", None),
        ("resurvey-moved-danger", 203),
    ]


async def test_an_anchor_the_check_did_not_measure_is_named(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await _surveyed(dialect, nursery, builder, hub, TIERS)
    cell.lose(208)
    cell.collect_s = 1.0
    answers = {}

    async def lock():
        answers["lock"] = await _lock(cell.ext, builder, hub, resurvey=True)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(lock)
        await trio.sleep(0.5)
        # one wakes, one restarts while the others are measured
        await discover(cell.ext, cell.devices[208])
        await cell.report([208])
        await cell.report([207], boots=9.0)

    assert _codes(answers["lock"]) == [
        (
            "resurvey-unchecked",
            207,
            "RTLS anchor A6 not checked: restarted during the check",
        ),
        ("resurvey-unchecked", 208, "RTLS anchor A7 not checked"),
    ]


async def test_resurvey_must_be_a_flag(cell, builder, hub):
    body = await _lock(cell.ext, builder, hub, resurvey="yes")

    assert body["reason"] == "Invalid resurvey: expected true or false"


async def test_an_upper_anchor_of_the_standard_cell_is_checked_in_its_tier(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await _surveyed(dialect, nursery, builder, hub, TIERS)
    _surveyed_moved(cell, 206, north=-0.1)

    body = await _lock(cell.ext, builder, hub, resurvey=True)

    assert _codes(body) == [
        ("resurvey-moved", 206, "RTLS anchor A5 is 10 cm off its surveyed position")
    ]
