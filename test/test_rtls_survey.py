"""The survey run (X-RTLS-SURVEY, rtls/survey.py): what starts it, its steps
on the anchors, what ends it early, and what it tells its clients."""

import math
import time

import pytest
import trio
from anchor_lock_support import CODE_50, HEALTHY_A1, _lock
from cell_settings_support import _request as cell_request
from cell_settings_support import _states as cell_states
from rtls_support import make_message
from rtlslink.dialect import load_dialect
from rtlslink.survey import SurveyAnchor, encode_step
from survey_support import (
    ASLEEP,
    FLAT,
    TIERS,
    TWO_TIERS,
    authorize_show,
    commit,
    held,
    measure,
    request,
    survey_cell,
    until,
)

from flockwave.server.ext.rtls import survey as survey_module
from flockwave.server.ext.rtls.survey_step import StepProblem, end_survey, run_step
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

IDS = list(FLAT)
FRAME = {"origin": 201, "xAxis": 202}
IDLE = {
    "state": "idle",
    "reason": None,
    "anchors": [],
    "step": 0,
    "steps": 0,
    "initiator": None,
    "pairs": [],
    "fit": None,
    "gate": {"blocked": [], "warnings": []},
    "commit": None,
    "frame": None,
    "placement": None,
}
#: 100 rounds of a 20 ms block: the wait for a step's reports, in seconds
STEP_LIMIT = 7.5
HOLD = 8.5


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


def _start(cell, builder, hub, **body):
    body = {"anchors": IDS, "frame": TWO_TIERS} | body
    return request(cell, builder, hub, "start", **body)


def _initiators(cell):
    """The initiator MAC of each step commanded."""
    return [int(value[10:14], 16) for value in cell.steps() if value != "0000"]


async def test_an_idle_server_reports_an_empty_survey(
    cell, builder, hub, autojump_clock
):
    body = await request(cell, builder, hub)
    assert body == {
        "type": "X-RTLS-SURVEY",
        "action": "status",
        "result": "status",
        "reason": None,
        "survey": IDLE | {"revision": 0},
    }


async def test_a_run_steps_every_anchor_returns_them_and_fits(
    cell, builder, hub, autojump_clock, monkeypatch
):
    monkeypatch.setattr(survey_module.random, "randint", lambda low, high: 0xFFFE)
    cell.collect_s = 1.2

    answer = await _start(cell, builder, hub)

    assert (answer["action"], answer["result"], answer["reason"]) == (
        "start",
        "started",
        None,
    )
    assert held(answer["survey"]) | {"frame": None} == IDLE | {
        "state": "measuring",
        "anchors": IDS,
        "step": 1,
        "steps": 4,
        "initiator": 201,
    }
    await until(cell, "returning")
    await cell.report()
    await until(cell, "ready")
    await trio.sleep(0.5)

    # step IDs count up from a random one and skip 0; the first step is the
    # session the anchors store, the last command ends the survey
    assert cell.steps() == [
        "fffe006455" + "0001" + "000200030004",
        "ffff006455" + "0002" + "000100030004",
        "0001006455" + "0003" + "000100020004",
        "0002006455" + "0004" + "000100020003",
        "0000",
    ]
    assert len(cell.commands) == 5 * len(IDS)
    assert [(s["state"], s["step"], s["initiator"]) for s in cell.broadcasts] == [
        ("measuring", 1, 201),
        ("measuring", 2, 202),
        ("measuring", 3, 203),
        ("measuring", 4, 204),
        ("returning", 4, None),
        ("ready", 4, None),
    ]
    survey = cell.broadcasts[-1]
    assert survey["frame"] == {
        "origin": 201,
        "xAxis": 202,
        "heightGroups": [IDS],
        "upper": [],
        "yPositive": 203,
    }
    assert survey["placement"] is None
    assert survey["gate"] == {"blocked": [], "warnings": []}
    assert [(pair["a"], pair["b"]) for pair in survey["pairs"]] == [
        (201, 202),
        (201, 203),
        (201, 204),
        (202, 203),
        (202, 204),
        (203, 204),
    ]
    pair = survey["pairs"][1]
    assert (
        pair["ab"]
        == pair["ba"]
        == {
            "rangeM": round(math.dist(FLAT[201], FLAT[203]), 4),
            "madM": 0.004,
            "n": 98,
            "share": 0.98,
        }
    )
    # a range is what a responder measured: twelve directions, no other
    assert len(cell.ext._survey._run.measurements) == 12
    fit = survey["fit"]
    assert fit["planar"] is True
    for system_id, truth in FLAT.items():
        fitted = fit["anchors"][str(system_id)]
        assert (fitted["x"], fitted["y"], fitted["z"]) == pytest.approx(truth, abs=1e-3)


async def test_the_steps_follow_the_stored_slots_not_the_request(
    cell, builder, hub, autojump_clock
):
    survey = await measure(cell, builder, hub, anchors=[204, 203, 202, 201])

    assert _initiators(cell) == [1, 2, 3, 4]
    assert survey["anchors"] == [204, 203, 202, 201]
    assert survey["frame"]["origin"] == 204


async def test_a_group_that_is_not_the_whole_cell_keeps_its_order(
    cell, builder, hub, autojump_clock
):
    await cell.asleep(204)
    await measure(cell, builder, hub, anchors=[203, 201, 202])

    assert cell.steps()[0].endswith("0003" + "00010002")
    assert _initiators(cell) == [3, 1, 2]


async def test_no_anchors_named_means_every_online_anchor_in_slot_order(
    dialect, nursery, builder, hub, autojump_clock
):
    # 204 is the stored initiator and slot 0, 202 slot 2, 201 slot 3
    cell = await survey_cell(dialect, nursery, dict(reversed(FLAT.items())))
    cell.lose(203)
    answer = await request(cell, builder, hub, "start", frame=TWO_TIERS)
    assert answer["survey"]["anchors"] == [204, 202, 201]
    assert answer["survey"]["frame"]["origin"] == 204


@pytest.mark.parametrize("state", ["asleep", "standby", "off"])
async def test_no_anchors_named_leaves_out_the_anchors_that_do_not_operate(
    dialect, nursery, builder, hub, autojump_clock, state
):
    cell = await survey_cell(dialect, nursery, TIERS)
    await cell.asleep(206)
    code = {"asleep": 3.0, "standby": 2.0, "off": 4.0}[state]
    await cell.report([203], **(ASLEEP | {"pwrst": code}))

    answer = await request(cell, builder, hub, "start", frame=TWO_TIERS)

    assert (answer["result"], answer["reason"]) == ("started", None)
    assert answer["survey"]["anchors"] == [201, 202, 204, 205, 207, 208]


async def test_no_anchors_named_waits_for_an_anchor_of_unknown_power_state(
    cell, builder, hub, autojump_clock
):
    # it may well be ranging: leaving it out would restart it at every step
    health = {k: v for k, v in HEALTHY_A1.items() if k != "pwrst"}
    await cell.ext._on_stats(203, health, time.monotonic())

    answer = await request(cell, builder, hub, "start", frame=TWO_TIERS)

    assert (answer["result"], answer["reason"]) == (
        "refused",
        "RTLS anchor A2 has not reported its power state",
    )


async def test_a_step_is_timed_by_the_longest_block_the_group_stores(
    cell, builder, hub, autojump_clock
):
    devices = cell.ext._protocol.devices
    for system_id in IDS:
        del devices[system_id].params["UWB_BLOCK_MS"]
    # none read: the slowest preset, 30 ms
    await measure(cell, builder, hub, rounds=20)
    assert cell.steps()[0][8:10] == "2b"

    cell.commands.clear()
    devices[202].params["UWB_BLOCK_MS"] = (12).to_bytes(2, "little")
    devices[203].params["UWB_BLOCK_MS"] = (20).to_bytes(2, "little")
    await measure(cell, builder, hub, rounds=20)
    assert cell.steps()[0][8:10] == "25"


async def test_a_whole_number_of_rounds_is_taken_as_that_integer(
    cell, builder, hub, autojump_clock
):
    survey = await measure(cell, builder, hub, rounds=100.0)
    assert survey["state"] == "ready"
    assert cell.steps()[0][4:8] == "0064"


async def test_rounds_set_the_command_and_how_long_a_step_is_held(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub, rounds=20)
    # 1.5 s + 3 x 20 rounds x 20 ms, held one second longer
    assert cell.steps()[0][4:10] == "0014" + "25"


async def test_a_step_with_a_report_missing_is_run_once_more(
    cell, builder, hub, autojump_clock
):
    # 203 completes the first attempt of step 1 only, 204 the second only
    cell.lost_reports = {203: {2}, 204: {1}}

    survey = await measure(cell, builder, hub)

    first, again = cell.steps()[:2]
    assert _initiators(cell) == [1, 1, 2, 3, 4]
    assert int(again[:4], 16) == int(first[:4], 16) % 0xFFFF + 1
    assert cell.sent(201, again)[0] - cell.sent(201, first)[0] == STEP_LIMIT
    # the anchors hold a step longer than it is waited for: no repeat
    assert len(cell.sent(201, first)) == 1
    assert survey["state"] == "ready"
    assert all(pair["ab"] and pair["ba"] for pair in survey["pairs"])


async def test_a_refusal_of_the_second_attempt_ends_the_run(
    cell, builder, hub, autojump_clock
):
    # 203 completed the first attempt of step 1; 204 did not, so the step is
    # run once more, and that command 203 refuses
    cell.lost_reports[204] = {1}
    cell.refused_steps[203] = {2}

    survey = await measure(cell, builder, hub)

    assert (survey["state"], survey["reason"]) == (
        "failed",
        "RTLS anchor A2 refused the survey step: its radio is not running,"
        " or the step does not fit its slots",
    )
    assert _initiators(cell) == [1, 1]


async def test_a_report_still_missing_leaves_its_range_out(
    cell, builder, hub, autojump_clock
):
    cell.lost_reports[203] = {1, 2}

    survey = await measure(cell, builder, hub)

    assert _initiators(cell) == [1, 1, 2, 3, 4]
    assert survey["state"] == "ready"
    pair = next(p for p in survey["pairs"] if (p["a"], p["b"]) == (201, 203))
    assert pair["ab"] is None and pair["ba"]["n"] == 98


@pytest.mark.parametrize("fault", ["misreporting", "wrong_role"])
async def test_a_report_counts_only_from_the_anchors_role_and_to_the_initiator(
    cell, builder, hub, autojump_clock, fault
):
    # 203 reports its range to another peer, or reports as if it led the step
    getattr(cell, fault).add(203)

    survey = await measure(cell, builder, hub)

    # the steps it does not lead are not complete without it: each runs twice
    assert _initiators(cell) == [1, 1, 2, 2, 3, 4, 4]
    for pair in survey["pairs"]:
        measured_by_203 = "ab" if pair["b"] == 203 else "ba"
        if 203 in (pair["a"], pair["b"]):
            assert pair[measured_by_203] is None, pair
        else:
            assert pair["ab"] and pair["ba"]


async def test_a_responder_that_heard_nothing_measured_no_range(
    cell, builder, hub, autojump_clock
):
    cell.share[201, 203] = 0.0

    survey = await measure(cell, builder, hub)

    # its report says zero samples and a range of zero: a missing direction
    pair = next(p for p in survey["pairs"] if (p["a"], p["b"]) == (201, 203))
    assert pair["ab"] is None and pair["ba"]["n"] == 98
    assert pair["rangeM"] == pair["ba"]["rangeM"]
    assert survey["gate"] == {"blocked": [], "warnings": []}


async def test_a_radio_lost_after_the_report_does_not_take_the_range_back(
    cell, autojump_clock
):
    group = [SurveyAnchor(system_id, cell.mac[system_id]) for system_id in IDS]
    cell.late[204] = 0.5
    lost = {"svid": 5, "svrole": 3, "svst": 3, "svrnd": 100}

    async def lose_the_radio():
        await trio.sleep(0.4)
        device = cell.devices[203]
        datagram = b"".join(
            device.named_value_float(name, float(value), 9999)
            for name, value in lost.items()
        )
        await cell.ext._process_datagram(datagram, device.address, 0.0)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(lose_the_radio)
        result = await run_step(cell.ext, group, group[0], 5, 100, HOLD, timeout=2.0)

    assert result.complete
    assert result.reports[203].samples == 98


async def test_a_restart_in_a_step_does_not_read_as_a_cell_that_differs(
    cell, builder, hub, autojump_clock
):
    ext = cell.ext
    restarting = {"cellcfg": -1.0, "txgain": -1.0, "sid": -1.0}
    cell.collect_s = 1.2
    await cell.asleep(204)
    await _start(cell, builder, hub, anchors=[201, 202, 203])
    await trio.sleep(0.5)
    ext._inf_pending = False

    # 201 is the cell's reference: without it every device reads unknown
    await cell.report([201], **restarting)

    kept = {"cellConfig": CODE_50, "txGain": 47}
    assert {key: ext._stats[201][key] for key in kept} == kept
    assert not ext._inf_pending
    await cell.report([204], **restarting)
    # an anchor outside the survey is believed
    assert ext._stats[204]["cellConfig"] is None
    assert cell_states(ext)[201] == cell_states(ext)[202] == ("ok", None)
    assert cell_states(ext)[204] == ("unknown", None)
    warnings = (await _lock(ext, builder, hub, "status"))["warnings"]
    assert {w["device"] for w in warnings if w["code"] == "cell-settings"} == {204}

    # once the survey is over, an anchor without a session reads as one
    await until(cell, "returning")
    await cell.report([202, 203])
    await until(cell, "ready")
    await cell.report([201], **restarting)
    assert ext._stats[201]["cellConfig"] is None


async def test_a_restart_in_a_commit_keeps_the_survey_id_until_the_new_one(
    cell, builder, hub, autojump_clock
):
    await measure(cell, builder, hub)
    await commit(cell, builder, hub)
    await measure(cell, builder, hub)
    cell.location_silent.add(204)
    task = {}

    async def ask():
        task["answer"] = await request(cell, builder, hub, "commit")

    async with trio.open_nursery() as nursery:
        nursery.start_soon(ask)
        await trio.sleep(0.1)
        await cell.report([201], sid=-1.0)
        assert cell.ext._stats[201]["surveyId"] == 1
        await cell.report()

    assert task["answer"]["survey"]["commit"]["running"] == [201, 202, 203]
    assert cell.ext._stats[201]["surveyId"] == 2


async def test_an_anchor_whose_radio_fails_the_steps_loses_its_pairs(
    cell, builder, hub, autojump_clock
):
    cell.failing.add(203)
    placement = {str(k): {"x": x, "y": y} for k, (x, y, _) in FLAT.items()}

    survey = await measure(cell, builder, hub, placement=placement)

    assert _initiators(cell) == [1, 1, 2, 2, 3, 3, 4, 4]
    assert survey["state"] == "ready"
    assert survey["placement"] == placement
    missing = [
        (entry["device"], entry["peer"])
        for entry in survey["gate"]["blocked"]
        if entry["code"] == "pair-missing"
    ]
    assert missing == [(201, 203), (202, 203), (203, 204)]


async def test_a_measurement_the_fit_cannot_start_from_fails_the_run(
    cell, builder, hub, autojump_clock
):
    cell.failing.add(203)

    survey = await measure(cell, builder, hub)

    assert (survey["state"], survey["reason"]) == (
        "failed",
        "Place the anchors roughly first",
    )
    assert survey["fit"] is None


async def test_a_long_step_is_kept_alive_by_repeating_its_command(
    cell, builder, hub, autojump_clock
):
    cell.lost_reports[203] = {1}
    await _start(cell, builder, hub, rounds=1000)
    await trio.sleep(31)

    # the command carries at most 10 s of hold; the wait is 61.5 s
    first = cell.steps()[0]
    assert first[4:10] == "03e8" + "64"
    sent = cell.sent(201, first)
    assert [at - sent[0] for at in sent] == [0, 5, 10, 15, 20, 25, 30]


async def test_a_missing_ack_is_sent_again_and_the_run_goes_on(
    cell, builder, hub, autojump_clock
):
    cell.unheard[204] = 2

    survey = await measure(cell, builder, hub)

    sent = cell.sent(204, cell.steps()[0])
    assert [round(at - sent[0], 3) for at in sent] == [0, 0.3, 0.6]
    assert survey["state"] == "ready"


async def test_an_anchor_that_never_answers_ends_the_run(
    cell, builder, hub, autojump_clock
):
    cell.silent.add(204)

    survey = await measure(cell, builder, hub)

    step = cell.steps()[0]
    sent = cell.sent(204, step)
    assert [round(at - sent[0], 3) for at in sent] == [0, 0.3, 0.6, 0.9]
    assert cell.steps() == [step, "0000"]
    assert (survey["state"], survey["reason"]) == (
        "failed",
        "RTLS anchor A3 did not answer",
    )
    assert survey["anchors"] == IDS and survey["fit"] is None


async def test_a_late_ack_of_the_step_before_is_not_this_steps(
    cell, builder, hub, autojump_clock
):
    # 204 answers after the write was sent again, so each ack comes twice;
    # then it stops hearing: the second ack of step 1 lands in step 2
    cell.late[204] = 0.4
    await _start(cell, builder, hub)
    await trio.sleep(0.35)
    cell.silent.add(204)

    await until(cell, "failed")
    assert cell.ext._survey.json["reason"] == "RTLS anchor A3 did not answer"
    assert _initiators(cell) == [1, 2]


@pytest.mark.parametrize(
    "stats, why", [({"lock": 1.0}, "locked"), ({"pwrst": 2.0}, "not operating")]
)
async def test_a_refused_step_ends_the_run_and_returns_the_anchors(
    cell, builder, hub, autojump_clock, stats, why
):
    cell.collect_s = 1.2
    await _start(cell, builder, hub)
    await trio.sleep(1.5)
    cell.refusing.add(203)
    await cell.report([203], **stats)

    await until(cell, "returning")
    returning_at = trio.current_time()
    await cell.report()
    await until(cell, "failed")

    assert cell.ext._survey.json["reason"] == (
        f"RTLS anchor A2 refused the survey step: {why}"
    )
    assert _initiators(cell) == [1, 2, 3]
    assert {to for to, value, _ in cell.commands if value == "0000"} == set(IDS)
    # an anchor that did not take the end returns when its hold is over
    assert trio.current_time() - returning_at == pytest.approx(HOLD, abs=0.02)


async def test_a_refusal_names_what_the_anchor_reports(
    cell, builder, hub, autojump_clock
):
    cell.refusing |= {204, 202}
    cell.collect_s = 1.2
    survey = await measure(cell, builder, hub)
    # the run does not wait for the reports of a step an anchor refused
    assert cell.sent(201, "0000") == cell.sent(201, cell.steps()[0])
    assert survey["reason"] == (
        "RTLS anchor A1 refused the survey step: its radio is not running,"
        " or the step does not fit its slots"
    )


async def test_an_anchor_that_goes_offline_ends_the_run(
    cell, builder, hub, autojump_clock
):
    cell.collect_s = 1.2
    await _start(cell, builder, hub)
    await trio.sleep(1.5)

    cell.lose(203)
    await until(cell, "failed")

    assert cell.ext._survey.json["reason"] == "RTLS anchor A2 went offline"
    assert _initiators(cell) == [1, 2]


async def test_cancel_returns_the_anchors_and_ends_idle(
    cell, builder, hub, autojump_clock
):
    cell.collect_s = 1.2
    await _start(cell, builder, hub)
    await trio.sleep(1.5)

    answer = await request(cell, builder, hub, "cancel")

    assert (answer["result"], answer["reason"]) == ("cancelled", None)
    assert (answer["survey"]["state"], answer["survey"]["step"]) == ("returning", 2)
    await trio.sleep(0.1)
    await cell.report()
    await until(cell, "idle")
    assert held(cell.ext._survey.json) == IDLE | {"reason": "cancelled"}
    assert _initiators(cell) == [1, 2]
    assert {to for to, value, _ in cell.commands if value == "0000"} == set(IDS)

    answer = await request(cell, builder, hub, "cancel")
    assert (answer["result"], answer["reason"]) == ("refused", "No survey is running")


async def test_cancel_while_the_anchors_return_drops_the_measurement(
    cell, builder, hub, autojump_clock
):
    await _start(cell, builder, hub)
    await until(cell, "returning")

    answer = await request(cell, builder, hub, "cancel")
    assert (answer["result"], answer["survey"]["state"]) == ("cancelled", "returning")
    await cell.report()
    await until(cell, "idle")
    assert held(cell.ext._survey.json) == IDLE | {"reason": "cancelled"}


async def test_a_step_names_an_anchor_that_is_not_live(cell, autojump_clock):
    group = [SurveyAnchor(system_id, cell.mac[system_id]) for system_id in IDS]
    cell.lose(203)

    result = await run_step(cell.ext, group, group[0], 5, 100, HOLD, timeout=1.0)

    offline = {203: StepProblem("offline", "is offline")}
    assert {k: p for k, p in result.problems.items() if p.ends_survey} == offline
    assert result.ends_survey and not result.complete
    assert await end_survey(cell.ext, group) == offline


async def test_a_report_of_another_step_or_anchor_is_not_this_steps(
    cell, autojump_clock
):
    group = [SurveyAnchor(system_id, cell.mac[system_id]) for system_id in IDS[:3]]
    cell.lost_reports = {201: {1}, 202: {1}, 203: {1}}
    stale = cell._report(cell.devices[202], encode_step(4, 100, HOLD, 1, [2, 3]))
    foreign = cell._report(cell.devices[204], encode_step(5, 100, HOLD, 1, [2, 4]))

    async def late():
        await trio.sleep(0.5)
        for system_id, datagram in ((202, stale), (204, foreign)):
            address = cell.devices[system_id].address
            await cell.ext._process_datagram(datagram, address, 0.0)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(late)
        result = await run_step(cell.ext, group, group[0], 5, 100, HOLD, timeout=1.0)

    assert result.reports == {}
    assert result.problems == {
        system_id: StepProblem("no-report", "sent no complete report")
        for system_id in IDS[:3]
    }
    assert not result.ends_survey


async def test_the_return_waits_for_the_boot_responders_to_range(
    cell, builder, hub, autojump_clock
):
    await _start(cell, builder, hub)
    await until(cell, "returning")

    await cell.report([201, 202, 203])
    await trio.sleep(1)
    assert cell.ext._survey.state == "returning"
    await cell.report([204], rnghz=0.0)
    await trio.sleep(1)
    assert cell.ext._survey.state == "returning"
    await cell.report([204])
    await until(cell, "ready")


async def test_ranging_reported_before_the_end_is_answered_does_not_count(
    cell, builder, hub, autojump_clock
):
    cell.collect_s = 1.2
    await _start(cell, builder, hub)
    await trio.sleep(0.5)
    await request(cell, builder, hub, "cancel")
    started = trio.current_time()

    # in the step the responders range too, to the step's initiator
    await cell.report()
    await until(cell, "idle")
    assert trio.current_time() - started == pytest.approx(HOLD, abs=0.02)


async def test_the_return_waits_no_longer_than_the_hold(
    cell, builder, hub, autojump_clock
):
    await _start(cell, builder, hub)
    await until(cell, "returning")
    started = trio.current_time()

    await until(cell, "ready")
    assert trio.current_time() - started == pytest.approx(HOLD, abs=0.02)


async def test_a_defect_in_the_run_fails_the_survey_only(
    cell, builder, hub, autojump_clock, monkeypatch
):
    async def broken(*args, **kwargs):
        raise RuntimeError("boom")

    monkeypatch.setattr(survey_module, "run_step", broken)
    survey = await measure(cell, builder, hub)
    assert (survey["state"], survey["reason"]) == (
        "failed",
        "The survey stopped on an internal error",
    )
    assert cell.steps() == ["0000"]

    monkeypatch.undo()
    assert (await measure(cell, builder, hub))["state"] == "ready"


async def _refusal(cell, builder, hub, **body):
    answer = await _start(cell, builder, hub, **body)
    assert answer["result"] == "refused"
    assert held(answer["survey"]) == IDLE
    assert cell.commands == []
    return answer["reason"]


async def test_start_is_refused_while_the_anchors_are_locked(
    cell, builder, hub, autojump_clock
):
    await cell.report([204], lock=1.0)
    assert await _refusal(cell, builder, hub) == "Unlock the anchors first"


async def test_a_lock_counts_before_the_devices_role_is_known(
    cell, builder, hub, autojump_clock
):
    # 204 was lost and is back: it reports its lock, its role is not read yet
    del cell.ext._protocol.devices[204].params["UWB_ROLE"]
    await cell.report([204], lock=1.0)

    reason = await _refusal(cell, builder, hub, anchors=[201, 202, 203])
    assert reason == "Unlock the anchors first"


async def test_start_is_refused_while_a_show_is_authorized(
    cell, builder, hub, autojump_clock
):
    authorize_show(cell.ext)
    assert await _refusal(cell, builder, hub) == "A show is authorized"


async def test_start_is_refused_while_an_apply_runs(cell, builder, hub, autojump_clock):
    cell.ext._cell_applying = True
    assert await _refusal(cell, builder, hub) == "An apply is running"


async def test_start_is_refused_while_the_extension_does_not_run(
    cell, builder, hub, autojump_clock
):
    cell.ext._nursery = None
    assert await _refusal(cell, builder, hub) == "RTLS is not running"


async def test_start_needs_every_anchors_own_mac(cell, builder, hub, autojump_clock):
    devices = cell.ext._protocol.devices
    devices[204].params["UWB_MAC"] = devices[203].params["UWB_MAC"]
    assert await _refusal(cell, builder, hub) == (
        "Two anchors of the survey have the same UWB_MAC"
    )
    del devices[203].params["UWB_MAC"]
    assert await _refusal(cell, builder, hub) == (
        "RTLS anchor 203: UWB_MAC not read yet"
    )


async def test_a_survey_runs_without_an_app_to_tell(cell, builder, hub, autojump_clock):
    cell.ext.app = None
    answer = await _start(cell, builder, hub)
    assert answer["result"] == "started"
    await until(cell, "returning")
    await cell.report()
    await until(cell, "ready")


async def test_start_is_refused_for_an_anchor_that_cannot_take_part(
    cell, builder, hub, autojump_clock
):
    await cell.report([203], cellcfg=-1.0)
    assert await _refusal(cell, builder, hub) == "RTLS anchor A2 is not ranging"
    await cell.report([203], pwrst=3.0)
    assert await _refusal(cell, builder, hub) == (
        "RTLS anchor A2 is asleep; wake it first"
    )
    cell.lose(203)
    assert await _refusal(cell, builder, hub) == "RTLS anchor A2 is offline"


async def test_start_waits_for_an_anchor_that_has_not_reported_its_session(
    cell, builder, hub, autojump_clock
):
    # after a server restart the health report comes first, the session
    # only with the status report
    health = {k: v for k, v in HEALTHY_A1.items() if k not in ("cellcfg", "txgain")}
    await cell.ext._on_stats(202, health, time.monotonic())

    assert await _refusal(cell, builder, hub) == (
        "Still waiting for RTLS anchor A1 to report its session"
    )
    await cell.report([202])
    assert (await _start(cell, builder, hub))["result"] == "started"


async def test_start_refuses_to_leave_an_operating_anchor_of_the_cell_out(
    cell, builder, hub, autojump_clock
):
    assert await _refusal(cell, builder, hub, anchors=[201, 202, 203]) == (
        "Include RTLS anchor A3 in the survey, or put it to sleep"
    )


async def test_start_names_every_anchor_left_out(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await survey_cell(dialect, nursery, TIERS)

    reason = await _refusal(cell, builder, hub, anchors=list(TIERS)[:6])

    assert reason == (
        "Include RTLS anchor A6, RTLS anchor A7 in the survey, or put them to sleep"
    )


async def test_start_refuses_to_leave_out_an_anchor_of_unknown_power_state(
    cell, builder, hub, autojump_clock
):
    # the same rule as for the default group: it may well be ranging
    health = {k: v for k, v in HEALTHY_A1.items() if k != "pwrst"}
    await cell.ext._on_stats(204, health, time.monotonic())

    assert await _refusal(cell, builder, hub, anchors=[201, 202, 203]) == (
        "Include RTLS anchor A3 in the survey, or put it to sleep"
    )


@pytest.mark.parametrize("state", ["asleep", "standby", "offline", "other cell"])
async def test_an_anchor_that_does_not_range_in_the_cell_may_be_left_out(
    cell, builder, hub, autojump_clock, state
):
    if state == "asleep":
        await cell.asleep(204)
    elif state == "standby":
        await cell.report([204], **(ASLEEP | {"pwrst": 2.0}))
    elif state == "offline":
        cell.lose(204)
    else:
        await cell.ext.set_param(204, "CELL_ID", "hall-b", "custom")

    answer = await _start(cell, builder, hub, anchors=[201, 202, 203])

    assert (answer["result"], answer["reason"]) == ("started", None)


@pytest.mark.parametrize(
    "body, reason",
    [
        ({"anchors": [201, 202]}, "A survey takes three to eight anchors, not 2"),
        (
            {"anchors": list(range(201, 210))},
            "A survey takes three to eight anchors, not 9",
        ),
        ({"anchors": [201, 202, 42]}, "Device 42 is not a known anchor"),
        (
            {"frame": {"origin": 201, "xAxis": 209}},
            "The frame names anchor 209, which is not in the survey",
        ),
        (
            {"frame": {"origin": 201, "xAxis": 201}},
            "The origin and the +x anchor must be two anchors",
        ),
    ],
)
async def test_start_is_refused_for_a_group_or_frame_that_cannot_work(
    cell, builder, hub, autojump_clock, body, reason
):
    assert await _refusal(cell, builder, hub, **body) == reason


async def test_a_second_start_is_refused_while_a_survey_runs(
    cell, builder, hub, autojump_clock
):
    cell.collect_s = 1.2
    await _start(cell, builder, hub)
    answer = await _start(cell, builder, hub)
    assert (answer["result"], answer["reason"]) == ("refused", "A survey is running")
    assert answer["survey"]["state"] == "measuring"


@pytest.mark.parametrize(
    "body",
    [
        {"action": "measure"},
        {"action": "start", "anchors": IDS},
        {"action": "start", "anchors": IDS, "frame": {"preset": "ring"}},
        {"action": "start", "anchors": IDS, "frame": TWO_TIERS, "rounds": 10},
        {"action": "start", "anchors": IDS, "frame": TWO_TIERS, "rounds": 100.5},
        {"action": "start", "anchors": IDS, "frame": TWO_TIERS, "rounds": "100"},
        {"action": "start", "anchors": IDS, "frame": TWO_TIERS, "rounds": True},
        {"action": "start", "anchors": IDS, "frame": TWO_TIERS, "rounds": float("nan")},
        {"action": "start", "anchors": [201, 201, 202], "frame": TWO_TIERS},
        {"action": "start", "anchors": "all", "frame": TWO_TIERS},
        {"action": "start", "anchors": IDS, "frame": {"origin": 201, "xAxis": "A1"}},
        {"action": "start", "anchors": IDS, "frame": FRAME | {"yPositive": 1.5}},
        {"action": "start", "anchors": IDS, "frame": FRAME | {"heightGroups": 201}},
        {"action": "start", "anchors": IDS, "frame": FRAME | {"heightGroups": [201]}},
        {"action": "start", "anchors": IDS, "frame": FRAME | {"upper": [True]}},
        *(
            {"action": "start", "anchors": IDS, "frame": TWO_TIERS, "placement": p}
            for p in (
                [201],
                {"201": {"x": 0}},
                {"201": [0, 0]},
                {"A1": {"x": 0, "y": 0}},
                {"201": {"x": True, "y": 0}},
                {"201": {"x": 0, "y": float("inf")}},
                {"201": {"x": 0, "y": 0, "z": "up"}},
            )
        ),
    ],
)
async def test_a_malformed_request_is_rejected(
    cell, builder, hub, autojump_clock, body
):
    response = await request(cell, builder, hub, **body)
    assert response["type"] == "ACK-NAK", body
    assert cell.ext._survey.state == "idle"


async def test_other_writes_to_the_cell_are_refused_while_a_survey_runs(
    cell, builder, hub, autojump_clock, tmp_path
):
    ext = cell.ext
    cell.collect_s = 1.2
    await cell.asleep(204)
    await _start(cell, builder, hub, anchors=[201, 202, 203])
    await trio.sleep(0.5)
    refusal = ("refused", "A survey is running")

    body = await cell_request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == refusal
    body = await _lock(ext, builder, hub)
    assert (body["result"], body["reason"]) == refusal
    assert body["results"] == {}

    message = {"type": "X-RTLS-SLEEP", "ids": [202, 204], "sleeping": True}
    response = await ext._handle_RTLS_SLEEP(make_message(builder, message), None, hub)
    assert response.body["result"]["202"] == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "A survey is running",
    }
    assert response.body["result"]["204"]["accepted"] is True

    image = tmp_path / "anchor.bin"
    image.write_bytes(b"image")
    with pytest.raises(RuntimeError, match="A survey is running"):
        await ext.start_ota(202, str(image), role="anchor")

    await until(cell, "returning")
    assert (await _lock(ext, builder, hub))["reason"] == "A survey is running"
    await cell.report()
    await until(cell, "ready")
    body = await cell_request(ext, builder, hub, rateHz=83)
    assert body["reason"] != "A survey is running"


async def test_the_state_is_pushed_at_most_twice_a_second(
    cell, builder, hub, autojump_clock
):
    survey = cell.ext._survey
    await measure(cell, builder, hub)

    # the steps of this cell take no time: only the first push went out
    assert [s["state"] for s in cell.broadcasts] == ["measuring"]
    await trio.sleep(0.45)
    assert len(cell.broadcasts) == 1

    # the last change of the burst is pushed when the half second is over
    await trio.sleep(0.05)
    assert [s["state"] for s in cell.broadcasts] == ["measuring", "ready"]
    assert cell.ext.app.message_hub.broadcasts[-1] == {
        "type": "X-RTLS-SURVEY",
        "survey": survey.json,
    }
    await trio.sleep(1)
    assert len(cell.broadcasts) == 2


async def test_a_full_notification_queue_holds_up_neither_the_run_nor_its_answers(
    cell, builder, hub, autojump_clock
):
    delivered, room = [], trio.Event()

    async def broadcast(message):
        await room.wait()
        delivered.append(message["survey"]["state"])

    cell.ext.app.message_hub.broadcast_message = broadcast

    with trio.fail_after(60):
        answer = await _start(cell, builder, hub)
        assert answer["result"] == "started"
        await until(cell, "returning")
        await cell.report()
        await until(cell, "ready")
        assert (await request(cell, builder, hub, "fit"))["result"] == "fitted"

    assert {to for to, value, _ in cell.commands if value == "0000"} == set(IDS)
    assert not cell.ext._survey.running
    assert delivered == []

    # once there is room, what is delivered ends with the state as it is
    room.set()
    await trio.sleep(1)
    assert delivered == ["measuring", "ready"]


async def test_a_notification_that_cannot_be_sent_does_not_stop_the_survey(
    cell, builder, hub, autojump_clock
):
    async def broken(message):
        raise trio.BrokenResourceError

    cell.ext.app.message_hub.broadcast_message = broken
    assert (await measure(cell, builder, hub))["state"] == "ready"


async def test_a_stopped_extension_keeps_no_survey(cell, builder, hub, autojump_clock):
    await measure(cell, builder, hub)
    cell.ext._forget_run_state()
    assert held(cell.ext._survey.json) == IDLE
