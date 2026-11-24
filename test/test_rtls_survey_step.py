"""The survey step command and report (rtls/survey_step.py) against the
firmware's wire contract."""

import pytest
from rtls_support import FakeDevice
from rtlslink.dialect import load_dialect

from flockwave.server.ext.rtls.survey_step import (
    ReportScanner,
    StepReport,
    encode_step,
    hold_for,
    next_step_id,
    time_limit,
)

RESPONDER = {
    "svid": 0x0102,
    "svrole": 3,
    "svst": 2,
    "svrnd": 100,
    "svpeer": 3,
    "svrng": 4.5,
    "svmad": 0.25,
    "svn": 96,
    "svok": 98,
}


@pytest.fixture(scope="module")
def anchor():
    return FakeDevice(load_dialect(), system_id=205)


def _datagram(anchor, frames, time_boot_ms=1000):
    return b"".join(
        anchor.named_value_float(name, float(value), time_boot_ms)
        for name, value in frames.items()
    )


def test_the_command_matches_the_firmware_vector():
    # rtls-link-zephyr docs/reference/anchor-geometry.md, "Survey step"
    command = encode_step(0x0102, 100, 3.0, 3, [1, 2, 4, 5, 6, 7, 8])
    assert command == "010200641e00030001000200040005000600070008"


def test_the_hold_is_rounded_up_to_the_tenth_the_command_carries():
    assert encode_step(1, 20, 2.71, 1, [2])[8:10] == "1c"
    assert encode_step(1, 20, 2.7, 1, [2])[8:10] == "1b"


@pytest.mark.parametrize(
    "arguments",
    [
        (0, 100, 3.0, 1, [2]),
        (0x10000, 100, 3.0, 1, [2]),
        (1, 19, 3.0, 1, [2]),
        (1, 1001, 3.0, 1, [2]),
        (1, 100, 0.9, 1, [2]),
        (1, 100, 10.1, 1, [2]),
        (1, 100, 3.0, 1, []),
        (1, 100, 3.0, 1, [2, 3, 4, 5, 6, 7, 8, 9]),
        (1, 100, 3.0, 1, [2, 2]),
        (1, 100, 3.0, 1, [1]),
        (1, 100, 3.0, 0, [2]),
        (1, 100, 3.0, 1, [0xFFFF]),
    ],
)
def test_a_step_the_command_cannot_carry_is_an_error(arguments):
    with pytest.raises(ValueError):
        encode_step(*arguments)


def test_step_ids_count_up_and_skip_zero():
    assert next_step_id(1) == 2
    assert next_step_id(0xFFFE) == 0xFFFF
    assert next_step_id(0xFFFF) == 1


def test_a_step_is_waited_for_three_collections_and_held_a_second_longer():
    assert time_limit(100, 12) == pytest.approx(5.1)
    assert hold_for(100, 12) == pytest.approx(6.1)
    # the command carries at most ten seconds
    assert time_limit(1000, 12) == pytest.approx(37.5)
    assert hold_for(1000, 12) == 10.0


def test_a_complete_responder_report_is_one_event(anchor):
    (event,) = ReportScanner().events(_datagram(anchor, RESPONDER))

    assert (event.kind, event.system_id) == ("survey_report", 205)
    assert StepReport.from_event(event) == StepReport(
        system_id=205,
        step_id=0x0102,
        role=3,
        state=2,
        samples=96,
        peer_mac=3,
        range_m=4.5,
        mad_m=0.25,
        valid=98,
    )


def test_an_initiator_reports_the_rounds_it_led(anchor):
    frames = {"svid": 7, "svrole": 2, "svst": 2, "svrnd": 100, "svn": 100}
    (event,) = ReportScanner().events(_datagram(anchor, frames))
    assert StepReport.from_event(event) == StepReport(205, 7, 2, 2, samples=100)


@pytest.mark.parametrize("state", [1, 3])
def test_a_report_that_is_not_complete_carries_no_range(anchor, state):
    frames = {"svid": 7, "svrole": 3, "svst": state, "svrnd": 12}
    (event,) = ReportScanner().events(_datagram(anchor, frames))
    assert StepReport.from_event(event) == StepReport(205, 7, 3, state)


@pytest.mark.parametrize(
    "change",
    [
        {"svrng": None},
        {"svmad": None},
        {"svpeer": None},
        {"svn": None},
        {"svok": None},
        {"svst": None},
        {"svrole": None},
        {"svid": None},
        {"svid": 0},
        {"svid": 7.5},
        {"svrole": 1},
        {"svst": 4},
        {"svrng": float("nan")},
        {"svpeer": 0xFFFF},
    ],
)
def test_a_partial_or_undecodable_report_is_dropped(anchor, change):
    frames = {k: v for k, v in (RESPONDER | change).items() if v is not None}
    assert ReportScanner().events(_datagram(anchor, frames)) == []


def test_the_frames_of_two_reports_are_not_mixed(anchor):
    # an older report's range frames beside a newer, still collecting one
    older = _datagram(anchor, RESPONDER, time_boot_ms=1000)
    collecting = {"svid": 0x0103, "svrole": 3, "svst": 1, "svrnd": 5}
    newer = _datagram(anchor, collecting, time_boot_ms=1500)

    events = ReportScanner().events(older + newer)

    assert [e.data["stepId"] for e in events] == [0x0102, 0x0103]
    assert "rangeM" not in events[1].data


def test_other_telemetry_is_not_a_report(anchor):
    scanner = ReportScanner()
    assert scanner.events(anchor.heartbeat()) == []
    assert scanner.events(anchor.named_value_float("rnghz", 50.0)) == []
