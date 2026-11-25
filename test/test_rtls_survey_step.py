"""What the survey relies on of the firmware SDK (rtlslink.survey and the
``survey_report`` protocol event), through the extension's own transport."""

import pytest
from rtlslink.dialect import load_dialect
from rtlslink.survey import StepReport
from survey_support import survey_cell

from flockwave.server.ext.rtls.survey_step import next_step_id

RESPONDER = {
    "svid": 0x0102,
    "svrole": 3,
    "svst": 2,
    "svrnd": 101,
    "svpeer": 1,
    "svrng": 4.5,
    "svmad": 0.25,
    "svn": 96,
    "svok": 98,
}


@pytest.fixture(scope="module")
def dialect():
    return load_dialect()


async def _reports(cell, system_id, frames, time_boot_ms=1000):
    """The reports the extension's event stream carries for one datagram."""
    device = cell.devices[system_id]
    datagram = b"".join(
        device.named_value_float(name, float(value), time_boot_ms)
        for name, value in frames.items()
    )
    with cell.ext._subscribed_events() as events:
        await cell.ext._process_datagram(datagram, device.address, 0.0)
        reports = []
        while events.statistics().current_buffer_used:
            event = events.receive_nowait()
            if event.kind == "survey_report":
                reports.append(StepReport.from_event(event))
    return reports


def test_step_ids_count_up_and_skip_zero():
    assert next_step_id(1) == 2
    assert next_step_id(0xFFFE) == 0xFFFF
    assert next_step_id(0xFFFF) == 1


async def test_a_responders_report_reaches_the_survey_whole(
    dialect, nursery, autojump_clock
):
    cell = await survey_cell(dialect, nursery)

    assert await _reports(cell, 203, RESPONDER) == [
        StepReport(
            system_id=203,
            step_id=0x0102,
            role=3,
            state=2,
            rounds=101,
            time_boot_ms=1000,
            samples=96,
            peer_mac=1,
            range_m=4.5,
            mad_m=0.25,
            valid=98,
        )
    ]
    # the anchor repeats a report while it is in the step: one event
    assert await _reports(cell, 203, RESPONDER) == []


async def test_a_report_with_a_frame_missing_does_not_reach_the_survey(
    dialect, nursery, autojump_clock
):
    cell = await survey_cell(dialect, nursery)
    partial = {k: v for k, v in RESPONDER.items() if k != "svrng"}
    assert await _reports(cell, 203, partial) == []


@pytest.mark.parametrize("state", [1, 3])
async def test_a_report_that_is_not_complete_carries_no_range(
    dialect, nursery, autojump_clock, state
):
    cell = await survey_cell(dialect, nursery)
    frames = {"svid": 7, "svrole": 3, "svst": state, "svrnd": 12}
    assert await _reports(cell, 203, frames) == [
        StepReport(203, 7, 3, state, rounds=12, time_boot_ms=1000)
    ]
