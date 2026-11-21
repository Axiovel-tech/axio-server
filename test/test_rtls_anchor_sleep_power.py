"""X-RTLS-SLEEP on anchors judged by their ``powerState``: standby falls
asleep in place, operating transitions reboot, and plain standby (SLEEP 3)
is not asleep."""

import pytest
import trio
import trio.testing
from anchor_sleep_support import (
    FakeAnchor,
    _anchor_sleep,
    _power_sleep,
)
from rtlslink.dialect import load_dialect

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


async def test_anchor_already_asleep_by_power_state(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, power="asleep", standby=True, reports_power_state=True)

    entry, _ = await _power_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True and entry["sleeping"] is True


async def test_standby_anchor_sleeps_in_place(dialect, builder, hub, autojump_clock):
    anchor = FakeAnchor(
        dialect, standby=True, in_place_sleep=True, reports_power_state=True
    )

    entry, elapsed = await _power_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": True,
        "sleeping": True,
        "detail": "asleep",
    }
    assert anchor.power == "asleep"
    assert anchor.reboot_at is None and not anchor.expired
    assert elapsed < anchor.reboot_s


async def test_standby_anchor_is_not_reported_asleep_while_in_standby(
    dialect, builder, hub, autojump_clock
):
    # firmware that leaves a standby anchor in standby on SLEEP=1 (it already
    # reads 1): STANDBY heartbeats alone must not count as asleep
    anchor = FakeAnchor(dialect, standby=True, reports_power_state=True)

    entry, _ = await _power_sleep(dialect, builder, hub, anchor, True, timeout=6)

    assert entry["accepted"] is False
    assert entry["sleeping"] is None
    assert "not seen asleep" in entry["detail"]


async def test_standby_anchor_wakes_through_its_reboot(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, standby=True, reports_power_state=True)

    entry, elapsed = await _power_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is True
    assert entry["sleeping"] is False
    assert anchor.power == "operating"
    # confirmed only after the reboot, not on a pre-reboot report
    assert anchor.expired
    assert elapsed >= anchor.reboot_s


async def test_operating_anchor_sleeps_through_its_reboot(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, reports_power_state=True, reboot_s=3.0)

    entry, elapsed = await _power_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True
    assert anchor.power == "asleep"
    assert elapsed >= anchor.reboot_s


@pytest.mark.parametrize("reports_power_state", [False, True])
async def test_plain_standby_anchor_sleeps_in_place(
    dialect, builder, hub, autojump_clock, reports_power_state
):
    # plain standby reads SLEEP 3; writing 1 moves it to asleep in place
    anchor = FakeAnchor(
        dialect,
        standby=True,
        sleep_value=3,
        in_place_sleep=True,
        reports_power_state=reports_power_state,
    )

    entry, elapsed = await _power_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True
    # only the power state tells asleep from standby
    assert entry["sleeping"] is (True if reports_power_state else None)
    assert anchor.power == "asleep"
    assert anchor.reboot_at is None
    assert elapsed < anchor.reboot_s


async def test_plain_standby_is_not_asleep(dialect, builder, hub, autojump_clock):
    # a standby anchor that keeps reading 3 after the write refused the
    # sleep: standby is not asleep, even with STANDBY heartbeats
    anchor = FakeAnchor(dialect, standby=True, sleep_value=3)
    anchor.refuse_with = 3

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": False,
        "detail": "sleep refused by the anchor in its standby state",
    }
