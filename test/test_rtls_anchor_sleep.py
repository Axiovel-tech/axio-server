"""X-RTLS-SLEEP on anchors (rtls/anchor_sleep.py) judged by their
``powerState``: reboots into and out of operating, sleep in place from
standby, refusals read back from SLEEP, and the deadline."""

import time

import pytest
import trio
import trio.testing
from anchor_sleep_support import (
    FakeAnchor,
    _anchor_extension,
    _anchor_sleep,
    _send_sleep,
)
from rtls_support import DEVICE_SYSID, StubApp, discover
from rtlslink.dialect import load_dialect
from rtlslink.protocol import RtlsProtocol

from flockwave.server.ext.rtls.extension import RtlsExtension
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


async def test_anchor_sleep_waits_for_the_reboot(dialect, builder, hub, autojump_clock):
    anchor = FakeAnchor(dialect)

    ext, entry, elapsed = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": True,
        "sleeping": True,
        "detail": "asleep",
    }
    # confirmed only after the reboot: off the network, lost, rediscovered
    assert anchor.expired
    assert elapsed >= anchor.reboot_s
    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is True


async def test_operating_anchor_sleeps_through_its_reboot(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, reboot_s=3.0)

    _, entry, elapsed = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True
    assert anchor.power == "asleep"
    assert elapsed >= anchor.reboot_s


async def test_standby_anchor_wakes_through_its_reboot(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, standby=True)

    ext, entry, elapsed = await _anchor_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is True
    assert entry["sleeping"] is False
    assert anchor.power == "operating"
    # confirmed only after the reboot, not on a pre-reboot report
    assert anchor.expired
    assert elapsed >= anchor.reboot_s
    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is False


async def test_anchor_already_asleep(dialect, builder, hub, autojump_clock):
    anchor = FakeAnchor(dialect, power="asleep", standby=True)

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True and entry["sleeping"] is True


async def test_plain_standby_anchor_sleeps_in_place(
    dialect, builder, hub, autojump_clock
):
    # plain standby reads SLEEP 3; writing 1 moves it to asleep in place
    anchor = FakeAnchor(dialect, standby=True, sleep_value=3, in_place_sleep=True)

    _, entry, elapsed = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": True,
        "sleeping": True,
        "detail": "asleep",
    }
    assert anchor.power == "asleep"
    assert anchor.reboot_at is None and not anchor.expired
    assert elapsed < anchor.reboot_s


async def test_sleep_confirmed_through_booting(dialect, builder, hub, autojump_clock):
    # back on the network but still booting, the anchor reads SLEEP 0: no refusal
    anchor = FakeAnchor(dialect, boot_s=3.0)

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True and entry["sleeping"] is True
    assert anchor.power == "asleep"


async def test_standby_anchor_is_not_reported_asleep_while_in_standby(
    dialect, builder, hub, autojump_clock
):
    # an anchor that stays in standby after SLEEP=1 is not reported asleep
    anchor = FakeAnchor(dialect, standby=True)

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True, timeout=6)

    assert entry["accepted"] is False
    assert entry["sleeping"] is None
    assert "not seen asleep" in entry["detail"]


async def test_plain_standby_is_not_asleep(dialect, builder, hub, autojump_clock):
    # a standby anchor that keeps reading 3 after the write refused the sleep
    anchor = FakeAnchor(dialect, standby=True, sleep_value=3)
    anchor.refuse_with = 3

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": False,
        "detail": "sleep refused by the anchor in its standby state",
    }


async def test_anchor_wake_refused_on_a_low_pack(dialect, builder, hub, autojump_clock):
    anchor = FakeAnchor(dialect, standby=True)
    anchor.refuse_with = 1

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is False
    assert entry["sleeping"] is True
    assert "pack is low or has no fresh voltage" in entry["detail"]
    assert "arming" not in entry["detail"]


async def test_anchor_off_only_wakes_from_its_button(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, power="off", standby=True, sleep_value=2)
    anchor.refuse_with = 2

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is False
    assert "only its button" in entry["detail"]


async def test_anchor_sleep_not_confirmed_in_time(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect)
    anchor.hang = True

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True, timeout=8)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "the anchor accepted the request but was not seen asleep "
        "within 8 s (it may still be rebooting)",
    }


async def test_anchor_without_power_state_is_not_commanded(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, reports_power_state=False)

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "the anchor does not report its power state; update its firmware",
    }
    assert anchor.params["SLEEP"][0] == bytes([0])  # nothing was written


async def test_anchor_without_sleep_mode(dialect, builder, hub, autojump_clock):
    # an anchor without a power path (native_sim, boards without one) has
    # no SLEEP parameter: say so instead of timing out on the write
    anchor = FakeAnchor(dialect)
    del anchor.params["SLEEP"]

    _, entry, _ = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "the anchor has no sleep mode (no SLEEP parameter)",
    }


async def test_anchor_wake_right_after_a_sleep(dialect, builder, hub, autojump_clock):
    """A wake sent as soon as a sleep is confirmed waits for its own reboot;
    the second reboot is shorter than the device timeout (no rediscovery)."""
    anchor = FakeAnchor(dialect)
    ext = await _anchor_extension(dialect, anchor)
    async with trio.open_nursery() as nursery:
        nursery.start_soon(anchor.run, ext)
        await trio.sleep(1.5)
        slept = await _send_sleep(ext, builder, hub, True)
        anchor.reboot_s = 3.0
        woke = await _send_sleep(ext, builder, hub, False)
        nursery.cancel_scope.cancel()

    assert slept["accepted"] is True and slept["sleeping"] is True
    assert woke == {
        "requested": False,
        "accepted": True,
        "sleeping": False,
        "detail": "awake",
    }
    assert anchor.power == "operating"


async def test_role_unknown_is_not_commanded(dialect, builder, hub, autojump_clock):
    """A device whose UWB_ROLE is neither cached nor read in time is not
    treated as a tag: an anchor refusing a wake would read as success."""
    anchor = FakeAnchor(dialect, standby=True)
    anchor.respond_to_list = False
    anchor.respond_to_read = False
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[anchor.address])
    ext.app = StubApp()

    async def fake_send(payload, address):
        for reply in anchor.handle(payload):
            await ext._process_datagram(reply, anchor.address, time.monotonic())

    ext._send = fake_send
    await discover(ext, anchor)

    entry = await _send_sleep(ext, builder, hub, False)

    assert entry == {
        "requested": False,
        "accepted": False,
        "sleeping": None,
        "detail": f"Device {DEVICE_SYSID}: role unknown (UWB_ROLE not read); "
        "not commanded",
    }
    assert anchor.params["SLEEP"][0] == bytes([1])  # nothing was written
