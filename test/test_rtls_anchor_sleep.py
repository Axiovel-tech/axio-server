"""X-RTLS-SLEEP on anchors (rtls/anchor_sleep.py) judged from heartbeats
and SLEEP readbacks: firmware without ``pwrst``, reboots, refusals and the
deadline."""

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
from rtlslink.protocol import PARAM_TYPE_UINT8, RtlsProtocol

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
    started = trio.current_time()

    ext, entry = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": True,
        "sleeping": True,
        "detail": "asleep",
    }
    # confirmed only after the reboot: off the network, lost, rediscovered
    assert anchor.expired
    assert trio.current_time() - started >= anchor.reboot_s
    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is True


async def test_anchor_wake_confirmed_by_the_rewound_uptime(
    dialect, builder, hub, autojump_clock
):
    # a reboot shorter than the device timeout: never lost, the uptime
    # probe shows the reboot
    anchor = FakeAnchor(dialect, standby=True, reboot_s=3.0)

    ext, entry = await _anchor_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is True
    assert entry["sleeping"] is False
    assert not anchor.expired
    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is False


async def test_anchor_wake_refused_on_a_low_pack(dialect, builder, hub, autojump_clock):
    anchor = FakeAnchor(dialect, standby=True)
    anchor.refuse_with = 1

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is False
    assert entry["sleeping"] is True
    assert "pack is low or has no fresh voltage" in entry["detail"]
    assert "arming" not in entry["detail"]


async def test_anchor_off_only_wakes_from_its_button(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, standby=True)
    anchor.params["SLEEP"] = (bytes([2]), PARAM_TYPE_UINT8)
    anchor.refuse_with = 2

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, False)

    assert entry["accepted"] is False
    assert "only its button" in entry["detail"]


async def test_anchor_already_asleep_is_unverified_without_power_state(
    dialect, builder, hub, autojump_clock
):
    # without pwrst, an anchor in standby that reads SLEEP 1 may be asleep or
    # in plain standby (older firmware): never reported asleep
    anchor = FakeAnchor(dialect, standby=True)

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True
    assert entry["sleeping"] is None
    assert "unverified" in entry["detail"]
    assert anchor.reboot_at is None and not anchor.expired


async def test_anchor_sleep_not_confirmed_in_time(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect)
    anchor.hang = True

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, True, timeout=8)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "the anchor accepted the request but was not seen asleep "
        "within 8 s (it may still be rebooting)",
    }


async def test_anchor_without_sleep_mode(dialect, builder, hub, autojump_clock):
    # an anchor without a power path (native_sim, boards without one) has
    # no SLEEP parameter: say so instead of timing out on the write
    anchor = FakeAnchor(dialect)
    del anchor.params["SLEEP"]

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "the anchor has no sleep mode (no SLEEP parameter)",
    }


async def test_anchor_wake_right_after_a_sleep(dialect, builder, hub, autojump_clock):
    """The uptime sampled before the wake is a few ms past the sleep's own
    reboot, so the wake's reboot does not lower it: the reboot shows as
    uptime that advanced less than the host time since that sample. The
    second reboot is shorter than the device timeout (no rediscovery)."""
    anchor = FakeAnchor(dialect)
    ext = await _anchor_extension(dialect, anchor)
    async with trio.open_nursery() as nursery:
        nursery.start_soon(anchor.run, ext)
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


async def test_safe_mode_after_a_wake_is_not_awake(
    dialect, builder, hub, autojump_clock
):
    # after the wake's reboot the firmware sits in safe mode (EMERGENCY): it
    # is neither STANDBY nor ACTIVE and must not confirm "awake"
    anchor = FakeAnchor(dialect, standby=True, reboot_s=3.0)
    anchor.safe_mode_after_reboot = True

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, False, timeout=12)

    assert entry["accepted"] is False
    assert entry["sleeping"] is None
    assert "not seen awake" in entry["detail"]


async def test_sleep_landing_off_is_refused_after_the_reboot(
    dialect, builder, hub, autojump_clock
):
    # STANDBY after a sleep's reboot also covers off: the readback after the
    # reboot tells (older firmware, no pwrst)
    anchor = FakeAnchor(dialect, reboot_s=3.0)
    original = anchor.after_sleep_write

    def lands_off(previous, value):
        original(previous, value)
        anchor.target = "off"

    anchor.after_sleep_write = lands_off

    _, entry = await _anchor_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is False
    assert entry["sleeping"] is False
    assert "only its button" in entry["detail"]


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
