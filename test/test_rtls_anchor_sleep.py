"""X-RTLS-SLEEP on anchors (rtls/anchor_sleep.py): a scripted anchor
reboots, falls asleep in place or refuses like the firmware's power path,
and the server reports what the anchor actually did."""

import time

import pytest
import trio
import trio.testing
from rtls_support import (
    DEVICE_SYSID,
    FakeDevice,
    StubApp,
    discover,
    make_message,
    set_fake_param,
)
from rtlslink.dialect import load_dialect
from rtlslink.protocol import PARAM_TYPE_UINT8, RtlsProtocol

from flockwave.server.ext.rtls.extension import (
    DEFAULT_DEVICE_TIMEOUT,
    POWER_STATES,
    RtlsExtension,
)
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


class FakeAnchor(FakeDevice):
    """Scripted anchor power path: a SLEEP change is acked, then the anchor
    reboots into the requested state, off the network for ``reboot_s``
    seconds, like rtls-link-zephyr's anchor (docs/reference/
    anchor-power-path.md). ``refuse_with`` makes it refuse instead,
    reading back as that SLEEP value; ``hang`` accepts and never reboots.
    With ``in_place_sleep`` a sleep request in plain standby moves it to
    asleep without a reboot; with ``reports_power_state`` it streams
    ``pwrst``."""

    def __init__(
        self,
        dialect,
        *,
        standby=False,
        power=None,
        reboot_s=9.0,
        in_place_sleep=False,
        reports_power_state=False,
        sleep_value=None,
    ):
        super().__init__(dialect)
        set_fake_param(self, "UWB_ROLE", 3, "uint8")
        if sleep_value is None:
            sleep_value = 1 if standby else 0
        self.params["SLEEP"] = (bytes([sleep_value]), PARAM_TYPE_UINT8)
        self.power = power or ("standby" if standby else "operating")
        self.in_place_sleep = in_place_sleep
        self.reports_power_state = reports_power_state
        self.reboot_s = reboot_s
        self.boot_at = -100.0
        self.reboot_at = None
        self.target = None
        self.refuse_with = None
        self.hang = False
        self.expired = False

    #: heartbeat EMERGENCY after the next reboot, like firmware safe mode
    safe_mode_after_reboot = False
    safe_mode = False

    @property
    def sleeping(self) -> bool:
        return self.power in ("standby", "asleep", "off")

    def heartbeat(self) -> bytes:
        if not self.safe_mode:
            return super().heartbeat()
        d = self.dialect
        message = d.MAVLink_heartbeat_message(
            type=d.MAV_TYPE_GENERIC,
            autopilot=d.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=d.MAV_STATE_EMERGENCY,
            mavlink_version=3,
        )
        return bytes(message.pack(self._mav))

    @property
    def down(self) -> bool:
        return self.reboot_at is not None and trio.current_time() >= self.reboot_at

    def uptime_ms(self) -> int:
        # a booted board reports a few ms at least (TIMESYNC tc1 0 is no reply)
        return 2 + int((trio.current_time() - self.boot_at) * 1000)

    def handle(self, data: bytes) -> list[bytes]:
        return [] if self.down else super().handle(data)

    def after_sleep_write(self, previous: bytes, value: bytes) -> None:
        asleep = value == b"\x01"
        if self.refuse_with is not None:
            self.params["SLEEP"] = (bytes([self.refuse_with]), PARAM_TYPE_UINT8)
        elif asleep and self.power == "standby" and self.in_place_sleep:
            self.power = "asleep"
        elif value != previous and not self.hang:
            self.target = "asleep" if asleep else "operating"
            self.reboot_at = trio.current_time() + 0.5

    async def run(self, extension):
        """Heartbeats once a second while up; the server loop's expiry is
        emulated once the reboot outlasts the device timeout."""
        while True:
            now = trio.current_time()
            if self.down:
                if not self.expired and now - self.reboot_at > DEFAULT_DEVICE_TIMEOUT:
                    self.expired = True
                    extension._expire_devices(time.monotonic() + 1000.0)
                if now >= self.reboot_at + self.reboot_s:
                    self.power, self.boot_at = self.target, now
                    self.reboot_at = None
                    self.safe_mode = self.safe_mode_after_reboot
                    if self.power == "off":
                        self.params["SLEEP"] = (bytes([2]), PARAM_TYPE_UINT8)
            if not self.down:
                await extension._process_datagram(
                    self.heartbeat(), self.address, time.monotonic()
                )
                if self.reports_power_state:
                    code = POWER_STATES.index(self.power)
                    await extension._process_datagram(
                        self.named_value_float("pwrst", float(code)),
                        self.address,
                        time.monotonic(),
                    )
            await trio.sleep(1.0)


async def _anchor_extension(dialect, anchor):
    """An extension wired to ``anchor`` that has discovered it."""
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[anchor.address])
    ext.app = StubApp()

    async def fake_send(payload, address):
        for reply in anchor.handle(payload):
            await ext._process_datagram(reply, anchor.address, time.monotonic())

    ext._send = fake_send
    await discover(ext, anchor)
    return ext


async def _send_sleep(ext, builder, hub, sleeping, **body):
    response = await ext._handle_RTLS_SLEEP(
        make_message(
            builder,
            {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": sleeping}
            | body,
        ),
        None,
        hub,
    )
    return response.body["result"][str(DEVICE_SYSID)]


async def _anchor_sleep(dialect, builder, hub, anchor, sleeping, **body):
    """Discover the anchor, run it, and send it one X-RTLS-SLEEP; returns the
    extension and the anchor's result entry."""
    ext = await _anchor_extension(dialect, anchor)
    async with trio.open_nursery() as nursery:
        nursery.start_soon(anchor.run, ext)
        entry = await _send_sleep(ext, builder, hub, sleeping, **body)
        nursery.cancel_scope.cancel()
    return ext, entry


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


async def test_anchor_already_asleep_by_power_state(
    dialect, builder, hub, autojump_clock
):
    anchor = FakeAnchor(dialect, power="asleep", standby=True, reports_power_state=True)

    entry, _ = await _power_sleep(dialect, builder, hub, anchor, True)

    assert entry["accepted"] is True and entry["sleeping"] is True


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


async def _power_sleep(dialect, builder, hub, anchor, sleeping, **body):
    """Like _anchor_sleep, once the anchor has reported its powerState."""
    ext = await _anchor_extension(dialect, anchor)
    async with trio.open_nursery() as nursery:
        nursery.start_soon(anchor.run, ext)
        await trio.sleep(1.5)
        if anchor.reports_power_state:
            assert ext._power_state(DEVICE_SYSID) == anchor.power
        started = trio.current_time()
        entry = await _send_sleep(ext, builder, hub, sleeping, **body)
        elapsed = trio.current_time() - started
        nursery.cancel_scope.cancel()
    return entry, elapsed


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
