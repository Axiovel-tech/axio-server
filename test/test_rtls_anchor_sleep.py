"""X-RTLS-SLEEP on anchors (rtls/anchor_sleep.py): the SLEEP write's
acknowledgement is the answer, and ``sleeping`` in X-RTLS-INF comes from the
anchor's ``powerState`` only."""

import time

import pytest
import trio
from rtls_support import (
    DEVICE_SYSID,
    FakeDevice,
    StubApp,
    discover,
    make_message,
    set_fake_param,
)
from rtlslink.dialect import load_dialect
from rtlslink.protocol import ANCHOR_POWER_STATES, PARAM_ACK_FAILED, RtlsProtocol

from flockwave.server.ext.rtls.extension import RtlsExtension
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

#: pwrst code per power state name
POWER_CODES = {name: code for code, name in ANCHOR_POWER_STATES.items()}


@pytest.fixture(scope="module")
def dialect():
    return load_dialect()


@pytest.fixture
def hub():
    return MessageHub()


@pytest.fixture
def builder():
    return FlockwaveMessageBuilder()


class FakeAnchor(FakeDevice):
    """An anchor in power state ``power`` that refuses SLEEP writes with
    PARAM_ACK_FAILED when ``refuse`` is set, like the firmware's write gate."""

    def __init__(self, dialect, power="operating", *, refuse=False):
        super().__init__(dialect)
        set_fake_param(self, "UWB_ROLE", 3, "uint8")
        self.power = power
        self.set_result = PARAM_ACK_FAILED if refuse else 0

    @property
    def sleeping(self) -> bool:
        return self.power in ("standby", "asleep", "off")

    async def report(self, ext, **stats: float) -> None:
        """A heartbeat and the health stats, pwrst first."""
        stats = {"pwrst": float(POWER_CODES[self.power]), **stats}
        for packet in (
            self.heartbeat(),
            *(self.named_value_float(name, value) for name, value in stats.items()),
        ):
            await ext._process_datagram(packet, self.address, time.monotonic())


async def _extension(dialect, anchor, **stats):
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[anchor.address])
    ext.app = StubApp()

    async def fake_send(payload, address):
        for reply in anchor.handle(payload):
            await ext._process_datagram(reply, anchor.address, time.monotonic())

    ext._send = fake_send
    await discover(ext, anchor)
    await anchor.report(ext, **stats)
    return ext


async def _sleep(ext, builder, hub, sleeping, **body):
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


@pytest.mark.parametrize(
    ("power", "sleeping", "detail"),
    [("operating", True, "sleep accepted"), ("asleep", False, "wake accepted")],
)
async def test_an_accepted_write_answers_from_the_ack(
    dialect, builder, hub, power, sleeping, detail
):
    """The ack is the answer: no wait for the transition, no SLEEP readback."""
    anchor = FakeAnchor(dialect, power)
    ext = await _extension(dialect, anchor)
    anchor.read_requests.clear()

    with trio.fail_after(1):
        entry = await _sleep(ext, builder, hub, sleeping)

    assert entry == {
        "requested": sleeping,
        "accepted": True,
        "sleeping": None,
        "detail": detail,
    }
    assert anchor.params["SLEEP"][0] == bytes([int(sleeping)])
    assert "SLEEP" not in anchor.read_requests


@pytest.mark.parametrize(
    ("power", "sleeping", "stats", "detail"),
    [
        ("preview", True, {}, "sleep refused by the anchor in its preview state"),
        ("operating", False, {}, "wake refused by the anchor in its operating state"),
        (
            "asleep",
            False,
            {"pwrin": 1, "vbat": 3.25, "lowbat": 1},
            "wake refused by the anchor: its pack is low or has no fresh "
            "voltage reading (last 3.25 V)",
        ),
        (
            "asleep",
            False,
            {"pwrin": 1},
            "wake refused by the anchor: its pack is low or has no fresh "
            "voltage reading",
        ),
        # the pack cannot be the reason: on PoE (a second wake while the
        # first one's reboot is pending), or a healthy pack
        (
            "standby",
            False,
            {"pwrin": 5, "vbat": 4.10},
            "wake refused by the anchor in its standby state",
        ),
        (
            "asleep",
            False,
            {"pwrin": 1, "vbat": 4.10, "lowbat": 0},
            "wake refused by the anchor in its asleep state",
        ),
        (
            "off",
            False,
            {},
            "refused: the anchor is off; only its button starts it again",
        ),
    ],
)
async def test_a_refusal_in_the_ack_says_why(
    dialect, builder, hub, power, sleeping, stats, detail
):
    anchor = FakeAnchor(dialect, power, refuse=True)
    ext = await _extension(dialect, anchor, **stats)

    entry = await _sleep(ext, builder, hub, sleeping)

    assert entry == {
        "requested": sleeping,
        "accepted": False,
        "sleeping": None,
        "detail": detail,
    }


async def test_another_rejection_reports_its_code(dialect, builder, hub):
    anchor = FakeAnchor(dialect)
    anchor.set_result = 1  # PARAM_ACK_VALUE_UNSUPPORTED
    ext = await _extension(dialect, anchor)

    entry = await _sleep(ext, builder, hub, True)

    assert entry["accepted"] is False
    assert entry["detail"] == "parameter write rejected (code 1)"


async def test_an_unanswered_write_times_out(dialect, builder, hub):
    anchor = FakeAnchor(dialect)
    ext = await _extension(dialect, anchor)
    anchor.respond_to_set = False

    entry = await _sleep(ext, builder, hub, True, timeout=0.2)

    assert entry["accepted"] is False
    assert entry["detail"] == f"Timeout while waiting for device {DEVICE_SYSID}"


async def test_anchor_without_sleep_mode(dialect, builder, hub):
    # an anchor without a power path has no SLEEP parameter: say so instead
    # of reporting its refusal
    anchor = FakeAnchor(dialect)
    del anchor.params["SLEEP"]
    ext = await _extension(dialect, anchor)

    entry = await _sleep(ext, builder, hub, True)

    assert entry == {
        "requested": True,
        "accepted": False,
        "sleeping": None,
        "detail": "the anchor has no sleep mode (no SLEEP parameter)",
    }


@pytest.mark.parametrize(
    ("power", "asleep"),
    [("asleep", True), ("standby", False), ("off", False), ("operating", False)],
)
async def test_inf_sleeping_comes_from_the_power_state(dialect, power, asleep):
    """An anchor in standby or off heartbeats STANDBY like an asleep one:
    only its powerState says it is asleep."""
    anchor = FakeAnchor(dialect, power)
    ext = await _extension(dialect, anchor)

    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is asleep


async def test_falling_asleep_in_place_pushes_inf(dialect):
    """Standby and asleep heartbeat the same STANDBY: the powerState change
    alone must push X-RTLS-INF, not the 10 s refresh."""
    anchor = FakeAnchor(dialect, "standby")
    ext = await _extension(dialect, anchor)
    ext._inf_pending = False
    await anchor.report(ext)
    assert ext._inf_pending is False

    anchor.power = "asleep"
    await anchor.report(ext)

    assert ext._inf_pending is True
    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is True


async def test_an_expired_write_is_never_sent(dialect, builder, hub):
    """A second request queued behind the first one's parameter write lock
    for longer than its timeout times out without sending its write."""
    anchor = FakeAnchor(dialect, "standby")
    ext = await _extension(dialect, anchor)
    lock = ext._param_write_locks.setdefault((DEVICE_SYSID, "SLEEP"), trio.Lock())
    writes = []
    parser = dialect.MAVLink(None)
    handle = anchor.handle

    def recording(payload):
        writes.extend(
            m
            for m in parser.parse_buffer(payload) or []
            if m.get_type() == "PARAM_EXT_SET"
        )
        return handle(payload)

    anchor.handle = recording
    with trio.fail_after(1):
        async with lock:
            entry = await _sleep(ext, builder, hub, False, timeout=0.05)

    assert entry["accepted"] is False
    assert entry["detail"] == f"Timeout while waiting for device {DEVICE_SYSID}"
    assert writes == []


async def test_inf_anchor_without_power_state_is_not_asleep(dialect):
    anchor = FakeAnchor(dialect, "standby")
    ext = await _extension(dialect, anchor)
    del ext._stats[DEVICE_SYSID]["powerState"]

    assert ext._inf_status(time.monotonic())[str(DEVICE_SYSID)]["sleeping"] is False


async def test_role_unknown_is_not_commanded(dialect, builder, hub):
    """A device whose UWB_ROLE is neither cached nor read in time is not
    treated as a tag: an anchor's ack would read as a verified wake."""
    anchor = FakeAnchor(dialect, "asleep")
    anchor.respond_to_list = False
    anchor.respond_to_read = False
    ext = await _extension(dialect, anchor)

    entry = await _sleep(ext, builder, hub, False, timeout=0.2)

    assert entry == {
        "requested": False,
        "accepted": False,
        "sleeping": None,
        "detail": f"Device {DEVICE_SYSID}: role unknown (UWB_ROLE not read); "
        "not commanded",
    }
    assert anchor.params["SLEEP"][0] == bytes([0])  # nothing was written
