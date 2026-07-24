"""Tests for the client message API of the rtls extension.

The protocol core itself (and its value codec) is covered by the test
suite of the standalone ``rtls-link`` SDK; here the message handlers
are exercised against that sans-IO core with a fake transport: the
extension's ``_send`` is replaced by a function that hands every
outbound datagram to a scripted fake device, whose MAVLink replies are
pushed back through ``_process_datagram``. No sockets are involved.
"""

import logging
import socket
import struct
import sys
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
import rtlslink as rtlslink_module
import trio
import trio.testing
from rtlslink.dialect import load_dialect
from rtlslink.protocol import (
    PARAM_ACK_FAILED,
    PARAM_ACK_VALUE_UNSUPPORTED,
    PARAM_TYPE_CUSTOM,
    PARAM_TYPE_INT32,
    PARAM_TYPE_REAL32,
    PARAM_TYPE_UINT8,
    RTLS_COMPONENT_ID,
    ProtocolEvent,
    RtlsProtocol,
    encode_param_value,
    param_type_from_name,
    raw_field_bytes,
)

from flockwave.server.ext.rtls.extension import (
    REFILL_INITIAL_DELAY,
    REFILL_MAX_ATTEMPTS,
    REFILL_READ_SPACING,
    SLEEP_PIN_TIMEOUT,
    RtlsExtension,
    _configured_addresses,
    _load_advertisement_parser,
    _presence_config,
    _stats_json,
)
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

DEVICE_SYSID = 42
DEVICE_ADDRESS = ("192.168.4.42", 3333)


class FakeDevice:
    """Scripted rtls-link device: parses the datagrams the server sends
    and produces the MAVLink replies a real firmware would."""

    def __init__(self, dialect, system_id: int = DEVICE_SYSID):
        self.dialect = dialect
        self.system_id = system_id
        self.address = DEVICE_ADDRESS
        self._mav = dialect.MAVLink(
            None, srcSystem=system_id, srcComponent=RTLS_COMPONENT_ID
        )
        self._parser = dialect.MAVLink(None)
        self._parser.robust_parsing = True

        # name -> (raw value bytes, MAV_PARAM_EXT_TYPE code)
        self.params: dict[str, tuple[bytes, int]] = {
            "MAV_SYS_ID": (bytes([system_id]), PARAM_TYPE_UINT8),
            "UWB_CH": (struct.pack("<i", 5), PARAM_TYPE_INT32),
            "POS_X": (struct.pack("<f", 1.5), PARAM_TYPE_REAL32),
            "FW_VERSION": (b"1.2.3", PARAM_TYPE_CUSTOM),
            "SLEEP": (bytes([0]), PARAM_TYPE_UINT8),
        }

        self.respond_to_list = True
        self.respond_to_read = True
        #: parameter names omitted from full-list responses, simulating
        #: the datagrams a real dump loses to UDP under the startup burst
        self.drop_from_list: set[str] = set()
        #: every parameter name the server asked for with a targeted
        #: PARAM_EXT_REQUEST_READ, in arrival order (recorded even while
        #: ``respond_to_read`` is off)
        self.read_requests: list[str] = []
        self.respond_to_set = True
        self.set_result = 0  # PARAM_ACK_ACCEPTED
        #: emulate the firmware's arming gate: a SLEEP=1 write is acked
        #: but flipped back to 0, as the real device does while armed
        self.refuse_sleep = False

    @property
    def sleeping(self) -> bool:
        return self.params["SLEEP"][0][:1] == b"\x01"

    def heartbeat(self) -> bytes:
        d = self.dialect
        # a sleeping device advertises STANDBY, like the firmware
        status = d.MAV_STATE_STANDBY if self.sleeping else d.MAV_STATE_ACTIVE
        message = d.MAVLink_heartbeat_message(
            type=d.MAV_TYPE_GENERIC,
            autopilot=d.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=status,
            mavlink_version=3,
        )
        return bytes(message.pack(self._mav))

    def named_value_float(
        self, name: str, value: float, time_boot_ms: int = 0
    ) -> bytes:
        """One NAMED_VALUE_FLOAT, as the firmware emits per health stat."""
        message = self.dialect.MAVLink_named_value_float_message(
            time_boot_ms=time_boot_ms,
            name=name.encode(),
            value=value,
        )
        return bytes(message.pack(self._mav))

    def handle(self, data: bytes) -> list[bytes]:
        out: list[bytes] = []
        for message in self._parser.parse_buffer(data) or []:
            kind = message.get_type()
            if kind == "PARAM_EXT_REQUEST_LIST" and self.respond_to_list:
                for index, name in enumerate(self.params):
                    if name in self.drop_from_list:
                        continue
                    out.append(self._param_value(name, index))
            elif kind == "PARAM_EXT_REQUEST_READ":
                name = message.param_id
                self.read_requests.append(name)
                if self.respond_to_read and name in self.params:
                    index = list(self.params).index(name)
                    out.append(self._param_value(name, index))
            elif kind == "PARAM_EXT_SET" and self.respond_to_set:
                name = message.param_id
                value = raw_field_bytes(message, "param_value") or b""
                if self.set_result == 0 and name in self.params:
                    width = len(self.params[name][0])
                    self.params[name] = (value[:width], message.param_type)
                out.append(
                    self._param_ack(name, value, message.param_type, self.set_result)
                )
                if name == "SLEEP" and self.refuse_sleep and self.sleeping:
                    # the arming gate runs after the ack on the firmware
                    self.params["SLEEP"] = (bytes([0]), PARAM_TYPE_UINT8)
        return out

    def _param_value(self, name: str, index: int) -> bytes:
        value, param_type = self.params[name]
        message = self.dialect.MAVLink_param_ext_value_message(
            param_id=name.encode(),
            param_value=value,
            param_type=param_type,
            param_count=len(self.params),
            param_index=index,
        )
        return bytes(message.pack(self._mav))

    def _param_ack(
        self, name: str, value: bytes, param_type: int, result: int
    ) -> bytes:
        message = self.dialect.MAVLink_param_ext_ack_message(
            param_id=name.encode(),
            param_value=value,
            param_type=param_type,
            param_result=result,
        )
        return bytes(message.pack(self._mav))


class StubMessageHub:
    """Captures the notifications that the extension broadcasts."""

    def __init__(self):
        self.broadcasts = []
        #: when set, enqueue_broadcast_message emulates a full TX queue by
        #: dropping the notification, like the real hub does on overflow
        self.full = False

    def create_notification(self, body=None):
        return body

    async def broadcast_message(self, message):
        self.broadcasts.append(message)

    def enqueue_broadcast_message(self, message):
        # the real hub's send_nowait drops the message when the queue is
        # full instead of blocking the caller
        if not self.full:
            self.broadcasts.append(message)


class StubApp:
    def __init__(self):
        self.message_hub = StubMessageHub()


class StubBeaconBasicProperties:
    def __init__(self, name=""):
        self.name = name


class StubBeacon:
    def __init__(self, id):
        self.id = id
        self.basic_properties = StubBeaconBasicProperties()
        self.position = None
        self.active = False

    def update_status(self, position=None, heading=None, active=None):
        if position is not None:
            self.position = position
        if active is not None:
            self.active = bool(active)


class StubBeaconAPI:
    """Stands in for the beacon-registry API the extension imports: ``use``
    yields a beacon and removes it again when its context exits, mirroring the
    real registry-backed context manager."""

    def __init__(self):
        self.beacons = {}

    @contextmanager
    def use(self, beacon_id):
        beacon = StubBeacon(beacon_id)
        self.beacons[beacon_id] = beacon
        try:
            yield beacon
        finally:
            self.beacons.pop(beacon_id, None)


@pytest.fixture(scope="module")
def dialect():
    return load_dialect()


@pytest.fixture
def device(dialect):
    return FakeDevice(dialect)


@pytest.fixture
def hub():
    return MessageHub()


@pytest.fixture
def builder():
    return FlockwaveMessageBuilder()


@pytest.fixture
def extension(dialect, device):
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[device.address])
    ext.app = StubApp()

    async def fake_send(payload, address):
        for reply in device.handle(payload):
            await ext._process_datagram(reply, device.address, time.monotonic())

    ext._send = fake_send  # fake transport: loops straight into the device
    return ext


async def discover(extension, device):
    """Feeds a device heartbeat into the extension; the extension then
    auto-fetches the parameter list, exactly as the run loop would."""
    await extension._process_datagram(
        device.heartbeat(), device.address, time.monotonic()
    )


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


def make_message(builder, body):
    return builder.create_message(body)


async def test_run_without_app_fails_fast(dialect):
    """A standalone harness that forgets ext.app must fail loudly at
    run() instead of silently dropping every stats/OTA broadcast (a real
    e2e-harness bug this guard exists for)."""
    ext = RtlsExtension()
    assert ext.app is None
    with pytest.raises(RuntimeError, match="without an app"):
        await ext.run(None, {}, None)


def set_fake_param(device, name, value, param_type_name):
    """Write a parameter into a FakeDevice's wire store (raw bytes + type)."""
    param_type = param_type_from_name(param_type_name)
    device.params[name] = (encode_param_value(value, param_type), param_type)


def set_cached_param(device, name, value, param_type_name):
    """Write a parameter into a bare device's server-side cache (a
    SimpleNamespace standing in for an RtlsDevice)."""
    param_type = param_type_from_name(param_type_name)
    device.params[name] = encode_param_value(value, param_type)
    device.param_types[name] = param_type


def add_rtls_cell_params(device, *, role=1, uwb_mac=254):
    """Give a FakeDevice a full RTLS cell: a role, its own MAC, the cell
    origin and a two-anchor NED table (A1 carries a survey bias)."""
    for name, value, param_type in (
        ("UWB_ROLE", role, "uint8"),
        ("UWB_MAC", uwb_mac, "uint16"),
        ("ORIGIN_LAT_E7", 413900000, "int32"),
        ("ORIGIN_LON_E7", 21500000, "int32"),
        ("ORIGIN_ALT_MM", 10000, "int32"),
        ("UWB_AN_COUNT", 2, "uint8"),
        ("UWB_AN0_X", -10.0, "real32"),
        ("UWB_AN0_Y", -10.0, "real32"),
        ("UWB_AN0_Z", 0.0, "real32"),
        ("UWB_AN0_MAC", 1, "uint16"),
        ("UWB_AN0_BIAS_M", 0.0, "real32"),
        ("UWB_AN1_X", 10.0, "real32"),
        ("UWB_AN1_Y", 10.0, "real32"),
        ("UWB_AN1_Z", -4.8, "real32"),
        ("UWB_AN1_MAC", 2, "uint16"),
        ("UWB_AN1_BIAS_M", -0.3, "real32"),
    ):
        set_fake_param(device, name, value, param_type)


def add_anchor_device(extension, system_id, *, role, uwb_mac):
    """Register a bare anchor device (role + MAC only) directly in the
    protocol's device table, as discovery + param-listing would."""
    anchor = SimpleNamespace(
        system_id=system_id,
        address=("192.168.4.%d" % (system_id % 250), 3333),
        last_seen=time.monotonic(),
        params={},
        param_types={},
        param_count=None,
    )
    set_cached_param(anchor, "UWB_ROLE", role, "uint8")
    set_cached_param(anchor, "UWB_MAC", uwb_mac, "uint16")
    anchor.param_count = len(anchor.params)
    extension._protocol.devices[system_id] = anchor
    return anchor


# ---- X-RTLS-INF ---------------------------------------------------------


async def test_inf_empty_before_discovery(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    assert response.body == {"type": "X-RTLS-INF", "status": {}, "anchors": []}


async def test_inf_lists_discovered_devices(extension, device, builder, hub):
    await discover(extension, device)

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)

    assert response.body["type"] == "X-RTLS-INF"
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["id"] == DEVICE_SYSID
    assert entry["address"] == list(DEVICE_ADDRESS)
    assert entry["age"] >= 0
    # the parameter list is auto-fetched on discovery, so the firmware
    # version and the parameter count are already known
    assert entry["firmwareVersion"] == "1.2.3"
    assert entry["paramCount"] == len(device.params)
    assert entry["otaStatus"] is None
    assert entry["sleeping"] is False
    # a device without RTLS params carries neither a role nor a name
    assert "role" not in entry
    assert "name" not in entry


# ---- role / site anchors / beacons --------------------------------------


async def test_inf_includes_rtls_role_and_name(extension, device, builder, hub):
    add_rtls_cell_params(device, role=2, uwb_mac=1)
    await discover(extension, device)

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)

    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["role"] == "anchor-initiator"
    assert entry["name"] == "RTLS anchor A0"


async def test_inf_site_anchor_list(extension, device, builder, hub):
    add_rtls_cell_params(device)  # tag, role=1
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)

    anchors = {a["id"]: a for a in response.body["anchors"]}
    assert set(anchors) == {
        "rtls::default::anchor_0",
        "rtls::default::anchor_1",
    }
    a1 = anchors["rtls::default::anchor_1"]
    assert a1["cell"] == "default"
    assert a1["index"] == 1
    assert a1["mac"] == 2
    # no live anchor device online yet
    assert a1["active"] is False
    # origin + NED projected to global
    assert a1["position"]["lat"] == pytest.approx(41.3900898)
    assert a1["position"]["lon"] == pytest.approx(2.1501197)
    assert a1["position"]["amsl"] == pytest.approx(14.8)


async def test_inf_site_anchor_list_without_beacon_registration(
    extension, device, builder, hub
):
    # register_beacons:false leaves _beacon_api None, but the X-RTLS-INF site
    # anchors list must still mirror the tag's advertised cell geometry (the
    # flag only disables the beacon-layer publication, not the anchors list).
    add_rtls_cell_params(device)  # tag, role=1
    assert extension._beacon_api is None
    await discover(extension, device)

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)

    anchors = {a["id"]: a for a in response.body["anchors"]}
    assert set(anchors) == {
        "rtls::default::anchor_0",
        "rtls::default::anchor_1",
    }


async def test_site_anchor_active_when_anchor_device_online(extension, device, builder, hub):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)

    # anchor A1's MAC (2) comes online as a live responder
    add_anchor_device(extension, 102, role=3, uwb_mac=2)
    extension._refresh_anchor_cells()

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    anchors = {a["id"]: a for a in response.body["anchors"]}
    assert anchors["rtls::default::anchor_1"]["active"] is True
    assert anchors["rtls::default::anchor_0"]["active"] is False


async def test_param_cache_registers_configured_anchors_as_beacons(extension, device):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)

    api = extension._beacon_api
    assert set(api.beacons) == {
        "rtls::default::anchor_0",
        "rtls::default::anchor_1",
    }
    anchor1 = api.beacons["rtls::default::anchor_1"]
    assert anchor1.basic_properties.name == "RTLS A1"
    assert anchor1.active is False
    assert anchor1.position.lat == pytest.approx(41.3900898)
    assert anchor1.position.lon == pytest.approx(2.1501197)
    assert anchor1.position.amsl == pytest.approx(14.8)


async def test_anchor_beacon_active_reflects_associated_anchor_device(extension, device):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    anchor1 = extension._beacon_api.beacons["rtls::default::anchor_1"]
    assert anchor1.active is False

    anchor_device = add_anchor_device(extension, 102, role=3, uwb_mac=2)
    extension._refresh_anchor_cells()
    assert anchor1.active is True

    # the anchor process dies -> its beacon goes inactive again
    extension._protocol.devices.pop(anchor_device.system_id)
    extension._handle_lost(ProtocolEvent("lost", anchor_device.system_id))
    assert anchor1.active is False


async def test_invalid_role_does_not_register_anchor_beacons(extension, device):
    add_rtls_cell_params(device, role=99)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    assert extension._beacon_api.beacons == {}


async def test_complete_cell_without_role_registers_legacy_tag_beacons(extension, device):
    add_rtls_cell_params(device)
    device.params.pop("UWB_ROLE")
    device.param_count = len(device.params)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    assert "rtls::default::anchor_1" in extension._beacon_api.beacons


async def test_source_tag_role_change_drops_anchor_beacons(extension, device):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    assert "rtls::default::anchor_1" in extension._beacon_api.beacons

    # the tag is re-roled into an anchor: it no longer sources the cell
    result = await extension.set_param(DEVICE_SYSID, "UWB_ROLE", 3, "uint8")
    assert result["accepted"] is True
    assert extension._beacon_api.beacons == {}


async def test_lost_source_tag_drops_anchor_beacons(extension, device):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    assert "rtls::default::anchor_1" in extension._beacon_api.beacons

    events = extension._protocol.expire(time.monotonic() + 1000)
    assert [event.kind for event in events] == ["lost"]
    for event in events:
        extension._handle_lost(event)
    assert extension._beacon_api.beacons == {}


async def test_lost_source_tag_falls_back_to_another_live_tag(extension, device):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    assert extension._anchor_cell_sources["default"] == DEVICE_SYSID

    # a second tag carries the same cell but a moved A1
    other = SimpleNamespace(
        system_id=99,
        address=("192.168.4.99", 3333),
        last_seen=time.monotonic(),
        params=dict(extension._protocol.devices[DEVICE_SYSID].params),
        param_types=dict(extension._protocol.devices[DEVICE_SYSID].param_types),
        param_count=extension._protocol.devices[DEVICE_SYSID].param_count,
    )
    set_cached_param(other, "UWB_AN1_X", 20.0, "real32")
    extension._protocol.devices[other.system_id] = other

    extension._protocol.devices.pop(DEVICE_SYSID)
    extension._handle_lost(ProtocolEvent("lost", DEVICE_SYSID))

    # the cell re-homes onto the surviving tag and re-projects from its geometry
    assert extension._anchor_cell_sources["default"] == other.system_id
    anchor1 = extension._beacon_api.beacons["rtls::default::anchor_1"]
    assert anchor1.position.lat == pytest.approx(41.3901797)


async def test_anchor_beacon_refreshes_after_param_set(extension, device):
    add_rtls_cell_params(device)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)

    result = await extension.set_param(DEVICE_SYSID, "UWB_AN1_X", 20.0, "real32")
    assert result["accepted"] is True

    anchor1 = extension._beacon_api.beacons["rtls::default::anchor_1"]
    assert anchor1.position.lat == pytest.approx(41.3901797)
    assert anchor1.position.lon == pytest.approx(2.1501197)
    assert anchor1.position.amsl == pytest.approx(14.8)


# ---- identity-param snapshot refill -------------------------------------


def add_anchor_identity_params(device, *, uwb_mac=2):
    """Give a FakeDevice an anchor identity: responder role, its own MAC
    and a two-entry anchor MAC table (MAC 2 -> index A1)."""
    for name, value, param_type in (
        ("UWB_ROLE", 3, "uint8"),
        ("UWB_MAC", uwb_mac, "uint16"),
        ("UWB_AN_COUNT", 2, "uint8"),
        ("UWB_AN0_MAC", 1, "uint16"),
        ("UWB_AN1_MAC", 2, "uint16"),
    ):
        set_fake_param(device, name, value, param_type)


async def test_refill_requests_only_missing_anchor_params(extension, device):
    add_anchor_identity_params(device)
    # the startup burst loses two identity datagrams of the dump
    device.drop_from_list = {"UWB_MAC", "UWB_AN1_MAC"}
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert "UWB_MAC" not in cached.params

    device.read_requests.clear()
    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )

    # exactly the missing names were re-requested, and the replies
    # completed the snapshot (dropping the refill bookkeeping)
    assert sorted(device.read_requests) == ["UWB_AN1_MAC", "UWB_MAC"]
    assert "UWB_MAC" in cached.params
    assert "UWB_AN1_MAC" in cached.params
    assert DEVICE_SYSID not in extension._refill

    # a complete snapshot is never polled again
    await extension._poll_param_refill(time.monotonic() + 10_000)
    assert sorted(device.read_requests) == ["UWB_AN1_MAC", "UWB_MAC"]


async def test_refill_restores_anchor_name(extension, device, builder, hub):
    add_anchor_identity_params(device)  # own MAC 2 -> A1
    device.drop_from_list = {"UWB_MAC"}
    await discover(extension, device)

    # degraded: without its own MAC the anchor renders the generic name
    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["name"] == f"RTLS anchor {DEVICE_SYSID}"

    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )

    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["name"] == "RTLS anchor A1"


async def test_refill_backs_off_and_gives_up_at_the_retry_cap(
    extension, device
):
    add_anchor_identity_params(device)
    device.drop_from_list = {"UWB_MAC"}
    await discover(extension, device)
    # ... and the device never answers targeted reads either
    device.respond_to_read = False
    device.read_requests.clear()

    base = time.monotonic() + REFILL_INITIAL_DELAY + 1
    await extension._poll_param_refill(base)
    assert device.read_requests == ["UWB_MAC"]

    # inside the backoff window nothing new is requested
    await extension._poll_param_refill(base + 1)
    assert device.read_requests == ["UWB_MAC"]

    # the retry cap bounds the total number of rounds, then the refill
    # sheds its bookkeeping: no infinite polling of a dead-silent device
    now = base
    for _ in range(REFILL_MAX_ATTEMPTS + 3):
        now += 10_000
        await extension._poll_param_refill(now)
    assert device.read_requests == ["UWB_MAC"] * REFILL_MAX_ATTEMPTS
    assert DEVICE_SYSID not in extension._refill

    # rediscovery re-arms the refill (fresh attempt budget)
    extension._protocol.devices.pop(DEVICE_SYSID)
    extension._handle_lost(ProtocolEvent("lost", DEVICE_SYSID))
    await discover(extension, device)
    assert DEVICE_SYSID in extension._refill


async def test_refill_skips_complete_but_unmatched_anchor(extension, device):
    # an anchor whose identity params are all PRESENT but whose own MAC
    # genuinely is not in the anchor table is a configuration state, not
    # dump loss: no re-requests (POS_X is dropped so the snapshot is not
    # count-complete and the semantic check itself is exercised)
    add_anchor_identity_params(device, uwb_mac=99)
    device.drop_from_list = {"POS_X"}
    await discover(extension, device)
    device.read_requests.clear()

    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )
    assert device.read_requests == []
    assert DEVICE_SYSID not in extension._refill


async def test_refill_ignores_lost_non_identity_params(extension, device):
    # a lossy dump that only lost params irrelevant to naming/geometry
    # must not trigger any targeted reads
    add_anchor_identity_params(device)
    device.drop_from_list = {"POS_X", "UWB_CH"}
    await discover(extension, device)
    device.read_requests.clear()

    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )
    assert device.read_requests == []
    assert DEVICE_SYSID not in extension._refill


async def test_refill_skips_count_complete_snapshot(extension, device):
    # the stock FakeDevice has no UWB_ROLE at all; its dump arrives
    # complete (params == param_count), so nothing was lost and the
    # absent role param must not be polled for
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert cached.param_count == len(cached.params)
    device.read_requests.clear()

    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )
    assert device.read_requests == []
    assert DEVICE_SYSID not in extension._refill


async def test_refill_restores_tag_cell_and_beacons(extension, device):
    add_rtls_cell_params(device)  # tag, full cell
    extension._beacon_api = StubBeaconAPI()
    # the lossy dump punched holes in the origin AND the per-anchor
    # table (cell_from_params needs the NED triple of every counted
    # anchor, and the MAC drives the beacon's active matching)
    device.drop_from_list = {"ORIGIN_LAT_E7", "UWB_AN1_X", "UWB_AN1_MAC"}
    await discover(extension, device)
    # the incomplete cell geometry renders no beacons
    assert extension._beacon_api.beacons == {}

    device.read_requests.clear()
    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )

    # CELL_ID and POS_YAW_DEG are geometry-consistency identity params
    # now; this fake registry has neither, so the first round requests
    # them (fruitlessly) too — and once the answered reads make the
    # snapshot count-complete again, absence is authoritative and the
    # refill entry is dropped as before
    assert sorted(device.read_requests) == [
        "CELL_ID",
        "ORIGIN_LAT_E7",
        "POS_YAW_DEG",
        "UWB_AN1_MAC",
        "UWB_AN1_X",
    ]
    assert "rtls::default::anchor_1" in extension._beacon_api.beacons
    assert DEVICE_SYSID not in extension._refill


async def test_refill_redumps_when_role_is_unknown(extension, device):
    add_anchor_identity_params(device)
    # the whole dump is lost at discovery: the server cannot even
    # classify the device, and targeted reads cannot enumerate holes
    # whose names were never listed — the refill re-requests the dump
    device.respond_to_list = False
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert not cached.params

    device.respond_to_list = True  # this time the dump gets through
    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )
    # no targeted reads were needed: the re-dump filled the snapshot
    assert device.read_requests == []
    assert "UWB_ROLE" in cached.params
    assert "UWB_AN1_MAC" in cached.params
    assert DEVICE_SYSID not in extension._refill


async def test_refill_repairs_legacy_tag_without_role_param(
    extension, device
):
    # a legacy tag (no UWB_ROLE param at all; the _cell_source_role
    # compatibility bridge) is recognized only once its snapshot is
    # count-complete — even a lost NON-identity param (POS_X) keeps it
    # unrecognized, so the refill must re-dump rather than issue
    # targeted reads for names it cannot know
    add_rtls_cell_params(device)
    device.params.pop("UWB_ROLE")
    extension._beacon_api = StubBeaconAPI()
    device.drop_from_list = {"ORIGIN_LAT_E7", "POS_X"}
    await discover(extension, device)
    assert extension._beacon_api.beacons == {}

    device.drop_from_list = set()  # the retransmitted dump gets through
    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )
    # snapshot count-complete -> the legacy tag inference activates and
    # the cell renders
    assert device.read_requests == []
    assert "rtls::default::anchor_1" in extension._beacon_api.beacons
    assert DEVICE_SYSID not in extension._refill


async def test_refill_paces_targeted_reads(extension, device, autojump_clock):
    add_anchor_identity_params(device)
    device.drop_from_list = {"UWB_MAC", "UWB_AN0_MAC", "UWB_AN1_MAC"}
    await discover(extension, device)
    device.respond_to_read = False

    stamps = []
    transport = extension._send

    async def recording_send(payload, address):
        stamps.append(trio.current_time())
        await transport(payload, address)

    extension._send = recording_send
    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )
    assert device.read_requests == ["UWB_MAC", "UWB_AN0_MAC", "UWB_AN1_MAC"]
    # the reads of one round are spaced out, not blasted as one burst
    # the firmware's management TX queue would drop replies from
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert len(gaps) == 2
    assert all(gap >= REFILL_READ_SPACING for gap in gaps)


def test_configured_addresses_allow_per_target_ports():
    assert _configured_addresses(["192.0.2.10:3344", "192.0.2.11"], 3333) == [
        ("192.0.2.10", 3344),
        ("192.0.2.11", 3333),
    ]


# ---- inter-anchor TWR telemetry passthrough -----------------------------


async def test_inter_anchor_twr_surfaced_in_inf(extension, device, builder, hub):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)

    # the anchor reports measured ranges to two peers as NAMED_VALUE_FLOAT in
    # the real firmware "twr<peer-mac-hex>" format; the SDK decodes the peer MAC
    # and the extension surfaces a per-peer row with a derived age. This crosses
    # the firmware -> SDK -> server name boundary end to end.
    t0 = time.monotonic()
    for name, value in (("twr0001", 14.0), ("twr0002", 9.5)):
        await extension._process_datagram(
            device.named_value_float(name, value), device.address, t0
        )

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    rows = {row["peerMac"]: row for row in entry["twr"]}
    assert rows[0x0001]["distanceM"] == pytest.approx(14.0)
    assert rows[0x0002]["distanceM"] == pytest.approx(9.5)
    assert all(row["ageMs"] >= 0 for row in entry["twr"])


async def test_twr_does_not_pollute_health_stats(extension, device):
    await discover(extension, device)
    # a TWR float must not be mistaken for a solve stat
    await extension._process_datagram(
        device.named_value_float("twr0001", 14.0), device.address, time.monotonic()
    )
    assert DEVICE_SYSID not in extension._stats
    distance_m, _harvested = extension._twr[DEVICE_SYSID][0x0001]
    assert distance_m == pytest.approx(14.0)


async def test_twr_pruned_on_device_loss(extension, device, builder, hub):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)
    await extension._process_datagram(
        device.named_value_float("twr0001", 14.0), device.address, time.monotonic()
    )
    assert DEVICE_SYSID in extension._twr

    events = extension._protocol.expire(time.monotonic() + 1000)
    for event in events:
        extension._handle_lost(event)
    assert DEVICE_SYSID not in extension._twr


# ---- X-RTLS-PARAM-LIST --------------------------------------------------


async def test_param_list(extension, device, builder, hub):
    await discover(extension, device)

    message = make_message(builder, {"type": "X-RTLS-PARAM-LIST", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_PARAM_LIST(message, None, hub)

    assert response.body["type"] == "X-RTLS-PARAM-LIST"
    assert response.body["id"] == DEVICE_SYSID
    assert response.body["count"] == len(device.params)
    params = response.body["params"]
    assert params["MAV_SYS_ID"] == {"value": DEVICE_SYSID, "type": "uint8", "index": 0}
    assert params["UWB_CH"]["value"] == 5
    assert params["UWB_CH"]["type"] == "int32"
    assert params["POS_X"]["value"] == 1.5
    assert params["POS_X"]["type"] == "real32"
    assert params["FW_VERSION"] == {"value": "1.2.3", "type": "custom", "index": 3}


async def test_param_list_unknown_device(extension, device, builder, hub):
    await discover(extension, device)
    message = make_message(builder, {"type": "X-RTLS-PARAM-LIST", "id": 99})
    response = await extension._handle_RTLS_PARAM_LIST(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "99" in response.body["reason"]


async def test_param_list_missing_id(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-PARAM-LIST"})
    response = await extension._handle_RTLS_PARAM_LIST(message, None, hub)
    assert response.body["type"] == "ACK-NAK"


async def test_param_list_timeout(extension, device, builder, hub, autojump_clock):
    await discover(extension, device)
    device.respond_to_list = False

    message = make_message(builder, {"type": "X-RTLS-PARAM-LIST", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_PARAM_LIST(message, None, hub)

    assert response.body["type"] == "ACK-NAK"
    assert "Timeout" in response.body["reason"]


# ---- X-RTLS-PARAM-GET ---------------------------------------------------


async def test_param_get(extension, device, builder, hub):
    await discover(extension, device)

    message = make_message(
        builder, {"type": "X-RTLS-PARAM-GET", "id": DEVICE_SYSID, "name": "UWB_CH"}
    )
    response = await extension._handle_RTLS_PARAM_GET(message, None, hub)

    assert response.body == {
        "type": "X-RTLS-PARAM-GET",
        "id": DEVICE_SYSID,
        "name": "UWB_CH",
        "value": 5,
        "paramType": "int32",
    }


async def test_param_get_timeout(extension, device, builder, hub, autojump_clock):
    await discover(extension, device)
    device.respond_to_read = False

    message = make_message(
        builder, {"type": "X-RTLS-PARAM-GET", "id": DEVICE_SYSID, "name": "UWB_CH"}
    )
    response = await extension._handle_RTLS_PARAM_GET(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "Timeout" in response.body["reason"]


async def test_param_get_missing_name(extension, device, builder, hub):
    await discover(extension, device)
    message = make_message(builder, {"type": "X-RTLS-PARAM-GET", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_PARAM_GET(message, None, hub)
    assert response.body["type"] == "ACK-NAK"


# ---- X-RTLS-PARAM-SET ---------------------------------------------------


async def test_param_set_with_explicit_type(extension, device, builder, hub):
    await discover(extension, device)

    message = make_message(
        builder,
        {
            "type": "X-RTLS-PARAM-SET",
            "id": DEVICE_SYSID,
            "name": "UWB_CH",
            "value": 9,
            "paramType": "int32",
        },
    )
    response = await extension._handle_RTLS_PARAM_SET(message, None, hub)

    assert response.body == {
        "type": "X-RTLS-PARAM-SET",
        "id": DEVICE_SYSID,
        "name": "UWB_CH",
        "value": 9,
        "paramType": "int32",
        "result": 0,
        "accepted": True,
    }
    # the device-side store was really updated
    assert device.params["UWB_CH"][0] == struct.pack("<i", 9)


async def test_param_set_infers_type_from_cache(extension, device, builder, hub):
    # discovery auto-fetches the parameter list, which caches the types
    await discover(extension, device)

    message = make_message(
        builder,
        {
            "type": "X-RTLS-PARAM-SET",
            "id": DEVICE_SYSID,
            "name": "POS_X",
            "value": -2.25,
        },
    )
    response = await extension._handle_RTLS_PARAM_SET(message, None, hub)

    assert response.body["type"] == "X-RTLS-PARAM-SET"
    assert response.body["accepted"] is True
    assert response.body["value"] == -2.25
    assert device.params["POS_X"][0] == struct.pack("<f", -2.25)


async def test_param_set_rejected_by_device(extension, device, builder, hub):
    await discover(extension, device)
    device.set_result = PARAM_ACK_FAILED

    message = make_message(
        builder,
        {
            "type": "X-RTLS-PARAM-SET",
            "id": DEVICE_SYSID,
            "name": "UWB_CH",
            "value": 9,
            "paramType": "int32",
        },
    )
    response = await extension._handle_RTLS_PARAM_SET(message, None, hub)

    assert response.body["type"] == "X-RTLS-PARAM-SET"
    assert response.body["accepted"] is False
    assert response.body["result"] == PARAM_ACK_FAILED
    # device store untouched
    assert device.params["UWB_CH"][0] == struct.pack("<i", 5)


async def test_param_set_clamped_ack_passthrough(extension, device, builder, hub):
    """Firmware acks VALUE_UNSUPPORTED when a numeric set clamps to the
    parameter bounds (image-pinned UWB_ROLE is the motivating case); the
    handler must pass the honest ack through as accepted=False."""
    await discover(extension, device)
    device.set_result = PARAM_ACK_VALUE_UNSUPPORTED

    message = make_message(
        builder,
        {
            "type": "X-RTLS-PARAM-SET",
            "id": DEVICE_SYSID,
            "name": "MAV_SYS_ID",
            "value": 250,
            "paramType": "uint8",
        },
    )
    response = await extension._handle_RTLS_PARAM_SET(message, None, hub)
    assert response.body["type"] == "X-RTLS-PARAM-SET"
    assert response.body["accepted"] is False
    assert response.body["result"] == PARAM_ACK_VALUE_UNSUPPORTED


async def test_param_set_unknown_type_rejected(extension, device, builder, hub):
    await discover(extension, device)
    message = make_message(
        builder,
        {
            "type": "X-RTLS-PARAM-SET",
            "id": DEVICE_SYSID,
            "name": "NO_SUCH_PARAM",
            "value": 1,
        },
    )
    response = await extension._handle_RTLS_PARAM_SET(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "type" in response.body["reason"]


async def test_param_set_timeout(extension, device, builder, hub, autojump_clock):
    await discover(extension, device)
    device.respond_to_set = False

    message = make_message(
        builder,
        {
            "type": "X-RTLS-PARAM-SET",
            "id": DEVICE_SYSID,
            "name": "UWB_CH",
            "value": 9,
            "paramType": "int32",
        },
    )
    response = await extension._handle_RTLS_PARAM_SET(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "Timeout" in response.body["reason"]


# ---- X-RTLS-SLEEP -------------------------------------------------------

# Some sleep assertions need an SDK that tracks the heartbeat system_status
# and the `slp` stat; with the pinned pre-sleep SDK the command path still
# works but the state is invisible. Those tests are skipped until the
# rtls-link pin moves past fw feature/sleep-mode.
requires_sleep_sdk = pytest.mark.skipif(
    not hasattr(rtlslink_module, "SLEEP_PARAM"),
    reason="pinned rtls-link SDK predates sleep mode",
)


@requires_sleep_sdk
async def test_sleep_accepted(extension, device, builder, hub, autojump_clock):
    await discover(extension, device)

    message = make_message(
        builder,
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": True},
    )
    response = await extension._handle_RTLS_SLEEP(message, None, hub)

    assert response.body["type"] == "X-RTLS-SLEEP"
    assert response.body["sleeping"] is True
    entry = response.body["result"][str(DEVICE_SYSID)]
    assert entry["accepted"] is True
    assert entry["sleeping"] is True
    assert device.sleeping is True

    # mirror-optimism: the verified sleep is reported by X-RTLS-INF right
    # away, without waiting for the next STANDBY heartbeat to re-latch it
    inf = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert inf.body["status"][str(DEVICE_SYSID)]["sleeping"] is True

    # the device now advertises STANDBY; the next heartbeat marks the
    # cached device and X-RTLS-INF reports it
    await discover(extension, device)
    inf = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert inf.body["status"][str(DEVICE_SYSID)]["sleeping"] is True


async def test_sleep_refused_by_arming_gate(
    extension, device, builder, hub, autojump_clock
):
    await discover(extension, device)
    device.refuse_sleep = True

    message = make_message(
        builder,
        {"type": "X-RTLS-SLEEP", "id": DEVICE_SYSID, "sleeping": True},
    )
    response = await extension._handle_RTLS_SLEEP(message, None, hub)

    entry = response.body["result"][str(DEVICE_SYSID)]
    assert entry["accepted"] is False
    assert entry["sleeping"] is False
    assert "refused" in entry["detail"]
    assert device.sleeping is False


async def test_sleep_wake(extension, device, builder, hub, autojump_clock):
    await discover(extension, device)
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)

    message = make_message(
        builder,
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": False},
    )
    response = await extension._handle_RTLS_SLEEP(message, None, hub)

    entry = response.body["result"][str(DEVICE_SYSID)]
    assert entry == {
        "requested": False,
        "accepted": True,
        "sleeping": False,
        "detail": "awake",
    }
    assert device.sleeping is False


@requires_sleep_sdk
async def test_wake_optimistically_reports_awake(
    extension, device, builder, hub, autojump_clock
):
    """An accepted wake must flip the server-side sleep latch immediately:
    the device's last heartbeat predates the wake (STANDBY) and on hardware
    it reboots off the network for seconds before its first ACTIVE
    heartbeat -- without the optimistic update every X-RTLS-INF in that
    window shows the woken drone asleep (control issue #16, cause 3)."""
    # the device is discovered asleep: its heartbeat latches STANDBY
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert cached.sleeping is True

    message = make_message(
        builder,
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": False},
    )
    response = await extension._handle_RTLS_SLEEP(message, None, hub)
    assert response.body["result"][str(DEVICE_SYSID)]["accepted"] is True

    # no post-wake heartbeat has arrived, yet the server already reports
    # the state it just established...
    assert cached.sleeping is False
    inf = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert inf.body["status"][str(DEVICE_SYSID)]["sleeping"] is False

    # ...and pushes it: the wake marked the device list dirty (or already
    # broadcast on the leading edge), so by the time the protocol loop's
    # flush runs, the latest pushed list reports the device awake -- the
    # discovery push above still said asleep
    await extension._flush_pending_inf(now=time.monotonic() + 10.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) >= 2
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["sleeping"] is False


async def _wake_asleep_device(extension, device, builder, hub):
    """Discover the FakeDevice asleep, then run an accepted wake through
    the X-RTLS-SLEEP handler (shared setup of the sleep-pin tests)."""
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await discover(extension, device)
    message = make_message(
        builder,
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": False},
    )
    response = await extension._handle_RTLS_SLEEP(message, None, hub)
    assert response.body["result"][str(DEVICE_SYSID)]["accepted"] is True


@requires_sleep_sdk
async def test_wake_pin_overrides_late_standby_heartbeat(
    extension, device, builder, hub, autojump_clock
):
    """A contradicting heartbeat right after an accepted wake must not
    revert the optimistic state: the firmware acks SLEEP=0 before its
    power task cuts over, so a pre-transition STANDBY heartbeat can still
    be in flight -- believing it would push 'asleep' back to clients and
    recreate the stale-state bug the optimism exists to fix."""
    await _wake_asleep_device(extension, device, builder, hub)
    pushes = len(_inf_broadcasts(extension))

    # the late pre-transition STANDBY heartbeat arrives...
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await _feed_heartbeat(extension, device, now=time.monotonic())

    # ...and is overridden: nothing was pushed for it, and both the query
    # and the eventually flushed push still report the wake outcome
    assert len(_inf_broadcasts(extension)) == pushes
    inf = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert inf.body["status"][str(DEVICE_SYSID)]["sleeping"] is False
    await extension._flush_pending_inf(now=time.monotonic() + 10.0)
    broadcasts = _inf_broadcasts(extension)
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["sleeping"] is False


@requires_sleep_sdk
async def test_wake_pin_cleared_by_confirming_heartbeat(
    extension, device, builder, hub, autojump_clock
):
    await _wake_asleep_device(extension, device, builder, hub)
    assert DEVICE_SYSID in extension._sleep_pins

    # the post-reboot ACTIVE heartbeat confirms the wake and clears the
    # pin without a duplicate push (the state did not change)
    await _feed_heartbeat(extension, device, now=time.monotonic())
    assert DEVICE_SYSID not in extension._sleep_pins

    # normal tracking has resumed: a later STANDBY flip is believed,
    # pushed, and reported again
    pushes = len(_inf_broadcasts(extension))
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await _feed_heartbeat(extension, device, now=time.monotonic() + 5.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == pushes + 1
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["sleeping"] is True


@requires_sleep_sdk
async def test_wake_pin_expires(extension, device, builder, hub, autojump_clock):
    await _wake_asleep_device(extension, device, builder, hub)
    assert DEVICE_SYSID in extension._sleep_pins

    # the device never confirmed the wake; past the pin deadline its own
    # STANDBY reports win again (truth over stale optimism)
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await _feed_heartbeat(
        extension, device, now=time.monotonic() + SLEEP_PIN_TIMEOUT + 1.0
    )
    assert DEVICE_SYSID not in extension._sleep_pins
    inf = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert inf.body["status"][str(DEVICE_SYSID)]["sleeping"] is True


@requires_sleep_sdk
async def test_wake_pin_covers_rediscovery_checkpoint(
    extension, device, builder, hub, autojump_clock
):
    """The wake reboot drops the device off the network (lost, latch
    pruned) and it may be rediscovered off a stale pre-transition STANDBY
    heartbeat. feed() installs that raw state on the device BEFORE the
    discovery handler runs, and the handler awaits the param-list send --
    so the pin seed/re-assert must run before that await, or a concurrent
    X-RTLS-INF query at the checkpoint briefly answers 'asleep'."""
    await _wake_asleep_device(extension, device, builder, hub)
    for event in extension._protocol.expire(time.monotonic() + 1000.0):
        extension._handle_lost(event)
    assert DEVICE_SYSID not in extension._protocol.devices
    assert DEVICE_SYSID in extension._sleep_pins  # the pin survives loss

    # emulate the concurrent query at the handler's await checkpoint: the
    # stubbed transport (which never replies, like a mid-reboot device)
    # queries INF from inside the param-list send
    seen = []

    async def send_and_query(payload, address):
        inf = await extension._handle_RTLS_INF(
            make_message(builder, {"type": "X-RTLS-INF"}), None, hub
        )
        seen.append(inf.body["status"][str(DEVICE_SYSID)].get("sleeping"))

    extension._send = send_and_query
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await _feed_heartbeat(extension, device, now=time.monotonic() + 5.0)

    # the query saw the pinned wake outcome, never the stale STANDBY
    assert seen == [False]
    assert DEVICE_SYSID in extension._sleep_pins  # contradiction: pin holds


async def test_sleep_multi_device_reports_unknown(
    extension, device, builder, hub, autojump_clock
):
    await discover(extension, device)

    message = make_message(
        builder,
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID, 99], "sleeping": True},
    )
    response = await extension._handle_RTLS_SLEEP(message, None, hub)

    result = response.body["result"]
    assert result[str(DEVICE_SYSID)]["accepted"] is True
    assert result["99"]["accepted"] is False
    assert "No such device" in result["99"]["detail"]


async def test_sleep_rejects_malformed_requests(extension, device, builder, hub):
    await discover(extension, device)

    for body in (
        {"type": "X-RTLS-SLEEP", "sleeping": True},
        {"type": "X-RTLS-SLEEP", "ids": [], "sleeping": True},
        {"type": "X-RTLS-SLEEP", "ids": ["42"], "sleeping": True},
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID]},
        {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": "yes"},
        # bool is an int subclass; JSON true must not target sysid 1
        {"type": "X-RTLS-SLEEP", "ids": [True], "sleeping": True},
        {"type": "X-RTLS-SLEEP", "ids": [0], "sleeping": True},
        {"type": "X-RTLS-SLEEP", "ids": [256], "sleeping": True},
        {"type": "X-RTLS-SLEEP", "ids": list(range(1, 255)) * 2, "sleeping": True},
    ):
        message = make_message(builder, body)
        response = await extension._handle_RTLS_SLEEP(message, None, hub)
        assert response.body.get("type") == "ACK-NAK", body


# ---- X-RTLS-OTA ---------------------------------------------------------


async def test_ota_full_flow(extension, device, builder, hub, tmp_path):
    await discover(extension, device)

    image = tmp_path / "firmware.bin"
    image.write_bytes(b"\x00" * 64)

    calls = []

    def fake_upgrade(address, image_path, *, timeout=10.0, on_progress=None):
        calls.append((address, image_path))
        if on_progress is not None:
            on_progress(32, 64)
            on_progress(64, 64)
        return "9.9.9"

    extension._ota_upgrade = fake_upgrade

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery

        start = make_message(
            builder, {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "image": str(image)}
        )
        response = await extension._handle_RTLS_OTA(start, None, hub)

        assert response.body["type"] == "X-RTLS-OTA"
        assert response.body["job"]["status"] == "running"
        assert response.body["job"]["image"] == str(image)

        # the job runs in the background; wait for it to finish
        with trio.fail_after(5):
            while extension._ota_jobs[DEVICE_SYSID]["status"] == "running":
                await trio.sleep(0.01)

    extension._nursery = None

    assert calls == [(DEVICE_ADDRESS[0], str(image))]

    query = make_message(builder, {"type": "X-RTLS-OTA", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_OTA(query, None, hub)
    job = response.body["job"]
    assert job["status"] == "success"
    assert job["progress"] == 1.0
    assert job["version"] == "9.9.9"
    assert job["error"] is None

    # a final status notification was broadcast to all clients
    broadcasts = extension.app.message_hub.broadcasts
    assert broadcasts
    assert broadcasts[-1]["type"] == "X-RTLS-OTA"
    assert broadcasts[-1]["id"] == DEVICE_SYSID
    assert broadcasts[-1]["job"]["status"] == "success"

    # the OTA state also shows up in X-RTLS-INF
    inf = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(inf, None, hub)
    assert response.body["status"][str(DEVICE_SYSID)]["otaStatus"] == "success"


async def test_ota_role_guardrail_accepts_matching_species(
    extension, device, builder, hub, tmp_path
):
    device.params["UWB_ROLE"] = (bytes([3]), PARAM_TYPE_UINT8)  # anchor-responder
    await discover(extension, device)

    image = tmp_path / "anchor.bin"
    image.write_bytes(b"\x00" * 16)
    extension._ota_upgrade = lambda address, image_path, *, timeout=10.0, on_progress=None: "9.9.9"

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        start = make_message(
            builder,
            {
                "type": "X-RTLS-OTA",
                "id": DEVICE_SYSID,
                "image": str(image),
                "role": "anchor",
            },
        )
        response = await extension._handle_RTLS_OTA(start, None, hub)
        assert response.body["type"] == "X-RTLS-OTA"
        assert response.body["job"]["status"] == "running"
        with trio.fail_after(5):
            while extension._ota_jobs[DEVICE_SYSID]["status"] == "running":
                await trio.sleep(0.01)
    extension._nursery = None
    assert extension._ota_jobs[DEVICE_SYSID]["status"] == "success"


async def test_ota_role_guardrail_rejects_wrong_species(
    extension, device, builder, hub, tmp_path
):
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)  # a tag device
    await discover(extension, device)

    image = tmp_path / "anchor.bin"
    image.write_bytes(b"\x00" * 16)

    start = make_message(
        builder,
        {
            "type": "X-RTLS-OTA",
            "id": DEVICE_SYSID,
            "image": str(image),
            "role": "anchor",
        },
    )
    response = await extension._handle_RTLS_OTA(start, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "image-pinned" in response.body["reason"]
    assert DEVICE_SYSID not in extension._ota_jobs


async def test_ota_role_guardrail_rejects_invalid_role(
    extension, device, builder, hub, tmp_path
):
    await discover(extension, device)
    image = tmp_path / "fw.bin"
    image.write_bytes(b"\x00" * 16)

    start = make_message(
        builder,
        {
            "type": "X-RTLS-OTA",
            "id": DEVICE_SYSID,
            "image": str(image),
            "role": "drone",
        },
    )
    response = await extension._handle_RTLS_OTA(start, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "Invalid role" in response.body["reason"]


async def test_ota_rejects_concurrent_jobs(extension, device, builder, hub, tmp_path):
    await discover(extension, device)

    image = tmp_path / "firmware.bin"
    image.write_bytes(b"\x00" * 16)

    release = threading.Event()

    def slow_upgrade(address, image_path, *, timeout=10.0, on_progress=None):
        release.wait(5)
        return "9.9.9"

    extension._ota_upgrade = slow_upgrade

    try:
        async with trio.open_nursery() as nursery:
            extension._nursery = nursery

            start = make_message(
                builder, {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "image": str(image)}
            )
            response = await extension._handle_RTLS_OTA(start, None, hub)
            assert response.body["job"]["status"] == "running"

            again = make_message(
                builder, {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "image": str(image)}
            )
            response = await extension._handle_RTLS_OTA(again, None, hub)
            assert response.body["type"] == "ACK-NAK"
            assert "in progress" in response.body["reason"]

            release.set()
    finally:
        release.set()
        extension._nursery = None

    assert extension._ota_jobs[DEVICE_SYSID]["status"] == "success"


async def test_ota_failure_is_reported(extension, device, builder, hub, tmp_path):
    await discover(extension, device)

    image = tmp_path / "firmware.bin"
    image.write_bytes(b"\x00" * 16)

    def failing_upgrade(address, image_path, *, timeout=10.0, on_progress=None):
        raise RuntimeError("device went away")

    extension._ota_upgrade = failing_upgrade

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        start = make_message(
            builder, {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "image": str(image)}
        )
        await extension._handle_RTLS_OTA(start, None, hub)
        with trio.fail_after(5):
            while extension._ota_jobs[DEVICE_SYSID]["status"] == "running":
                await trio.sleep(0.01)
    extension._nursery = None

    job = extension._ota_jobs[DEVICE_SYSID]
    assert job["status"] == "error"
    assert "device went away" in job["error"]


async def test_ota_rejects_missing_image(extension, device, builder, hub):
    await discover(extension, device)
    message = make_message(
        builder,
        {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "image": "/no/such/file.bin"},
    )
    response = await extension._handle_RTLS_OTA(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "No such image" in response.body["reason"]


async def test_ota_status_query_without_job(extension, device, builder, hub):
    await discover(extension, device)
    message = make_message(builder, {"type": "X-RTLS-OTA", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_OTA(message, None, hub)
    assert response.body == {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "job": None}


# ---- device loss --------------------------------------------------------


async def test_lost_device_disappears_from_inf(extension, device, builder, hub):
    await discover(extension, device)
    protocol = extension._protocol

    # simulate the device going silent past its timeout
    events = protocol.expire(time.monotonic() + 1000)
    assert [event.kind for event in events] == ["lost"]

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    assert response.body["status"] == {}


# ---- X-RTLS-INF push notifications --------------------------------------
#
# The GUI's device list comes from a one-shot X-RTLS-INF query, so the
# server pushes the full list (same body shape as the query response, for
# a wholesale replace) whenever a device is gained or lost -- otherwise
# an unplugged anchor stays green on every connected client.


def _inf_broadcasts(extension):
    return [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-INF"
    ]


async def _feed_heartbeat(extension, device, now):
    """Feed one device heartbeat at a controlled monotonic time (the
    ``discover`` helper uses the real clock, which the INF throttle tests
    cannot work with)."""
    await extension._process_datagram(device.heartbeat(), device.address, now)


async def test_inf_broadcast_on_discovery(extension, device, builder, hub):
    await _feed_heartbeat(extension, device, now=0.0)

    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 1
    body = broadcasts[-1]
    # same body shape as the query response, so clients can apply a
    # wholesale replace
    response = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert set(body) == set(response.body)
    assert set(body["status"]) == {str(DEVICE_SYSID)}
    entry = body["status"][str(DEVICE_SYSID)]
    assert entry["id"] == DEVICE_SYSID
    assert entry["address"] == list(DEVICE_ADDRESS)


async def test_inf_broadcast_on_loss(extension, device):
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1

    # the device goes silent past its timeout; the protocol loop handles the
    # lost event (which only marks the list dirty -- the handler is sync) and
    # then runs the periodic flush, which broadcasts the now-empty list
    events = extension._protocol.expire(1000.0)
    assert [event.kind for event in events] == ["lost"]
    for event in events:
        extension._handle_lost(event)
    await extension._flush_pending_inf(now=1000.0)

    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["status"] == {}


async def test_inf_broadcast_coalesces_bursts(extension, device, dialect):
    # leading edge: the first discovery broadcasts straight away
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1

    # a second device appears inside the throttle window (during a show,
    # drone tags flap in bursts): only marked pending, NOT broadcast yet
    second = FakeDevice(dialect, system_id=DEVICE_SYSID + 1)
    second.address = ("192.168.4.43", 3333)
    await extension._process_datagram(second.heartbeat(), second.address, 0.5)
    assert len(_inf_broadcasts(extension)) == 1

    # ...and a flush inside the window stays quiet too
    await extension._flush_pending_inf(now=0.9)
    assert len(_inf_broadcasts(extension)) == 1

    # the protocol loop's flush delivers the coalesced list (trailing edge)
    # once the window has elapsed, carrying both devices
    await extension._flush_pending_inf(now=1.5)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    assert set(broadcasts[-1]["status"]) == {
        str(DEVICE_SYSID),
        str(DEVICE_SYSID + 1),
    }

    # nothing pending: a further flush does not re-broadcast
    await extension._flush_pending_inf(now=2.0)
    assert len(_inf_broadcasts(extension)) == 2


async def test_inf_rebroadcast_refreshes_ages(extension, device):
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1

    # without a transition the list is only rebroadcast on the slow
    # heartbeat, so ages keep refreshing on clients
    await extension._flush_pending_inf(now=5.0)
    assert len(_inf_broadcasts(extension)) == 1

    await extension._flush_pending_inf(now=10.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    entry = broadcasts[-1]["status"][str(DEVICE_SYSID)]
    assert entry["age"] == pytest.approx(10.0)


async def test_inf_no_rebroadcast_without_devices(extension, device):
    await _feed_heartbeat(extension, device, now=0.0)
    for event in extension._protocol.expire(1000.0):
        extension._handle_lost(event)
    await extension._flush_pending_inf(now=1000.0)
    assert len(_inf_broadcasts(extension)) == 2

    # an empty device table has no ages to refresh: the slow heartbeat
    # stays quiet until the next transition
    await extension._flush_pending_inf(now=2000.0)
    assert len(_inf_broadcasts(extension)) == 2


# ---- tag<->drone association (source-IP join) ---------------------------


def _set_uav_addresses(extension, addresses):
    """Install a fake UAV source-address feed (the seam through which the
    extension reads the mavlink extension's API)."""
    extension._uav_source_addresses = lambda: addresses


async def test_uav_mapping_appears_in_inf(extension, device, builder, hub):
    await discover(extension, device)

    # before any UAV shares the tag's IP the device is unassociated
    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    assert "uav" not in response.body["status"][str(DEVICE_SYSID)]

    # a connected UAV heard from the tag's bridge IP associates with it
    _set_uav_addresses(extension, {"05": (DEVICE_ADDRESS[0], 14550)})
    assert extension._refresh_uav_map() is True

    response = await extension._handle_RTLS_INF(message, None, hub)
    assert response.body["status"][str(DEVICE_SYSID)]["uav"] == "05"


async def test_uav_mapping_clears_on_ip_change(extension, device, builder, hub):
    await discover(extension, device)
    _set_uav_addresses(extension, {"05": (DEVICE_ADDRESS[0], 14550)})
    assert extension._refresh_uav_map() is True

    # a DHCP renewal moves the UAV's source IP off the tag: the mapping
    # must clear on the next recompute, never persist
    _set_uav_addresses(extension, {"05": ("192.168.4.99", 14550)})
    assert extension._refresh_uav_map() is True

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    assert "uav" not in response.body["status"][str(DEVICE_SYSID)]


async def test_uav_mapping_ambiguous_ip_maps_nothing(
    extension, device, builder, hub
):
    await discover(extension, device)
    # two UAVs claiming one source IP cannot be told apart; mapping either
    # would risk exactly the mis-attribution this feature exists to prevent
    _set_uav_addresses(
        extension,
        {"05": (DEVICE_ADDRESS[0], 14550), "06": (DEVICE_ADDRESS[0], 14551)},
    )
    assert extension._refresh_uav_map() is False

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    assert "uav" not in response.body["status"][str(DEVICE_SYSID)]


async def test_uav_mapping_change_pushes_inf(extension, device):
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1

    # the association appearing is a mapping change: the poll in the
    # protocol loop pushes the device list (throttled, like gained/lost)
    _set_uav_addresses(extension, {"05": (DEVICE_ADDRESS[0], 14550)})
    await extension._poll_uav_map(now=2.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["uav"] == "05"

    # an unchanged mapping stays quiet
    await extension._poll_uav_map(now=4.0)
    assert len(_inf_broadcasts(extension)) == 2

    # the UAV disappearing clears the mapping and pushes again
    _set_uav_addresses(extension, {})
    await extension._poll_uav_map(now=6.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 3
    assert "uav" not in broadcasts[-1]["status"][str(DEVICE_SYSID)]


def test_uav_addresses_empty_without_mavlink_api(extension):
    # no mavlink extension (or one that is not loaded) leaves every device
    # unassociated instead of failing
    assert extension._mavlink_api is None
    assert extension._uav_source_addresses() == {}
    extension._mavlink_api = SimpleNamespace(loaded=False)
    assert extension._uav_source_addresses() == {}


@requires_sleep_sdk
async def test_inf_broadcast_on_sleeping_flip(extension, device):
    """An ACTIVE<->STANDBY heartbeat transition must push X-RTLS-INF
    (throttled) instead of waiting for the 10 s slow refresh -- a tag that
    slept inside the refresh window otherwise renders green (control
    issue #16, cause 1)."""
    # discovery broadcasts the list once (device awake)
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1

    # a heartbeat without a state change pushes nothing new
    await _feed_heartbeat(extension, device, now=2.0)
    assert len(_inf_broadcasts(extension)) == 1

    # the device falls asleep (STANDBY heartbeat): pushed right away
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)
    await _feed_heartbeat(extension, device, now=4.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["sleeping"] is True

    # further STANDBY heartbeats stay quiet...
    await _feed_heartbeat(extension, device, now=6.0)
    assert len(_inf_broadcasts(extension)) == 2

    # ...and the wake transition pushes again
    device.params["SLEEP"] = (bytes([0]), PARAM_TYPE_UINT8)
    await _feed_heartbeat(extension, device, now=8.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 3
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["sleeping"] is False


@requires_sleep_sdk
async def test_inf_sleeping_omitted_when_latch_stale(extension, device):
    """The sleep latch has no staleness of its own; when a device stays
    alive past the device timeout without a heartbeat re-latching it (any
    traffic refreshes liveness in passive mode), X-RTLS-INF must omit the
    flag -- clients render the absence as unknown -- instead of reporting
    the stale latched state as definite."""
    await _feed_heartbeat(extension, device, now=0.0)
    status = extension._inf_status(now=1.0)
    assert status[str(DEVICE_SYSID)]["sleeping"] is False

    # heartbeats go silent while other traffic keeps the device alive
    cached = extension._protocol.devices[DEVICE_SYSID]
    cached.last_seen = 100.0
    status = extension._inf_status(now=100.0)
    assert "sleeping" not in status[str(DEVICE_SYSID)]

    # a fresh heartbeat restores the definite report
    await _feed_heartbeat(extension, device, now=100.0)
    status = extension._inf_status(now=100.5)
    assert status[str(DEVICE_SYSID)]["sleeping"] is False


# ---- X-RTLS-STATS health telemetry --------------------------------------


async def _feed_stats(extension, device, values, now):
    """Feed one NAMED_VALUE_FLOAT per stat at a controlled monotonic time,
    mirroring the firmware's per-stat emit cycle."""
    for name, value in values.items():
        await extension._process_datagram(
            device.named_value_float(name, value), device.address, now
        )


FULL_STATS = {
    "rate": 8.0,
    "solvepct": 80.0,
    "anc": 7.0,
    "agems": 20.0,
    "ppm": -1.25,
    "ancmask": 254.0,
    # fed last on purpose: the leading-edge broadcast fires on whichever
    # field completes the set, and these tests assert on the rate values
    "slp": 0.0,
}


def test_stats_json_carries_optional_battery_voltage():
    entry = _stats_json(DEVICE_SYSID, {**FULL_STATS, "vbat": 12.34567})

    assert entry["batteryVoltage"] == 12.346


def test_stats_json_without_vbat_omits_battery_voltage():
    entry = _stats_json(DEVICE_SYSID, FULL_STATS)

    assert "batteryVoltage" not in entry


async def test_stats_broadcast_on_update(extension, device):
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=0.0)

    broadcasts = extension.app.message_hub.broadcasts
    assert broadcasts
    body = broadcasts[-1]
    assert body["type"] == "X-RTLS-STATS"
    entry = body["stats"][str(DEVICE_SYSID)]
    assert entry == {
        "id": DEVICE_SYSID,
        "solveRateHz": 8.0,
        "solvePct": 80.0,
        "anchorsSeen": 7,
        "fixAgeMs": 20,
        "clockPpm": -1.25,
        "anchorMask": 254,
    }
    # integer-semantic fields are cast to int
    assert isinstance(entry["anchorsSeen"], int)
    assert isinstance(entry["fixAgeMs"], int)
    assert isinstance(entry["anchorMask"], int)


async def test_stats_query_returns_latest_snapshot(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=0.0)

    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)

    assert response.body["type"] == "X-RTLS-STATS"
    entry = response.body["stats"][str(DEVICE_SYSID)]
    assert entry["id"] == DEVICE_SYSID
    assert entry["solveRateHz"] == 8.0
    assert entry["anchorsSeen"] == 7
    assert entry["anchorMask"] == 254


@requires_sleep_sdk
async def test_stats_query_carries_sleep_state(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_stats(extension, device, {**FULL_STATS, "slp": 1.0}, now=0.0)

    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)

    entry = response.body["stats"][str(DEVICE_SYSID)]
    assert entry["sleeping"] is True


async def test_stats_without_slp_omits_sleep_state(extension, device, builder, hub):
    # firmware that predates sleep mode never sends slp: the snapshot
    # must still broadcast/query fine, with the key absent (unknown),
    # not False
    await discover(extension, device)
    legacy = {k: v for k, v in FULL_STATS.items() if k != "slp"}
    await _feed_stats(extension, device, legacy, now=0.0)

    stats_broadcasts = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert stats_broadcasts, "legacy stats must still broadcast"
    entry = stats_broadcasts[-1]["stats"][str(DEVICE_SYSID)]
    assert "sleeping" not in entry


async def test_stats_query_by_id(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=0.0)

    message = make_message(builder, {"type": "X-RTLS-STATS", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert set(response.body["stats"]) == {str(DEVICE_SYSID)}

    # an unknown device id yields an empty snapshot, not an error
    other = make_message(builder, {"type": "X-RTLS-STATS", "id": 99})
    response = await extension._handle_RTLS_STATS(other, None, hub)
    assert response.body["stats"] == {}


async def test_stats_query_empty_before_any_stats(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert response.body == {"type": "X-RTLS-STATS", "stats": {}}


async def test_stats_broadcast_throttled(extension, device):
    await discover(extension, device)

    # first update broadcasts immediately
    await _feed_stats(extension, device, FULL_STATS, now=0.0)
    # a second update well within STATS_INTERVAL must NOT broadcast again
    await _feed_stats(extension, device, {**FULL_STATS, "rate": 9.0}, now=0.3)

    stats_broadcasts = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert len(stats_broadcasts) == 1

    # once the interval elapses, a further update broadcasts again
    await _feed_stats(extension, device, {**FULL_STATS, "rate": 10.0}, now=1.5)
    stats_broadcasts = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert len(stats_broadcasts) == 2
    # and it carries the latest value
    assert (
        stats_broadcasts[-1]["stats"][str(DEVICE_SYSID)]["solveRateHz"] == 10.0
    )


async def test_stats_trailing_flush_delivers_latest_within_interval(extension, device):
    await discover(extension, device)

    # leading edge: the first complete update broadcasts straight away
    await _feed_stats(extension, device, FULL_STATS, now=0.0)
    # a newer complete update lands inside the throttle window, so _on_stats
    # only caches it -- it is NOT broadcast yet
    await _feed_stats(extension, device, {**FULL_STATS, "rate": 11.0}, now=0.5)

    stats_broadcasts = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert len(stats_broadcasts) == 1

    # the protocol loop's periodic flush runs once the window has elapsed and
    # pushes the latest cached snapshot -- even though no new stats arrived
    await extension._flush_pending_stats(now=1.0)

    stats_broadcasts = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert len(stats_broadcasts) == 2
    assert stats_broadcasts[-1]["stats"][str(DEVICE_SYSID)]["solveRateHz"] == 11.0

    # a redundant flush with no newer data does not re-broadcast
    await extension._flush_pending_stats(now=5.0)
    stats_broadcasts = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert len(stats_broadcasts) == 2


async def test_lost_device_stats_pruned_from_query(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=0.0)

    # the snapshot is reported before the device is lost
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert str(DEVICE_SYSID) in response.body["stats"]

    # drive the protocol loop's expiry path: the device times out, the
    # extension handles the lost event and prunes its cached stats
    protocol = extension._protocol
    events = protocol.expire(time.monotonic() + 1000)
    assert [event.kind for event in events] == ["lost"]
    for event in events:
        extension._handle_lost(event)

    # the X-RTLS-STATS query no longer reports the timed-out device
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert response.body["stats"] == {}
    # internal throttle bookkeeping was cleaned up too
    assert DEVICE_SYSID not in extension._stats
    assert DEVICE_SYSID not in extension._last_stats_broadcast
    assert DEVICE_SYSID not in extension._last_stats_sent


# ---- show-clock pin (cluster -> GPS) ----


def _add_pin_params(device):
    """Give the fake device the GPS_PIN_* param slots the firmware defines."""
    device.params.update(
        {
            "GPS_PIN_WEEK": (struct.pack("<H", 0), 4),  # MAV_PARAM_EXT_TYPE_UINT16
            "GPS_PIN_TOW_MS": (struct.pack("<I", 0), 5),  # UINT32
            "GPS_PIN_C0_HI": (struct.pack("<I", 0), 5),
            "GPS_PIN_C0_LO": (struct.pack("<I", 0), 5),
        }
    )


CLUSTER_STATS = {
    "rate": 8.0,
    "solvepct": 80.0,
    "agems": 20.0,
    "ppm": -1.25,
    "ancmask": 254.0,
    "clkh": 0.0,
    "clks": 120.5,
    "clkok": 1.0,
}


def test_show_clock_pin_pure():
    """mint_pin pairs cluster seconds with wall UTC; pin_writes follows the
    week-zero-first contract; the restart prediction tracks wall time."""
    from rtlslink import TICK_SECONDS

    from flockwave.server.ext.rtls.show_clock import mint_pin, pin_writes

    pin = mint_pin(120.5, now_unix=1_752_000_000.0)
    assert pin.c0_ticks == round(120.5 / TICK_SECONDS)
    assert 0 < pin.week < 4096
    assert 0 <= pin.tow_ms < 604_800_000

    writes = pin_writes(pin)
    # week zeroed first, written last; C0 split into the two u32 halves
    assert writes[0] == ("GPS_PIN_WEEK", 0, "uint16")
    assert writes[-1] == ("GPS_PIN_WEEK", pin.week, "uint16")
    assert ("GPS_PIN_C0_HI", (pin.c0_ticks >> 32) & 0xFFFFFFFF, "uint32") in writes
    assert ("GPS_PIN_C0_LO", pin.c0_ticks & 0xFFFFFFFF, "uint32") in writes

    # ten wall seconds later a live cluster should read ~10 s further
    assert pin.predicted_cluster_seconds(1_752_000_010.0) == pytest.approx(130.5)


async def test_show_clock_pin_distributed_on_fresh_stats(extension, device):
    """A clkok stats snapshot mints the pin and pushes it to the device in
    the contract order; the device ends up carrying the minted pin."""
    from flockwave.server.ext.rtls.show_clock import ShowClockPinManager

    _add_pin_params(device)
    extension._show_clock = ShowClockPinManager(extension)
    await discover(extension, device)

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, CLUSTER_STATS, now=0.0)
        # nursery exit waits for the spawned push to complete
    extension._nursery = None

    pin = extension._show_clock.pin
    assert pin is not None
    assert DEVICE_SYSID in extension._show_clock._pinned

    week = struct.unpack("<H", device.params["GPS_PIN_WEEK"][0])[0]
    tow = struct.unpack("<I", device.params["GPS_PIN_TOW_MS"][0])[0]
    c0_hi = struct.unpack("<I", device.params["GPS_PIN_C0_HI"][0])[0]
    c0_lo = struct.unpack("<I", device.params["GPS_PIN_C0_LO"][0])[0]
    assert week == pin.week
    assert tow == pin.tow_ms
    assert (c0_hi << 32) | c0_lo == pin.c0_ticks


async def test_show_clock_pin_not_minted_without_clkok(extension, device):
    """Stale or missing cluster stats must never mint a pin."""
    from flockwave.server.ext.rtls.show_clock import ShowClockPinManager

    _add_pin_params(device)
    extension._show_clock = ShowClockPinManager(extension)
    await discover(extension, device)

    stale = dict(CLUSTER_STATS, clkok=0.0)
    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, stale, now=0.0)
    extension._nursery = None

    assert extension._show_clock.pin is None
    week = struct.unpack("<H", device.params["GPS_PIN_WEEK"][0])[0]
    assert week == 0


async def test_show_clock_repin_on_cluster_restart(extension, device):
    """A cluster time far off the pin's prediction re-mints the pin and
    redistributes it (the restart makes every distributed pin invalid)."""
    from flockwave.server.ext.rtls.show_clock import ShowClockPinManager

    _add_pin_params(device)
    extension._show_clock = ShowClockPinManager(extension)
    await discover(extension, device)

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, CLUSTER_STATS, now=0.0)
    extension._nursery = None
    first_pin = extension._show_clock.pin
    assert DEVICE_SYSID in extension._show_clock._pinned

    # the cluster restarts: reported time drops far below the prediction
    restarted = dict(CLUSTER_STATS, clks=2.0)
    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, restarted, now=1.0)
    extension._nursery = None

    second_pin = extension._show_clock.pin
    assert second_pin is not first_pin
    assert DEVICE_SYSID in extension._show_clock._pinned
    week = struct.unpack("<H", device.params["GPS_PIN_WEEK"][0])[0]
    tow = struct.unpack("<I", device.params["GPS_PIN_TOW_MS"][0])[0]
    assert (week, tow) == (second_pin.week, second_pin.tow_ms)


async def test_show_clock_ignores_repeated_accumulated_snapshot(
    extension, device, monkeypatch
):
    """Other NAMED_VALUE_FLOAT fields repeat the cached clkh/clks pair.
    A parameter transaction can delay the next clock field beyond the restart
    tolerance; that stale snapshot must not trigger a false re-pin."""
    from flockwave.server.ext.rtls.show_clock import ShowClockPinManager

    now = [1_752_000_000.0]
    monkeypatch.setattr(
        "flockwave.server.ext.rtls.show_clock.time.time", lambda: now[0]
    )
    _add_pin_params(device)
    extension._show_clock = ShowClockPinManager(extension)
    await discover(extension, device)

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, CLUSTER_STATS, now=0.0)
    first_pin = extension._show_clock.pin
    assert first_pin is not None

    # The pin writes can occupy the management channel for longer than the
    # five-second restart tolerance. The SDK still emits accumulated snapshots
    # for non-clock stats, carrying the old clkh/clks pair.
    now[0] += 6.0
    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, CLUSTER_STATS, now=6.0)
    assert extension._show_clock.pin is first_pin

    # A genuinely new sample is still evaluated and agrees with the pin.
    advanced = dict(CLUSTER_STATS, clks=126.5)
    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, advanced, now=6.0)
    extension._nursery = None
    assert extension._show_clock.pin is first_pin


async def test_show_clock_lost_device_repinned_on_return(extension, device):
    """A lost device is dropped from the pinned set, so its next fresh
    stats snapshot after rediscovery pushes the pin again (it may have
    rebooted with default params)."""
    from flockwave.server.ext.rtls.show_clock import ShowClockPinManager

    _add_pin_params(device)
    extension._show_clock = ShowClockPinManager(extension)
    await discover(extension, device)

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        await _feed_stats(extension, device, CLUSTER_STATS, now=0.0)
    extension._nursery = None
    assert DEVICE_SYSID in extension._show_clock._pinned

    events = extension._protocol.expire(time.monotonic() + 1000)
    for event in events:
        extension._handle_lost(event)
    assert DEVICE_SYSID not in extension._show_clock._pinned


# ---- passive presence: state advertisements ------------------------------
#
# Firmware with feature/state-advertisement announces itself to UDP :3343
# every ADV_PERIOD_S: one datagram = HEARTBEAT + SYSTEM_TIME +
# PARAM_EXT_VALUE FW_VERSION/UWB_ROLE. The extension parses it with the
# SDK's parse_advertisement (a seam, so these tests run against the
# pinned SDK even when it predates the module) and feeds ONLY the
# advertiser's own validated frames — re-serialized by a fresh
# per-datagram parser — through the same protocol seam as
# management-channel datagrams.

#: an advertisement's UDP source: the firmware sends it from a throwaway
#: ephemeral port that never receives -- the extension must record the
#: device at (source IP, management port) instead
ADV_SOURCE = (DEVICE_ADDRESS[0], 49321)


def make_advertisement(device, *, uptime_ms=120_000):
    """One state-advertisement datagram as lib/net/phone_home_advert.cpp
    packs it: HEARTBEAT first, then SYSTEM_TIME (uptime) and the
    PARAM_EXT_VALUE frames for FW_VERSION and UWB_ROLE (when present),
    back to back in one datagram."""
    d = device.dialect
    frames = [device.heartbeat()]
    system_time = d.MAVLink_system_time_message(
        time_unix_usec=0, time_boot_ms=uptime_ms
    )
    frames.append(bytes(system_time.pack(device._mav)))
    for name in ("FW_VERSION", "UWB_ROLE"):
        if name in device.params:
            frames.append(device._param_value(name, list(device.params).index(name)))
    return b"".join(frames)


def stub_advertisement_parser(
    *,
    system_id=DEVICE_SYSID,
    version=None,
    kind=None,
    uptime_ms=None,
    sleeping=False,
):
    """A parse_advertisement stand-in decoupled from the (possibly
    pre-advertisement) pinned SDK: returns an Advertisement-shaped record
    with the caller-declared enrichment and the real parser's address
    contract (source IP + management port, never the source port)."""

    def parse(data, source_address, *, management_port=3333):
        return SimpleNamespace(
            system_id=system_id,
            component_id=RTLS_COMPONENT_ID,
            system_status=3 if sleeping else 4,
            source_address=source_address,
            firmware_version=version,
            role=None,
            uptime_ms=uptime_ms,
            management_port=management_port,
            address=(source_address[0], management_port),
            sleeping=sleeping,
            kind=kind,
        )

    return parse


try:
    from rtlslink.advertisement import (
        parse_advertisement as real_parse_advertisement,
    )
except ImportError:  # pinned SDK predates the module
    real_parse_advertisement = None

requires_advertisement_sdk = pytest.mark.skipif(
    real_parse_advertisement is None,
    reason="pinned rtls-link SDK predates state advertisements",
)


def test_presence_config_active_defaults():
    assert _presence_config({}) == (2.0, 6.0)
    # explicit values still win, as before
    assert _presence_config({"heartbeat_interval": 1, "device_timeout": 4}) == (
        1.0,
        4.0,
    )


def test_presence_config_passive_defaults():
    # 60 s hello (boards must still learn the server address; legacy
    # boards still get probed) and 30 s timeout = 3x the 10 s
    # advertisement period
    assert _presence_config({"passive": True}) == (60.0, 30.0)


def test_presence_config_passive_respects_explicit_overrides():
    config = {"passive": True, "hello_interval": 20, "device_timeout": 45}
    assert _presence_config(config) == (20.0, 45.0)
    # heartbeat_interval is the ACTIVE-mode knob; passive mode uses
    # hello_interval only
    assert _presence_config({"passive": True, "heartbeat_interval": 1}) == (
        60.0,
        30.0,
    )


async def test_advertisement_discovers_device_and_broadcasts_inf(
    extension, device, builder, hub
):
    """An advertisement alone (no active probe, no param listing) must
    discover the device at its management address, push X-RTLS-INF, and
    enrich the entry with the advertised version/role/uptime."""
    set_fake_param(device, "UWB_ROLE", 1, "uint8")
    device.respond_to_list = False  # nothing but the advertisement
    extension._parse_advertisement = stub_advertisement_parser(
        version="9.9.9", kind="tag", uptime_ms=120_000
    )

    await extension._process_advertisement(make_advertisement(device), ADV_SOURCE, 0.0)

    # discovered at (source IP, management port), not the throwaway source
    recorded = extension._protocol.devices[DEVICE_SYSID]
    assert recorded.address == DEVICE_ADDRESS
    # the discovery pushed the device list, exactly like an active discovery
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 1
    assert set(broadcasts[-1]["status"]) == {str(DEVICE_SYSID)}

    response = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["address"] == list(DEVICE_ADDRESS)
    # advertisement beats the param cache (the raw frames carried the
    # param-store version "1.2.3"; the advertisement record is fresher)
    assert entry["firmwareVersion"] == "9.9.9"
    assert entry["role"] == "tag"
    assert entry["uptimeMs"] == 120_000
    assert entry["sleeping"] is False


async def test_advertisement_refreshes_known_device(extension, device):
    """For an already known device an advertisement is a liveness refresh
    (a heartbeat), not a rediscovery: last_seen moves, no INF push."""
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1
    extension._parse_advertisement = stub_advertisement_parser()

    await extension._process_advertisement(
        make_advertisement(device), ADV_SOURCE, 100.0
    )

    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 100.0
    assert extension._protocol.expire(105.0) == []  # 5 s < 6 s timeout
    assert len(_inf_broadcasts(extension)) == 1  # no gained/lost transition


async def test_advertisement_without_enrichment_keeps_param_path(
    extension, device, builder, hub
):
    """Old advertisement firmware (heartbeat-only datagram) yields a record
    with None enrichment: no uptime is invented and the firmware version
    still comes from the param path."""
    extension._parse_advertisement = stub_advertisement_parser()  # all None

    await extension._process_advertisement(device.heartbeat(), ADV_SOURCE, 0.0)
    # discovery auto-fetched the param list through the fake transport
    response = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["firmwareVersion"] == "1.2.3"  # param path intact
    assert "uptimeMs" not in entry
    assert "role" not in entry


async def test_advertisement_piggybacked_frames_are_dropped(
    extension, device, dialect
):
    """A single valid heartbeat must not smuggle extra frames into the
    protocol: piggybacked HEARTBEAT/PARAM_EXT_VALUE frames for OTHER
    system ids (and non-advertisement message types) are stripped before
    the datagram content reaches the shared protocol core."""
    device.respond_to_list = False
    intruder = FakeDevice(dialect, system_id=99)
    # sysid 99 is already legitimately known via the management channel
    await extension._process_datagram(intruder.heartbeat(), intruder.address, 0.0)
    victim = extension._protocol.devices[99]
    assert victim.last_seen == 0.0
    baseline_params = dict(victim.params)

    extension._parse_advertisement = stub_advertisement_parser()
    datagram = (
        make_advertisement(device)
        + intruder.heartbeat()  # spoofed liveness for sysid 99
        + intruder._param_value("FW_VERSION", 3)  # spoofed param for sysid 99
        + device.named_value_float("rate", 8.0)  # wrong type, even if own sysid
    )
    await extension._process_advertisement(datagram, ADV_SOURCE, 50.0)

    # the advertiser itself went through the normal machinery...
    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 50.0
    # ...but the piggybacked frames changed nothing for sysid 99
    assert victim.last_seen == 0.0
    assert victim.params == baseline_params
    # and the smuggled stats frame never became a stats event
    assert extension._stats == {}


async def test_advertisement_truncated_tail_does_not_poison_protocol_parser(
    extension, device, dialect
):
    """The protocol core's MAVLink parser is stateful across datagrams: raw
    advertisement bytes ending in a truncated frame would leave it
    mid-frame and eat the first frame of the NEXT management datagram.
    The sanitizing re-serialization must isolate it."""
    device.respond_to_list = False
    extension._parse_advertisement = stub_advertisement_parser()

    truncated = make_advertisement(device) + device.heartbeat()[:-3]
    await extension._process_advertisement(truncated, ADV_SOURCE, 0.0)
    assert DEVICE_SYSID in extension._protocol.devices

    # the very next management-channel datagram must still parse whole
    second = FakeDevice(dialect, system_id=DEVICE_SYSID + 1)
    await extension._process_datagram(second.heartbeat(), second.address, 1.0)
    assert DEVICE_SYSID + 1 in extension._protocol.devices


async def test_advertisement_enrichment_pruned_on_loss(extension, device):
    extension._parse_advertisement = stub_advertisement_parser(
        version="9.9.9", uptime_ms=1000
    )
    await extension._process_advertisement(make_advertisement(device), ADV_SOURCE, 0.0)
    assert DEVICE_SYSID in extension._adv

    for event in extension._protocol.expire(1000.0):
        extension._handle_lost(event)
    assert DEVICE_SYSID not in extension._adv


async def test_advertisement_garbage_is_ignored(extension, device):
    # a non-advertisement datagram parses to None -> dropped
    extension._parse_advertisement = lambda data, addr, management_port=3333: None
    await extension._process_advertisement(b"not mavlink", ADV_SOURCE, 0.0)
    assert extension._protocol.devices == {}
    assert _inf_broadcasts(extension) == []

    # a parser blowing up on hostile input must not kill the listener path
    def exploding(data, addr, *, management_port=3333):
        raise ValueError("boom")

    extension._parse_advertisement = exploding
    await extension._process_advertisement(b"\xfd\x00garbage", ADV_SOURCE, 0.0)
    assert extension._protocol.devices == {}


async def test_advertisement_noop_without_parser(extension, device):
    """With the listener disabled (pre-advertisement SDK) the datagram
    path is inert; nothing else in the extension is affected."""
    assert extension._parse_advertisement is None
    await extension._process_advertisement(make_advertisement(device), ADV_SOURCE, 0.0)
    assert extension._protocol.devices == {}


def test_load_advertisement_parser_degrades_without_sdk(monkeypatch):
    """Requirement: an old pinned SDK (no rtlslink.advertisement) logs ONE
    warning and disables the listener instead of breaking the extension."""
    monkeypatch.setitem(sys.modules, "rtlslink.advertisement", None)
    warnings = []
    log = SimpleNamespace(warning=lambda message, *args, **kwargs: warnings.append(message))

    assert _load_advertisement_parser(log) is None

    assert len(warnings) == 1
    assert "advertisement" in warnings[0]
    # a logger-less caller (unit harness) must not crash either
    assert _load_advertisement_parser(None) is None


@requires_advertisement_sdk
async def test_advertisement_full_path_with_real_parser(
    extension, device, builder, hub
):
    """End to end against the real SDK parser (runs once the rtls-link pin
    ships rtlslink.advertisement; skipped on older pins): a firmware-shaped
    advertisement datagram discovers + enriches the device."""
    set_fake_param(device, "UWB_ROLE", 1, "uint8")
    device.respond_to_list = False
    extension._parse_advertisement = real_parse_advertisement

    await extension._process_advertisement(
        make_advertisement(device, uptime_ms=45_000), ADV_SOURCE, 0.0
    )

    response = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["address"] == list(DEVICE_ADDRESS)
    assert entry["firmwareVersion"] == "1.2.3"
    assert entry["role"] == "tag"
    assert entry["uptimeMs"] == 45_000


# ---- passive presence: liveness across the slow hello --------------------


def _passive_extension(extension, dialect, device):
    """Reconfigure the fixture extension the way run() does for
    passive mode: 60 s hello, 30 s timeout, passive keepalive."""
    extension._passive = True
    extension._protocol = RtlsProtocol(
        dialect,
        targets=[device.address],
        heartbeat_interval=60.0,
        device_timeout=30.0,
    )
    return extension


async def test_passive_any_datagram_refreshes_liveness(extension, dialect, device):
    """In passive mode ANY inbound datagram from a known device's address
    counts as proof of life -- the pinned protocol core refreshes
    last_seen on heartbeats only, so without this a board whose
    heartbeats are lost to a contended AP would flap between the 60 s
    hellos. Attribution is by source address, not by protocol events:
    real firmware datagrams exist that yield no event at all (the pn/pe/pd
    position stats and SYSTEM_TIME are dropped by the pinned SDK)."""
    _passive_extension(extension, dialect, device)
    # the fake transport answers the discovery param listing on the REAL
    # clock, which would itself refresh last_seen under passive keepalive
    # and defeat the controlled timeline below
    device.respond_to_list = False
    await _feed_heartbeat(extension, device, now=0.0)

    # a lone stats datagram (no heartbeat) 25 s in refreshes liveness
    await extension._process_datagram(
        device.named_value_float("rate", 8.0), device.address, 25.0
    )
    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 25.0

    # so does a datagram the SDK emits NO event for (a position stat)
    await extension._process_datagram(
        device.named_value_float("pn", 1.25), device.address, 28.0
    )
    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 28.0
    assert extension._protocol.expire(31.0) == []

    # but not the same sysid's frames from a FOREIGN source IP: the
    # refresh is guarded by the known address, and an actual move is
    # left to the heartbeat seam (which migrates the recorded address)
    await extension._process_datagram(
        device.named_value_float("pn", 1.25), ("192.168.4.250", 3333), 30.0
    )
    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 28.0

    # total silence past the timeout still expires the device
    assert [e.kind for e in extension._protocol.expire(59.0)] == ["lost"]


async def test_passive_refresh_not_fooled_by_dhcp_address_reuse(
    extension, dialect, device
):
    """Bench DHCP reassigns IPs across power cycles. When device A's IP is
    reused by device B, B's traffic must refresh B ONLY: liveness is
    attributed by the MAVLink header system id (with the source IP as a
    guard), never by source address — an address-keyed refresh would
    keep ghost A alive forever on B's datagrams."""
    _passive_extension(extension, dialect, device)
    device.respond_to_list = False
    await _feed_heartbeat(extension, device, now=0.0)  # A = sysid 42 at IP X

    # power cycle: the SAME IP now belongs to sysid 43
    successor = FakeDevice(dialect, system_id=DEVICE_SYSID + 1)
    await extension._process_datagram(successor.heartbeat(), DEVICE_ADDRESS, 5.0)
    assert DEVICE_SYSID + 1 in extension._protocol.devices

    # B keeps chattering from the reused address, including datagrams
    # that yield no protocol event (position stats)
    for now in (10.0, 20.0, 28.0):
        await extension._process_datagram(
            successor.named_value_float("pn", 1.0), DEVICE_ADDRESS, now
        )

    assert extension._protocol.devices[DEVICE_SYSID + 1].last_seen == 28.0
    # ghost A was never refreshed by its successor's traffic...
    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 0.0
    # ...so it expires on schedule while B stays alive
    events = extension._protocol.expire(31.0)
    assert [(e.kind, e.system_id) for e in events] == [("lost", DEVICE_SYSID)]
    assert DEVICE_SYSID + 1 in extension._protocol.devices


async def test_active_mode_stats_do_not_refresh_liveness(extension, device):
    """The pinned-protocol truth this feature is built around: only
    HEARTBEATs refresh last_seen in RtlsProtocol.feed; stats/param traffic
    does not. Active mode keeps that behavior byte for byte."""
    await _feed_heartbeat(extension, device, now=0.0)
    await extension._process_datagram(
        device.named_value_float("rate", 8.0), device.address, 5.0
    )

    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 0.0
    # default 6 s timeout: the stats at t=5 did not keep it alive
    assert [e.kind for e in extension._protocol.expire(6.5)] == ["lost"]


async def test_passive_legacy_board_kept_alive_by_own_heartbeats(
    extension, dialect, device
):
    """A LEGACY board (no advertisement firmware) does not flap in passive
    mode: once it learns the server address (any hello), its firmware
    HeartbeatService streams 1 Hz heartbeats to that peer unconditionally,
    and heartbeats refresh last_seen even in the pinned protocol. Fed here
    every 10 s (sparser than reality) across two hello periods."""
    _passive_extension(extension, dialect, device)
    device.respond_to_list = False  # keep the timeline heartbeat-only
    for now in range(0, 130, 10):
        await _feed_heartbeat(extension, device, now=float(now))
        assert extension._protocol.expire(float(now) + 9.0) == []
    assert DEVICE_SYSID in extension._protocol.devices


# ---- passive presence: run() wiring ---------------------------------------


class RunStubHub(StubMessageHub):
    @contextmanager
    def use_message_handlers(self, handlers):
        yield


class RunStubApp:
    def __init__(self):
        self.message_hub = RunStubHub()

    def import_api(self, name):
        return StubBeaconAPI()


async def test_run_applies_passive_presence_config():
    """run() must translate passive:true into the slow hello + 30 s
    timeout on the protocol, and advertisement_port:0 must disable the
    listener socket."""
    ext = RtlsExtension()
    ext.app = app = RunStubApp()
    config = {
        "passive": True,
        "advertisement_port": 0,
        "broadcast": [],
        "devices": [],
    }

    async with trio.open_nursery() as nursery:
        nursery.start_soon(ext.run, app, config, logging.getLogger("test-rtls"))
        with trio.fail_after(5):
            while ext._protocol is None:
                await trio.sleep(0.01)
        assert ext._passive is True
        assert ext._protocol._heartbeat_interval == 60.0
        assert ext._protocol._device_timeout == 30.0
        assert ext._adv_sock is None
        nursery.cancel_scope.cancel()


async def test_run_degrades_gracefully_without_advertisement_sdk(monkeypatch):
    """With an old pinned SDK, run() warns once, skips the listener (and
    its socket bind) and keeps the whole extension operational."""
    monkeypatch.setitem(sys.modules, "rtlslink.advertisement", None)
    ext = RtlsExtension()
    ext.app = app = RunStubApp()
    warnings = []
    log = SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warning=lambda message, *args, **kwargs: warnings.append(message),
    )

    async with trio.open_nursery() as nursery:
        nursery.start_soon(ext.run, app, {"broadcast": [], "devices": []}, log)
        with trio.fail_after(5):
            while ext._protocol is None:
                await trio.sleep(0.01)
        assert ext._parse_advertisement is None
        assert ext._adv_sock is None
        assert len(warnings) == 1
        assert "advertisement" in warnings[0]
        nursery.cancel_scope.cancel()


async def test_run_disables_listener_on_bind_collision():
    """The listener binds WITHOUT SO_REUSEADDR: a port collision must fail
    loudly (EADDRINUSE -> one warning, listener disabled) instead of two
    reuse-enabled sockets silently splitting unicast delivery."""
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("0.0.0.0", 0))  # a REAL socket already owns the port
    port = blocker.getsockname()[1]

    ext = RtlsExtension()
    ext.app = app = RunStubApp()
    # pretend the SDK ships the parser so run() reaches the bind
    ext._parse_advertisement = stub_advertisement_parser()
    warnings = []
    log = SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warning=lambda message, *args, **kwargs: warnings.append(message),
    )

    try:
        async with trio.open_nursery() as nursery:
            nursery.start_soon(
                ext.run,
                app,
                {"advertisement_port": port, "broadcast": [], "devices": []},
                log,
            )
            with trio.fail_after(5):
                while not warnings:
                    await trio.sleep(0.01)
            assert ext._adv_sock is None
            assert len(warnings) == 1
            assert str(port) in warnings[0]
            nursery.cancel_scope.cancel()
    finally:
        blocker.close()


def test_cell_id_from_params_reads_firmware_label():
    from flockwave.server.ext.rtls.extension import _cell_id_from_params

    assert _cell_id_from_params({}) == "default"
    assert _cell_id_from_params({"CELL_ID": "arena-north"}) == "arena-north"
    # legacy speculative name still honored
    assert _cell_id_from_params({"RTLS_CELL_ID": "old"}) == "old"
    assert _cell_id_from_params({"CELL_ID": ""}) == "default"


# ---- X-RTLS-POS position-estimate debug stream ---------------------------


async def _feed_pos(
    extension, device, north, east, down, sigma=-1.0, *, time_boot_ms=0, now=0.0
):
    """Feed one POS_DBG_HZ emit cycle (NAMED_VALUE_FLOAT pn/pe/pd/psig), all
    stamped with the same time_boot_ms as the firmware does. The firmware
    always trails the cycle with psig, sending -1 when the solver had no
    covariance -- the default mirrors that."""
    fields = [("pn", north), ("pe", east), ("pd", down), ("psig", sigma)]
    for name, value in fields:
        await extension._process_datagram(
            device.named_value_float(name, value, time_boot_ms),
            device.address,
            now,
        )


def _pos_broadcasts(extension):
    return [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-POS"
    ]


async def test_pos_broadcast_on_complete_cycle(extension, device):
    await discover(extension, device)
    await _feed_pos(
        extension, device, 1.2044, -0.3506, -0.82, sigma=0.1204,
        time_boot_ms=1234, now=0.0,
    )

    broadcasts = _pos_broadcasts(extension)
    assert broadcasts
    entry = broadcasts[-1]["positions"][str(DEVICE_SYSID)]
    assert entry == {
        "id": DEVICE_SYSID,
        "north": 1.204,
        "east": -0.351,
        "down": -0.82,
        "sigma": 0.12,
        "timeBootMs": 1234,
        "ageMs": 0,
    }


async def test_pos_negative_sigma_omitted(extension, device):
    # the firmware reports psig = -1 when the solver had no covariance
    await discover(extension, device)
    await _feed_pos(
        extension, device, 1.0, 2.0, -0.5, sigma=-1.0, time_boot_ms=10, now=0.0
    )

    entry = _pos_broadcasts(extension)[-1]["positions"][str(DEVICE_SYSID)]
    assert "sigma" not in entry
    assert entry["north"] == 1.0


async def test_pos_incomplete_cycle_does_not_broadcast(extension, device):
    await discover(extension, device)
    # pn/pe only: the NED triple never completes
    for name, value in (("pn", 1.0), ("pe", 2.0)):
        await extension._process_datagram(
            device.named_value_float(name, value, 5), device.address, 0.0
        )
    assert not _pos_broadcasts(extension)

    # a pd from a *different* emit cycle must not complete the stale pn/pe
    await extension._process_datagram(
        device.named_value_float("pd", -0.5, 6), device.address, 0.0
    )
    assert not _pos_broadcasts(extension)


async def test_pos_does_not_pollute_health_stats(extension, device):
    await discover(extension, device)
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=1, now=0.0)

    assert DEVICE_SYSID not in extension._stats
    assert not [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]


async def test_pos_broadcast_throttled(extension, device):
    await discover(extension, device)

    # first complete cycle broadcasts immediately (leading edge)
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=1, now=0.0)
    # a second cycle well within POS_INTERVAL must not broadcast again
    await _feed_pos(extension, device, 1.1, 2.1, -0.5, time_boot_ms=2, now=0.02)
    assert len(_pos_broadcasts(extension)) == 1

    # once the interval elapses, a further cycle broadcasts the latest values
    await _feed_pos(extension, device, 1.5, 2.5, -0.6, time_boot_ms=3, now=0.2)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["positions"][str(DEVICE_SYSID)]["north"] == 1.5


async def test_pos_trailing_edge_flush(extension, device):
    await discover(extension, device)

    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=1, now=0.0)
    # this cycle lands inside the throttle window: cached, not broadcast
    await _feed_pos(extension, device, 1.9, 2.9, -0.7, time_boot_ms=2, now=0.02)
    assert len(_pos_broadcasts(extension)) == 1

    # the run loop's periodic flush pushes the pending snapshot once the
    # window has elapsed, so the last estimate of a burst is never lost
    extension._flush_pending_pos(0.2)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["positions"][str(DEVICE_SYSID)]["north"] == 1.9

    # nothing newer pending: a further flush is a no-op
    extension._flush_pending_pos(0.4)
    assert len(_pos_broadcasts(extension)) == 2


async def test_pos_query_returns_latest_snapshot(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_pos(
        extension, device, 1.0, 2.0, -0.5, sigma=0.2, time_boot_ms=7, now=0.0
    )

    message = make_message(builder, {"type": "X-RTLS-POS"})
    response = await extension._handle_RTLS_POS(message, None, hub)

    assert response.body["type"] == "X-RTLS-POS"
    entry = response.body["positions"][str(DEVICE_SYSID)]
    assert entry["north"] == 1.0
    assert entry["east"] == 2.0
    assert entry["down"] == -0.5
    assert entry["sigma"] == 0.2
    assert entry["ageMs"] >= 0


async def test_pos_query_by_id(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=7, now=0.0)

    message = make_message(builder, {"type": "X-RTLS-POS", "id": DEVICE_SYSID})
    response = await extension._handle_RTLS_POS(message, None, hub)
    assert set(response.body["positions"]) == {str(DEVICE_SYSID)}

    # an unknown device id yields an empty snapshot, not an error
    other = make_message(builder, {"type": "X-RTLS-POS", "id": 99})
    response = await extension._handle_RTLS_POS(other, None, hub)
    assert response.body["positions"] == {}


async def test_pos_query_empty_before_any_estimate(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-POS"})
    response = await extension._handle_RTLS_POS(message, None, hub)
    assert response.body == {"type": "X-RTLS-POS", "positions": {}}


async def test_pos_pruned_on_device_loss(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=7, now=0.0)
    assert DEVICE_SYSID in extension._pos

    events = extension._protocol.expire(time.monotonic() + 1000)
    for event in events:
        extension._handle_lost(event)

    assert DEVICE_SYSID not in extension._pos
    assert DEVICE_SYSID not in extension._pos_wire
    message = make_message(builder, {"type": "X-RTLS-POS"})
    response = await extension._handle_RTLS_POS(message, None, hub)
    assert response.body["positions"] == {}


async def test_pos_nonfinite_ned_invalidates_only_that_cycle(extension, device):
    await discover(extension, device)
    await _feed_pos(
        extension, device, float("nan"), 2.0, -0.5, time_boot_ms=1, now=0.0
    )
    assert not _pos_broadcasts(extension)

    # the next (finite) cycle regroups on its fresh stamp and broadcasts
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=2, now=0.0)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 1
    assert broadcasts[-1]["positions"][str(DEVICE_SYSID)]["north"] == 1.0


async def test_pos_nonfinite_sigma_does_not_blackhole_the_stream(
    extension, device
):
    # a persistently non-finite psig (covariance blow-up) must degrade to
    # "no sigma", not silence the whole stream: the poisoned field still
    # counts for the cycle grouping
    await discover(extension, device)
    await _feed_pos(
        extension, device, 1.0, 2.0, -0.5, sigma=float("inf"),
        time_boot_ms=1, now=0.0,
    )

    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 1
    entry = broadcasts[-1]["positions"][str(DEVICE_SYSID)]
    assert entry["north"] == 1.0
    assert "sigma" not in entry


async def test_pos_late_replay_cannot_regress_a_newer_cycle(extension, device):
    # a delayed datagram of an OLDER cycle must be dropped, not assembled
    # into the current cycle (it would silently regress one coordinate)
    await discover(extension, device)
    for name, value in (("pn", 1.0), ("pe", 2.0), ("pd", -0.5)):
        await extension._process_datagram(
            device.named_value_float(name, value, 200), device.address, 0.0
        )
    # late psig replayed from the previous cycle (stamp 100 < 200)
    await extension._process_datagram(
        device.named_value_float("psig", 0.5, 100), device.address, 0.0
    )
    assert not _pos_broadcasts(extension)

    # the current cycle's own psig still completes it
    await extension._process_datagram(
        device.named_value_float("psig", 0.2, 200), device.address, 0.0
    )
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 1
    entry = broadcasts[-1]["positions"][str(DEVICE_SYSID)]
    assert entry == {
        "id": DEVICE_SYSID,
        "north": 1.0,
        "east": 2.0,
        "down": -0.5,
        "sigma": 0.2,
        "timeBootMs": 200,
        "ageMs": 0,
    }


async def test_pos_newer_stamp_clears_stale_partial_assembly(extension, device):
    # cycle t=100 loses its pd/psig datagrams; the fields of cycle t=1100
    # must not pair with the stale leftovers
    await discover(extension, device)
    for name, value in (("pn", 9.0), ("pe", 9.0)):
        await extension._process_datagram(
            device.named_value_float(name, value, 100), device.address, 0.0
        )
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=1100, now=0.0)

    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 1
    assert broadcasts[-1]["positions"][str(DEVICE_SYSID)]["north"] == 1.0


async def test_pos_reboot_backstep_resets_the_assembly(extension, device):
    # a device reboot rewinds time_boot_ms; the recurring low stamps must
    # start a fresh cycle instead of being dropped as "older" forever
    await discover(extension, device)
    await _feed_pos(
        extension, device, 1.0, 2.0, -0.5, time_boot_ms=600_000, now=0.0
    )
    assert len(_pos_broadcasts(extension)) == 1

    # rebooted: stamps restart near zero (backstep far above the threshold)
    await _feed_pos(extension, device, 3.0, 4.0, -0.7, time_boot_ms=1500, now=0.2)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 2
    entry = broadcasts[-1]["positions"][str(DEVICE_SYSID)]
    assert entry["north"] == 3.0
    assert entry["timeBootMs"] == 1500


async def test_pos_broadcast_does_not_block_on_a_full_hub_queue(
    extension, device
):
    # X-RTLS-POS is emitted from the extension's single receive/expiry
    # loop, so it must never await the hub's bounded TX queue: on overflow
    # the notification is dropped and the stream continues
    await discover(extension, device)
    extension.app.message_hub.full = True
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=1, now=0.0)
    assert not _pos_broadcasts(extension)
    # the extension's own state advanced normally despite the drop
    assert DEVICE_SYSID in extension._pos

    # once the queue drains, the next cycle broadcasts the latest snapshot
    extension.app.message_hub.full = False
    await _feed_pos(extension, device, 1.5, 2.5, -0.6, time_boot_ms=200, now=0.5)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 1
    assert broadcasts[-1]["positions"][str(DEVICE_SYSID)]["north"] == 1.5


async def test_pos_from_unknown_sysid_is_ignored(extension, device):
    # no discovery: an unknown sysid has no `lost` path, so caching it
    # would grow the pos dicts unboundedly (or resurrect a pruned device)
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=1, now=0.0)

    assert not _pos_broadcasts(extension)
    assert not extension._pos
    assert not extension._pos_wire


async def test_inf_site_anchor_list_carries_ned(extension, device, builder, hub):
    # the debug position view plots anchors in the same NED frame as the
    # X-RTLS-POS estimates, so the anchors list carries the native
    # cell-frame coordinates beside the derived GPS position
    add_rtls_cell_params(device)  # tag, role=1
    await discover(extension, device)

    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)

    anchors = {a["id"]: a for a in response.body["anchors"]}
    assert anchors["rtls::default::anchor_0"]["ned"] == {
        "north": -10.0,
        "east": -10.0,
        "down": 0.0,
    }
    assert anchors["rtls::default::anchor_1"]["ned"] == {
        "north": 10.0,
        "east": 10.0,
        "down": -4.8,
    }


# ---- X-RTLS-GEO ---------------------------------------------------------


def wire_devices(extension, *devices):
    """Fake transport routing by address, for tests that talk to more
    than one scripted device at once."""
    table = {d.address: d for d in devices}

    async def fake_send(payload, address):
        target = table.get(tuple(address))
        if target is None:
            return
        for reply in target.handle(payload):
            await extension._process_datagram(
                reply, target.address, time.monotonic()
            )

    extension._send = fake_send


def make_second_tag(dialect, *, system_id=DEVICE_SYSID + 1):
    second = FakeDevice(dialect, system_id=system_id)
    second.address = ("192.168.4.%d" % (system_id % 250), 3333)
    add_rtls_cell_params(second)
    return second


async def geo_message(extension, builder, hub, body):
    message = make_message(builder, {"type": "X-RTLS-GEO", **body})
    return await extension._handle_RTLS_GEO(message, None, hub)


async def adopt_from(extension, builder, hub, sysid=DEVICE_SYSID):
    """Adopts a tag's geometry as the canonical one (the bootstrap step
    of the server-owned-truth model)."""
    response = await geo_message(
        extension, builder, hub, {"op": "adopt", "reference": sysid}
    )
    assert response.body["type"] == "X-RTLS-GEO", response.body
    return response


async def test_geo_check_consistent(extension, device, dialect, builder, hub):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    response = await geo_message(
        extension, builder, hub, {"op": "check"}
    )

    body = response.body
    assert body["type"] == "X-RTLS-GEO"
    assert body["op"] == "check"
    assert body["cell"] == "default"
    assert body["consistent"] is True
    # EVERY tag is a target now, the adopted one included
    assert body["devices"][str(DEVICE_SYSID)] == {"status": "consistent"}
    assert body["devices"][str(DEVICE_SYSID + 1)] == {"status": "consistent"}


async def test_geo_check_reports_deltas_and_missing(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    set_fake_param(device, "POS_YAW_DEG", 90.0, "real32")
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")  # a moved anchor
    # no POS_YAW_DEG on the second tag at all
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    response = await geo_message(
        extension, builder, hub, {"op": "check"}
    )

    body = response.body
    assert body["consistent"] is False
    entry = body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "mismatch"
    assert entry["deltas"]["UWB_AN1_X"] == {"expected": 10.0, "actual": 10.5}
    assert entry["missing"] == ["POS_YAW_DEG"]


async def test_geo_check_float_tolerance(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    set_fake_param(device, "POS_YAW_DEG", 90.0, "real32")
    second = make_second_tag(dialect)
    # representation noise well under the default 1e-4 tolerance
    set_fake_param(second, "POS_YAW_DEG", 90.00003, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    response = await geo_message(
        extension, builder, hub, {"op": "check"}
    )

    assert response.body["consistent"] is True


async def test_geo_check_incomplete_target(extension, device, builder, hub):
    add_rtls_cell_params(device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    # a cache-only tag whose snapshot never gained the origin params or
    # the anchor table (the count itself matches, so nothing mismatches)
    stub = add_anchor_device(extension, 77, role=1, uwb_mac=254)
    set_cached_param(stub, "UWB_AN_COUNT", 2, "uint8")

    response = await geo_message(
        extension, builder, hub, {"op": "check"}
    )

    entry = response.body["devices"]["77"]
    assert entry["status"] == "incomplete"
    assert "ORIGIN_LAT_E7" in entry["missing"]
    assert response.body["consistent"] is False


async def test_geo_check_without_canonical_geometry_is_rejected(
    extension, builder, hub
):
    response = await geo_message(extension, builder, hub, {"op": "check"})
    assert response.body["type"] == "ACK-NAK"
    assert "canonical" in response.body["reason"]


async def test_geo_invalid_op_is_rejected(extension, builder, hub):
    response = await geo_message(extension, builder, hub, {"op": "explode"})
    assert response.body["type"] == "ACK-NAK"
    assert "op" in response.body["reason"]


async def test_geo_non_tag_target_is_an_error_entry(
    extension, device, builder, hub
):
    add_rtls_cell_params(device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    add_anchor_device(extension, 90, role=2, uwb_mac=1)

    response = await geo_message(
        extension,
        builder,
        hub,
        {"op": "check", "ids": [90]},
    )

    entry = response.body["devices"]["90"]
    assert entry["status"] == "error"
    assert "Not a tag" in entry["detail"]


async def test_geo_sync_writes_reboots_and_converges(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    set_fake_param(device, "POS_YAW_DEG", 90.0, "real32")
    second = make_second_tag(dialect)
    set_fake_param(second, "POS_YAW_DEG", 0.0, "real32")
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    set_fake_param(second, "ORIGIN_ALT_MM", 20000, "int32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    resets = []
    extension._geo_reset = resets.append

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "synced"
    assert entry["failures"] == {}
    assert set(entry["written"]) == {"POS_YAW_DEG", "UWB_AN1_X", "ORIGIN_ALT_MM"}
    assert "ORIGIN_LAT_E7" in entry["skipped"]
    assert entry["rebooted"] is True
    assert resets == [second.address[0]]

    # the device-side wire store converged to the reference geometry
    assert struct.unpack("<f", second.params["UWB_AN1_X"][0][:4])[0] == 10.0
    assert struct.unpack("<i", second.params["ORIGIN_ALT_MM"][0][:4])[0] == 10000

    check = await geo_message(
        extension, builder, hub, {"op": "check"}
    )
    assert check.body["consistent"] is True


async def test_geo_sync_count_is_written_last(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN_COUNT", 1, "uint8")
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    writes = []
    original_handle = second.handle

    def recording_handle(data):
        for message in dialect.MAVLink(None).parse_buffer(bytes(data)) or []:
            if message.get_type() == "PARAM_EXT_SET":
                writes.append(message.param_id)
        return original_handle(data)

    second.handle = recording_handle
    extension._geo_reset = lambda address: None

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "synced"
    # a partially synced registry must never declare a window onto a
    # half-written anchor table: the count trails every other write
    assert writes[-1] == "UWB_AN_COUNT"
    assert "UWB_AN1_X" in writes[:-1]


async def test_geo_sync_rejected_write_means_partial_and_no_reboot(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    second.set_result = PARAM_ACK_FAILED
    resets = []
    extension._geo_reset = resets.append

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "partial"
    assert "UWB_AN1_X" in entry["failures"]
    assert "rejected by device" in entry["failures"]["UWB_AN1_X"]
    assert entry["rebooted"] is False
    assert "writes failed" in entry["rebootDetail"]
    assert resets == []


async def test_geo_sync_no_changes_needed_skips_reboot(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    resets = []
    extension._geo_reset = resets.append

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "synced"
    assert entry["written"] == []
    assert entry["rebooted"] is False
    assert "no changes" in entry["rebootDetail"]
    assert resets == []


async def test_geo_sync_reboot_false_skips_reset(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    resets = []
    extension._geo_reset = resets.append

    response = await geo_message(
        extension,
        builder,
        hub,
        {"op": "sync", "reboot": False},
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "synced"
    assert "rebooted" not in entry
    assert resets == []


async def test_geo_sync_timeout_aborts_that_device_only(
    extension, device, dialect, builder, hub, autojump_clock
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    third = make_second_tag(dialect, system_id=DEVICE_SYSID + 2)
    set_fake_param(third, "UWB_AN1_X", 11.5, "real32")
    third.respond_to_set = False
    wire_devices(extension, device, second, third)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    await extension._process_datagram(
        third.heartbeat(), third.address, time.monotonic()
    )

    extension._geo_reset = lambda address: None

    response = await geo_message(
        extension,
        builder,
        hub,
        {"op": "sync", "timeout": 1},
    )

    healthy = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert healthy["status"] == "synced"
    silent = response.body["devices"][str(DEVICE_SYSID + 2)]
    assert silent["status"] == "error"
    assert "timeout" in silent["detail"]


async def test_geo_concurrent_sync_is_refused(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    extension._geo_sync_running = True
    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )
    assert response.body["type"] == "ACK-NAK"
    assert "in progress" in response.body["reason"]

    # and the latch is released once a real sync finishes
    extension._geo_sync_running = False
    extension._geo_reset = lambda address: None
    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )
    assert response.body["type"] == "X-RTLS-GEO"
    assert extension._geo_sync_running is False


async def test_geo_adopt_refuses_an_incomplete_snapshot(
    extension, device, builder, hub
):
    add_rtls_cell_params(device)
    await discover(extension, device)
    # simulate a lossy dump: the registry claims more params than cached
    extension._get_devices()[DEVICE_SYSID].param_count = 99

    response = await geo_message(
        extension, builder, hub, {"op": "adopt", "reference": DEVICE_SYSID}
    )
    assert response.body["type"] == "ACK-NAK"
    assert "snapshot incomplete" in response.body["reason"]


async def test_geo_incomplete_target_snapshot_is_flagged(
    extension, device, builder, hub
):
    # a lossy-dump hole in an OPTIONAL param must not read as "consistent"
    add_rtls_cell_params(device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    stub = add_anchor_device(extension, 78, role=1, uwb_mac=254)
    stub.param_count = 99

    response = await geo_message(
        extension, builder, hub, {"op": "check"}
    )
    entry = response.body["devices"]["78"]
    assert entry["status"] == "incomplete"
    assert "snapshot incomplete" in entry["detail"]


async def test_cell_id_change_rehomes_the_source(extension, device):
    add_rtls_cell_params(device)
    set_fake_param(device, "CELL_ID", "a", "custom")
    await discover(extension, device)
    assert extension._anchor_cell_sources == {"a": DEVICE_SYSID}

    await extension.set_param(DEVICE_SYSID, "CELL_ID", "b")

    # the old cell id must not keep pointing at the re-homed device
    # (it would render a phantom cell with the new cell's geometry)
    assert extension._anchor_cell_sources == {"b": DEVICE_SYSID}


async def test_geo_sync_withholds_count_after_failed_writes(
    extension, device, dialect, builder, hub
):
    # ref count 2, target count 1: the count write is due — but earlier
    # rejected table writes must withhold it, or the next reboot would
    # activate a window onto a mixed anchor table
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN_COUNT", 1, "uint8")
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    second.set_result = PARAM_ACK_FAILED
    writes = []
    original_handle = second.handle

    def recording_handle(data):
        for message in dialect.MAVLink(None).parse_buffer(bytes(data)) or []:
            if message.get_type() == "PARAM_EXT_SET":
                writes.append(message.param_id)
        return original_handle(data)

    second.handle = recording_handle
    extension._geo_reset = lambda address: None

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "partial"
    assert entry["failures"]["UWB_AN_COUNT"].startswith("withheld"), entry
    assert "UWB_AN_COUNT" not in writes  # never even sent to the device


async def test_geo_incomplete_snapshot_with_full_geometry_is_accepted(
    extension, device, dialect, builder, hub
):
    # a lossy dump that only lost NON-geometry params must not block the
    # check: with every geometry param (optional ones included) cached,
    # the comparison is fully determined
    add_rtls_cell_params(device)
    set_fake_param(device, "POS_YAW_DEG", 90.0, "real32")
    set_fake_param(device, "CELL_ID", "default", "custom")
    second = make_second_tag(dialect)
    set_fake_param(second, "POS_YAW_DEG", 90.0, "real32")
    set_fake_param(second, "CELL_ID", "default", "custom")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    for sysid in (DEVICE_SYSID, DEVICE_SYSID + 1):
        extension._get_devices()[sysid].param_count = 99

    response = await geo_message(
        extension, builder, hub, {"op": "check"}
    )

    assert response.body["type"] == "X-RTLS-GEO", response.body
    assert response.body["consistent"] is True, response.body


async def test_param_write_locks_are_pruned(extension, device, autojump_clock):
    await discover(extension, device)

    await extension.set_param(DEVICE_SYSID, "UWB_CH", 7)
    assert extension._param_write_locks == {}

    device.respond_to_set = False
    with pytest.raises(trio.TooSlowError):
        await extension.set_param(DEVICE_SYSID, "UWB_CH", 8, timeout=1)
    assert extension._param_write_locks == {}


async def test_geo_sync_detects_concurrent_drift(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    # a concurrent writer changes an ALREADY-SYNCED param mid-sync: the
    # device pushes a param_value for UWB_AN0_X (written earlier in the
    # sync order) while the server writes UWB_AN1_X
    original_handle = second.handle

    def drifting_handle(data):
        out = original_handle(data)
        parser = dialect.MAVLink(None)
        for message in parser.parse_buffer(bytes(data)) or []:
            if (
                message.get_type() == "PARAM_EXT_SET"
                and message.param_id == "UWB_AN1_X"
            ):
                set_fake_param(second, "UWB_AN0_X", 99.0, "real32")
                index = list(second.params).index("UWB_AN0_X")
                out.append(second._param_value("UWB_AN0_X", index))
        return out

    second.handle = drifting_handle
    resets = []
    extension._geo_reset = resets.append

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "partial", entry
    assert "post-write verification" in entry["failures"]["UWB_AN0_X"], entry
    assert entry["rebooted"] is False  # a drifted geometry must not activate
    assert resets == []


async def test_geo_adopt_refuses_an_anchor(extension, device, builder, hub):
    add_rtls_cell_params(device)
    await discover(extension, device)
    add_anchor_device(extension, 91, role=2, uwb_mac=1)

    response = await geo_message(
        extension, builder, hub, {"op": "adopt", "reference": 91}
    )
    assert response.body["type"] == "ACK-NAK"
    assert "not a tag" in response.body["reason"]


async def test_geo_sync_survives_mid_sync_rediscovery(
    extension, device, dialect, builder, hub
):
    # the target is lost + rediscovered mid-sync: acks mirror into the
    # NEW device object, and the post-write verification must read that
    # one — the detached old object would flag phantom drift forever
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    swapped = []
    original_handle = second.handle

    def swapping_handle(data):
        if not swapped:
            old = extension._protocol.devices[DEVICE_SYSID + 1]
            clone = SimpleNamespace(
                system_id=old.system_id,
                address=old.address,
                last_seen=old.last_seen,
                params=dict(old.params),
                param_types=dict(old.param_types),
                param_count=old.param_count,
            )
            extension._protocol.devices[DEVICE_SYSID + 1] = clone
            swapped.append(True)
        return original_handle(data)

    second.handle = swapping_handle
    resets = []
    extension._geo_reset = resets.append

    response = await geo_message(
        extension, builder, hub, {"op": "sync"}
    )

    entry = response.body["devices"][str(DEVICE_SYSID + 1)]
    assert entry["status"] == "synced", entry
    assert entry["rebooted"] is True
    assert resets == [second.address[0]]


async def test_refill_repairs_geometry_consistency_params(extension, device):
    # a dump-loss hole in an OPTIONAL geometry param (POS_YAW_DEG, CELL_ID,
    # a bias) kept X-RTLS-GEO reporting the tag 'incomplete' forever: the
    # knowability gate refuses to trust the omission and the refill never
    # repaired non-identity holes. These params are identity now.
    add_rtls_cell_params(device)
    set_fake_param(device, "POS_YAW_DEG", 30.0, "real32")
    set_fake_param(device, "CELL_ID", "default", "custom")
    device.drop_from_list = {"POS_YAW_DEG", "UWB_AN1_BIAS_M"}
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert "POS_YAW_DEG" not in cached.params

    device.read_requests.clear()
    await extension._poll_param_refill(
        time.monotonic() + REFILL_INITIAL_DELAY + 1
    )

    assert sorted(device.read_requests) == ["POS_YAW_DEG", "UWB_AN1_BIAS_M"]
    assert "POS_YAW_DEG" in cached.params
    assert "UWB_AN1_BIAS_M" in cached.params
    assert DEVICE_SYSID not in extension._refill


def add_stub_tag(extension, system_id, *, yaw=None, cell="default", an1_x=10.0):
    """Cache-only tag with a full, count-complete geometry registry."""
    tag = SimpleNamespace(
        system_id=system_id,
        address=("192.168.4.%d" % (system_id % 250), 3333),
        last_seen=time.monotonic(),
        params={},
        param_types={},
        param_count=None,
    )
    entries = [
        ("UWB_ROLE", 1, "uint8"),
        ("ORIGIN_LAT_E7", 413900000, "int32"),
        ("ORIGIN_LON_E7", 21500000, "int32"),
        ("ORIGIN_ALT_MM", 10000, "int32"),
        ("CELL_ID", cell, "custom"),
        ("UWB_AN_COUNT", 1, "uint8"),
        ("UWB_AN0_X", an1_x, "real32"),
        ("UWB_AN0_Y", -10.0, "real32"),
        ("UWB_AN0_Z", 0.0, "real32"),
        ("UWB_AN0_MAC", 1, "uint16"),
    ]
    if yaw is not None:
        entries.append(("POS_YAW_DEG", yaw, "real32"))
    for name, value, type_name in entries:
        set_cached_param(tag, name, value, type_name)
    tag.param_count = len(tag.params)
    extension._protocol.devices[system_id] = tag
    return tag


class StubUAV:
    def __init__(self, params):
        self._params = params

    async def get_parameter(self, name, fetch=False):
        if name not in self._params:
            raise KeyError(name)
        return self._params[name]


def wire_verify_fleet(extension, device, dialect, *, drone_params=None):
    """Two consistent wire tags paired to two stub drones with healthy
    solver stats; returns the second tag for perturbation."""
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    wire_devices(extension, device, second)

    defaults = {"EK3_SRC1_YAW": 9.0, "EK3_SRC_VC_YAW": 40.0}
    uavs = {
        "05": StubUAV(dict(drone_params or defaults)),
        "06": StubUAV(dict(defaults)),
    }
    extension.app.find_uav_by_id = lambda uav_id: uavs.get(uav_id)
    extension.app.object_registry = SimpleNamespace(
        ids_by_type=lambda _type: list(uavs)
    )
    extension._uav_map = {DEVICE_SYSID: "05", DEVICE_SYSID + 1: "06"}
    for sysid in (DEVICE_SYSID, DEVICE_SYSID + 1):
        extension._stats[sysid] = {
            "id": sysid,
            "solveRateHz": 12.5,
            "solvePct": 97.0,
            "fixAgeMs": 80,
            "clockPpm": 1.2,
            "anchorMask": 0b1111,
            "clockSyncOk": True,
        }
        extension._stats_at[sysid] = time.monotonic()
    return second


async def verify_message(extension, builder, hub, body=None):
    message = make_message(builder, {"type": "X-RTLS-VERIFY", **(body or {})})
    return await extension._handle_RTLS_VERIFY(message, None, hub)


async def test_verify_rejects_an_empty_fleet(extension, builder, hub):
    extension.app.object_registry = SimpleNamespace(ids_by_type=lambda _type: [])

    response = await verify_message(extension, builder, hub)

    assert response.body["passed"] is False
    pairing = next(
        rule for rule in response.body["rules"] if rule["id"] == "pairing"
    )
    assert pairing["severity"] == "error"
    assert pairing["status"] == "fail"
    assert "no online tags" in pairing["detail"]
    assert "no drones are known" in pairing["detail"]


async def test_verify_passes_on_a_healthy_fleet(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(extension, device, dialect)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    response = await verify_message(extension, builder, hub)

    body = response.body
    assert body["type"] == "X-RTLS-VERIFY"
    assert body["passed"] is True, body["rules"]
    statuses = {rule["id"]: rule["status"] for rule in body["rules"]}
    assert statuses == {
        "geometry": "pass",
        "firmware": "pass",
        "pairing": "pass",
        "yaw-source": "pass",
        "uwb": "pass",
        "params": "skipped",
    }


async def test_verify_flags_wrong_yaw_source(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(
        extension,
        device,
        dialect,
        drone_params={"EK3_SRC1_YAW": 1.0, "EK3_SRC_VC_YAW": 40.0},
    )
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)

    response = await verify_message(extension, builder, hub)

    body = response.body
    assert body["passed"] is False
    rule = next(r for r in body["rules"] if r["id"] == "yaw-source")
    assert rule["status"] == "fail"
    assert "virtual compass" in rule["detail"]
    assert rule["devices"]["05"]["yawSource"] == 1.0


async def test_verify_flags_geometry_drift_and_missing_stats(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(extension, device, dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    del extension._stats[DEVICE_SYSID + 1]  # and one tag went silent

    response = await verify_message(extension, builder, hub)

    body = response.body
    assert body["passed"] is False
    by_id = {rule["id"]: rule for rule in body["rules"]}
    assert by_id["geometry"]["status"] == "fail"
    assert by_id["uwb"]["status"] == "fail"
    assert "no telemetry" in by_id["uwb"]["detail"]


async def test_verify_in_depth_reports_param_diffs_as_warnings(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(extension, device, dialect)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    # give both drones the full in-depth set; one WPNAV_SPEED differs
    from flockwave.server.ext.rtls.verify import IN_DEPTH_PARAMS

    base = dict.fromkeys(IN_DEPTH_PARAMS, 1.0)
    base.update({"EK3_SRC1_YAW": 9.0, "EK3_SRC_VC_YAW": 40.0})
    drone_a = dict(base)
    drone_b = dict(base)
    drone_b["WPNAV_SPEED"] = 2.0
    extension.app.find_uav_by_id = lambda uav_id: {
        "05": StubUAV(drone_a),
        "06": StubUAV(drone_b),
    }.get(uav_id)

    response = await verify_message(extension, builder, hub, {"inDepth": True})

    body = response.body
    rule = next(r for r in body["rules"] if r["id"] == "params")
    assert rule["status"] == "fail"
    assert rule["severity"] == "warning"
    assert "WPNAV_SPEED" in rule["diffs"]
    # warnings never block the flight verdict
    assert body["passed"] is True, body["rules"]


async def test_verify_concurrent_run_is_refused(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(extension, device, dialect)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    extension._verify_running = True
    response = await verify_message(extension, builder, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "in progress" in response.body["reason"]


async def test_verify_flags_silent_telemetry_as_stale(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(extension, device, dialect)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    now = time.monotonic()
    extension._stats_at[DEVICE_SYSID] = now
    extension._stats_at[DEVICE_SYSID + 1] = now - 60  # stream went silent

    response = await verify_message(extension, builder, hub)

    rule = next(r for r in response.body["rules"] if r["id"] == "uwb")
    assert rule["status"] == "fail"
    assert "silent" in rule["detail"]


async def test_verify_vc_yaw_is_a_wrapped_angle(
    extension, device, dialect, builder, hub
):
    second = wire_verify_fleet(extension, device, dialect)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    await adopt_from(extension, builder, hub)
    now = time.monotonic()
    extension._stats_at[DEVICE_SYSID] = now
    extension._stats_at[DEVICE_SYSID + 1] = now
    # 0 and 360 are the same yaw: must NOT fail the fleet
    extension.app.find_uav_by_id = lambda uav_id: {
        "05": StubUAV({"EK3_SRC1_YAW": 9.0, "EK3_SRC_VC_YAW": 0.0}),
        "06": StubUAV({"EK3_SRC1_YAW": 9.0, "EK3_SRC_VC_YAW": 360.0}),
    }.get(uav_id)

    response = await verify_message(extension, builder, hub)

    rule = next(r for r in response.body["rules"] if r["id"] == "yaw-source")
    assert rule["status"] == "pass", rule


# ---- X-RTLS-GEO rolling-summary fit -------------------------------------


def _setup_anchor_calibration_fleet(extension, device):
    """Install one tag cell and the eight four-tripod anchor identities."""
    add_rtls_cell_params(device)
    set_fake_param(device, "UWB_AN_COUNT", 8, "uint8")
    positions = (
        (0.0, 0.0, 0.0),
        (20.0, 0.0, 0.0),
        (0.0, 16.0, 0.0),
        (20.0, 16.0, 0.0),
        (0.0, 0.0, -2.5),
        (20.0, 0.0, -2.5),
        (0.0, 16.0, -2.5),
        (20.0, 16.0, -2.5),
    )
    for index, position in enumerate(positions):
        for axis, value in zip("XYZ", position, strict=True):
            set_fake_param(device, f"UWB_AN{index}_{axis}", value, "real32")
        set_fake_param(device, f"UWB_AN{index}_MAC", index + 1, "uint16")
        set_fake_param(device, f"UWB_AN{index}_BIAS_M", 0.0, "real32")
        add_anchor_device(
            extension,
            70 + index,
            role=2 if index == 0 else 3,
            uwb_mac=index + 1,
        )
    return positions


def _rolling_summary(positions, *, sequence=1, count=80):
    """Test description of the seven responder-owned A0 spokes."""
    return {
        "version": 1,
        "sequence": sequence,
        "timeBootMs": 1000 * sequence,
        "ranges": [
            {
                "anchorIndex": index,
                "distanceM": sum(value * value for value in positions[index])
                ** 0.5,
                "madM": 0.005,
                "count": count,
            }
            for index in range(1, 8)
        ],
    }


def _emit_responder_summaries(extension, summary):
    """Emit one local A0-peer generation for every described responder."""
    from flockwave.server.ext.rtls.fit import on_twr_summary

    for item in summary["ranges"]:
        index = item["anchorIndex"]
        on_twr_summary(
            extension,
            70 + index,
            {
                "version": summary["version"],
                "sequence": summary["sequence"] + index,
                "validMask": item.get("validMask", 0x01),
                "timeBootMs": summary["timeBootMs"] + index,
                "ranges": [
                    {
                        "peerMac": item.get("measuredPeerMac", 1),
                        "distanceM": item["distanceM"],
                        "madM": item["madM"],
                        "count": item["count"],
                    }
                ],
            },
            time.monotonic(),
        )


async def _fit_after_summary(extension, builder, hub, body, summary):
    """Start the request, then emit one generation from each responder."""
    response = {}

    async def request():
        message = make_message(builder, {"type": "X-RTLS-GEO", **body})
        response["value"] = await extension._handle_RTLS_GEO(message, None, hub)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(request)
        await trio.testing.wait_all_tasks_blocked()
        _emit_responder_summaries(extension, summary)
    return response["value"]


async def test_geo_strict_fit_waits_for_and_pins_a_complete_summary(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    response = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict"},
        _rolling_summary(positions),
    )

    body = response.body
    assert body["selectedModel"] == "strict"
    assert body["summary"]["captureId"] == 1
    assert body["summary"]["validMask"] == 0xFE
    assert len(body["summary"]["sources"]) == 7
    assert body["strict"]["accepted"]
    # numeric recovery is proven to tolerance by the unit fit tests; here we
    # only confirm the integration wires a sane, apply-ready payload (exact
    # equality on an LM-solver output would be needlessly brittle)
    assert abs(body["strict"]["parameters"]["lengthM"] - 20.0) < 1e-3
    assert body["applyGeometry"]["POS_YAW_DEG"] == 0.0
    assert body["applyGeometry"]["UWB_AN0_X"] == 0.0
    assert abs(body["applyGeometry"]["UWB_AN7_Z"] + 2.5) < 1e-3


async def test_geo_strict_fit_honors_explicit_cell_with_multiple_stored_cells(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    extension._geo_canonical["other"] = {
        **extension._geo_canonical["default"],
        "CELL_ID": "other",
    }

    response = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict", "cell": "default"},
        _rolling_summary(positions),
    )

    assert response.body["type"] == "X-RTLS-GEO"
    assert response.body["cell"] == "default"
    assert response.body["selectedModel"] == "strict"


async def test_geo_refined_fit_reuses_the_requested_pinned_summary(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    strict = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict"},
        _rolling_summary(positions, sequence=7),
    )
    capture_id = strict.body["summary"]["captureId"]

    # New telemetry may arrive, but the opt-in refined fit compares the exact
    # snapshot the operator reviewed.
    _emit_responder_summaries(
        extension, _rolling_summary(positions, sequence=80)
    )
    message = make_message(
        builder,
        {
            "type": "X-RTLS-GEO",
            "op": "fit",
            "mode": "refined",
            "captureId": capture_id,
        },
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)

    assert response.body["summary"]["captureId"] == capture_id
    assert response.body["refined"]["model"] == "refined"
    assert response.body["selectedModel"] is None
    assert response.body["comparison"]["meaningfulImprovement"] is False
    assert response.body["applyGeometry"] is None


async def test_geo_fit_rejects_a_low_quality_summary_actionably(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    response = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict"},
        _rolling_summary(positions, count=5),
    )

    assert response.body["type"] == "ACK-NAK"
    assert "A1 summary has 5 samples" in response.body["reason"]


async def test_geo_fit_times_out_actionably_without_any_summary(
    extension, device, builder, hub
):
    _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    message = make_message(
        builder,
        {"type": "X-RTLS-GEO", "op": "fit", "mode": "strict", "timeout": 0.2},
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)

    assert response.body["type"] == "ACK-NAK"
    assert "no fresh rolling TWR summary from A1" in response.body["reason"]


async def test_geo_fit_rejects_a_stale_summary(extension, device, builder, hub):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    # All responder summaries predate the request and must not satisfy it.
    _emit_responder_summaries(extension, _rolling_summary(positions))

    message = make_message(
        builder,
        {"type": "X-RTLS-GEO", "op": "fit", "mode": "strict", "timeout": 0.2},
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)

    assert response.body["type"] == "ACK-NAK"
    assert "no fresh rolling TWR summary" in response.body["reason"]


async def test_geo_fit_rejects_an_unsupported_summary_version(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    summary = _rolling_summary(positions)
    summary["version"] = 2
    response = await _fit_after_summary(
        extension, builder, hub, {"op": "fit", "mode": "strict"}, summary
    )

    assert response.body["type"] == "ACK-NAK"
    assert "version 2 is unsupported" in response.body["reason"]


async def test_geo_fit_names_the_missing_spokes(extension, device, builder, hub):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    # A7 fails to publish after the request.
    summary = _rolling_summary(positions)
    summary["ranges"] = summary["ranges"][:-1]
    response = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict", "timeout": 0.2},
        summary,
    )

    assert response.body["type"] == "ACK-NAK"
    assert "no fresh rolling TWR summary from A7" in response.body["reason"]


async def test_geo_fit_rejects_summary_peers_outside_the_cell(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    # A5 reports a peer other than A0.
    summary = _rolling_summary(positions)
    summary["ranges"][4]["measuredPeerMac"] = 0x0063
    response = await _fit_after_summary(
        extension, builder, hub, {"op": "fit", "mode": "strict"}, summary
    )

    assert response.body["type"] == "ACK-NAK"
    assert "A5 summary does not contain exactly its A0 range" in response.body["reason"]


async def test_geo_fit_requires_the_four_tripod_anchor_count(
    extension, device, builder, hub
):
    add_rtls_cell_params(device)  # the stock two-anchor test cell
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    message = make_message(
        builder, {"type": "X-RTLS-GEO", "op": "fit", "mode": "strict"}
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)

    assert response.body["type"] == "ACK-NAK"
    assert "exactly 8 anchors" in response.body["reason"]


async def test_geo_fit_requires_one_unambiguous_initiator(
    extension, device, builder, hub
):
    _setup_anchor_calibration_fleet(extension, device)
    # a second online device also claims the A0 MAC as initiator
    add_anchor_device(extension, 90, role=2, uwb_mac=1)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    message = make_message(
        builder, {"type": "X-RTLS-GEO", "op": "fit", "mode": "strict"}
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)

    assert response.body["type"] == "ACK-NAK"
    assert "multiple online devices claim the configured A0 MAC" in response.body["reason"]


async def test_geo_fit_rejects_a_responder_with_the_wrong_role(
    extension, device, builder, hub
):
    _setup_anchor_calibration_fleet(extension, device)
    # A5 has the configured MAC but claims to be another initiator. Accepting
    # it would wait for telemetry that this role is deliberately unable to send.
    set_cached_param(extension._protocol.devices[75], "UWB_ROLE", 2, "uint8")
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    message = make_message(
        builder, {"type": "X-RTLS-GEO", "op": "fit", "mode": "strict"}
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)

    assert response.body["type"] == "ACK-NAK"
    assert "A5 has UWB role 2; expected responder" in response.body["reason"]


async def test_geo_refined_fit_requires_the_pinned_capture(
    extension, device, builder, hub
):
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    # refined before any strict fit
    message = make_message(
        builder,
        {"type": "X-RTLS-GEO", "op": "fit", "mode": "refined", "captureId": 1},
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "strict fit first" in response.body["reason"]

    strict = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict"},
        _rolling_summary(positions, sequence=3),
    )
    capture_id = strict.body["summary"]["captureId"]
    # a stale GUI asking for a generation that is no longer pinned
    message = make_message(
        builder,
        {
            "type": "X-RTLS-GEO",
            "op": "fit",
            "mode": "refined",
            "captureId": capture_id + 1,
        },
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "no longer the pinned" in response.body["reason"]


async def test_geo_fit_session_dies_with_its_a0(extension, device, builder, hub):
    from rtlslink.protocol import ProtocolEvent

    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    strict = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict"},
        _rolling_summary(positions),
    )
    assert extension._geo_fit_session is not None
    capture_id = strict.body["summary"]["captureId"]

    extension._handle_lost(ProtocolEvent("lost", 70))

    assert extension._geo_fit_session is None
    assert 70 not in extension._twr_summaries
    message = make_message(
        builder,
        {
            "type": "X-RTLS-GEO",
            "op": "fit",
            "mode": "refined",
            "captureId": capture_id,
        },
    )
    response = await extension._handle_RTLS_GEO(message, None, hub)
    assert response.body["type"] == "ACK-NAK"


async def test_geo_fit_exposes_only_the_constrained_models(
    extension, device, builder, hub
):
    """Identifiability guard: A0-star radii cannot support per-anchor moves
    or per-plane skew, so the protocol must not offer them."""
    positions = _setup_anchor_calibration_fleet(extension, device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)

    strict = await _fit_after_summary(
        extension,
        builder,
        hub,
        {"op": "fit", "mode": "strict"},
        _rolling_summary(positions),
    )
    assert "moves" not in strict.body
    assert set(strict.body["strict"]["parameters"]) == {
        "lengthM",
        "widthM",
        "heightM",
    }

    message = make_message(
        builder,
        {
            "type": "X-RTLS-GEO",
            "op": "fit",
            "mode": "refined",
            "captureId": strict.body["summary"]["captureId"],
        },
    )
    refined = await extension._handle_RTLS_GEO(message, None, hub)
    assert set(refined.body["refined"]["parameters"]) == {
        "bottomLengthM",
        "bottomWidthM",
        "topLengthM",
        "topWidthM",
        "heightM",
        "angleDeg",
    }


async def test_geo_adopt_unanimous_fleet_needs_no_reference(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )

    response = await geo_message(extension, builder, hub, {"op": "adopt"})
    assert response.body["type"] == "X-RTLS-GEO", response.body
    assert response.body["op"] == "adopt"

    check = await geo_message(extension, builder, hub, {"op": "check"})
    assert check.body["consistent"] is True


async def test_geo_adopt_refuses_a_disagreeing_fleet(
    extension, device, dialect, builder, hub
):
    add_rtls_cell_params(device)
    second = make_second_tag(dialect)
    set_fake_param(second, "UWB_AN1_X", 10.5, "real32")
    wire_devices(extension, device, second)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )

    response = await geo_message(extension, builder, hub, {"op": "adopt"})
    assert response.body["type"] == "ACK-NAK"
    assert "disagrees" in response.body["reason"]
    # explicit adoption still works — the operator names the truth
    response = await geo_message(
        extension, builder, hub, {"op": "adopt", "reference": DEVICE_SYSID}
    )
    assert response.body["type"] == "X-RTLS-GEO"


async def test_geo_canonical_geometry_persists(
    extension, device, builder, hub, dialect, tmp_path
):
    extension._geo_store_path = tmp_path / "geometry.json"
    add_rtls_cell_params(device)
    await discover(extension, device)
    await adopt_from(extension, builder, hub)
    assert (tmp_path / "geometry.json").exists()

    # a fresh extension instance (server restart) reads the same store
    second_ext = RtlsExtension()
    second_ext._protocol = RtlsProtocol(dialect, targets=[device.address])
    second_ext.app = StubApp()

    async def fake_send(payload, address):
        for reply in device.handle(payload):
            await second_ext._process_datagram(
                reply, device.address, time.monotonic()
            )

    second_ext._send = fake_send
    second_ext._geo_store_path = tmp_path / "geometry.json"
    await second_ext._process_datagram(
        device.heartbeat(), device.address, time.monotonic()
    )
    message = make_message(builder, {"type": "X-RTLS-GEO", "op": "check"})
    response = await second_ext._handle_RTLS_GEO(message, None, hub)
    assert response.body["type"] == "X-RTLS-GEO", response.body
    assert response.body["consistent"] is True


async def test_geo_sync_without_canonical_is_rejected(
    extension, device, builder, hub
):
    add_rtls_cell_params(device)
    await discover(extension, device)
    response = await geo_message(extension, builder, hub, {"op": "sync"})
    assert response.body["type"] == "ACK-NAK"
    assert "canonical" in response.body["reason"]
