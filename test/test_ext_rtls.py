"""Tests for the client message API of the rtls extension.

The protocol core itself (and its value codec) is covered by the test
suite of the standalone ``rtls-link`` SDK; here the message handlers
are exercised against that sans-IO core with a fake transport: the
extension's ``_send`` is replaced by a function that hands every
outbound datagram to a scripted fake device, whose MAVLink replies are
pushed back through ``_process_datagram``. No sockets are involved.
"""

import logging
import math
import socket
import struct
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
import rtlslink as rtlslink_module
import trio
import trio.testing
from rtls_support import (
    DEVICE_ADDRESS,
    DEVICE_SYSID,
    FakeDevice,
    StubApp,
    StubBeaconAPI,
    StubMessageHub,
    discover,
    make_message,
    set_fake_param,
)
from rtlslink.dialect import load_dialect
from rtlslink.protocol import (
    PARAM_ACK_FAILED,
    PARAM_ACK_VALUE_UNSUPPORTED,
    PARAM_TYPE_UINT8,
    ProtocolEvent,
    RtlsProtocol,
    encode_param_value,
    param_type_from_name,
)

from flockwave.server.ext.rtls.extension import (
    REFILL_INITIAL_DELAY,
    REFILL_MAX_ATTEMPTS,
    REFILL_READ_SPACING,
    SLEEP_PIN_TIMEOUT,
    STATS_STALE_AFTER,
    RtlsExtension,
    _stats_json,
)
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder


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


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


async def test_run_without_app_fails_fast(dialect):
    """A standalone harness that forgets ext.app must fail loudly at
    run() instead of silently dropping every stats/OTA broadcast (a real
    e2e-harness bug this guard exists for)."""
    ext = RtlsExtension()
    assert ext.app is None
    with pytest.raises(RuntimeError, match="without an app"):
        await ext.run(None, {}, None)


def set_cached_param(device, name, value, param_type_name):
    """Write a parameter into a bare device's server-side cache (a
    SimpleNamespace standing in for an RtlsDevice)."""
    param_type = param_type_from_name(param_type_name)
    device.params[name] = encode_param_value(value, param_type)
    device.param_types[name] = param_type


#: the cell code every device of the verify fleet runs, its anchors and the
#: survey they are on: four, which a position takes
SESSION_CODE = 0x123456
SURVEY_ID = 4
VERIFY_ANCHORS = (200, 201, 202, 203)


def add_rtls_cell_params(device, *, role=1, uwb_mac=254):
    """Give a FakeDevice a role, its own MAC and the cell origin; an anchor
    also the two-row MAC table of its stored session. A tag holds no table."""
    params = [
        ("UWB_ROLE", role, "uint8"),
        ("UWB_MAC", uwb_mac, "uint16"),
        ("ORIGIN_LAT_E7", 413900000, "int32"),
        ("ORIGIN_LON_E7", 21500000, "int32"),
        ("ORIGIN_ALT_MM", 10000, "int32"),
    ]
    if role != 1:
        params += [
            ("UWB_AN_COUNT", 2, "uint8"),
            ("UWB_AN0_MAC", 1, "uint16"),
            ("UWB_AN1_MAC", 2, "uint16"),
        ]
    for name, value, param_type in params:
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
    assert response.body == {
        "type": "X-RTLS-INF",
        "status": {},
        "anchors": [],
        "survey": {
            "id": None,
            "anchors": 0,
            "notSurveyed": [],
            "otherSurvey": [],
            "excluding": {},
            "rangeDeviation": [],
            "rangeDanger": [],
        },
        "anchorLock": {
            "locked": False,
            "lockedCount": 0,
            "anchorCount": 0,
            "unlocked": [],
            "showGate": {
                "required": True,
                "refused": True,
                "reason": "no anchors locked; lock them in Control first",
            },
        },
        "cellSettings": {
            "rateHz": None,
            "blockMs": None,
            "txGain": None,
            "reference": None,
            "differ": [],
            "unknown": [],
        },
    }


async def test_unlisted_board_is_found_from_its_stream(dialect, device):
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[])
    ext.app = StubApp()
    ext._send = lambda payload, address: trio.lowlevel.checkpoint()

    await ext._process_datagram(device.heartbeat(), device.address, time.monotonic())

    assert device.system_id in ext._protocol.devices
    polled = ext._protocol.poll(time.monotonic() + 60)
    assert device.address in [address for _, address in polled]


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


async def test_invalid_role_does_not_register_anchor_beacons(extension, device):
    add_rtls_cell_params(device, role=99)
    extension._beacon_api = StubBeaconAPI()
    await discover(extension, device)
    assert extension._beacon_api.beacons == {}


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
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)

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

    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)

    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["name"] == "RTLS anchor A1"


async def test_refill_backs_off_and_gives_up_at_the_retry_cap(extension, device):
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

    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)
    assert device.read_requests == []
    assert DEVICE_SYSID not in extension._refill


async def test_refill_ignores_lost_non_identity_params(extension, device):
    # a lossy dump that only lost params irrelevant to naming
    # must not trigger any targeted reads
    add_anchor_identity_params(device)
    device.drop_from_list = {"POS_X", "UWB_CH"}
    await discover(extension, device)
    device.read_requests.clear()

    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)
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

    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)
    assert device.read_requests == []
    assert DEVICE_SYSID not in extension._refill


async def test_refill_restores_a_tags_origin(extension, device):
    add_rtls_cell_params(device)
    device.drop_from_list = {"ORIGIN_LAT_E7"}
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert "ORIGIN_LAT_E7" not in cached.params

    device.read_requests.clear()
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)

    # CELL_ID and POS_YAW_DEG are identity params too; this fake registry
    # has neither, so the first round requests them (fruitlessly) — and
    # once the answered reads make the snapshot count-complete again,
    # absence is authoritative and the refill entry is dropped
    assert sorted(device.read_requests) == ["CELL_ID", "ORIGIN_LAT_E7", "POS_YAW_DEG"]
    assert "ORIGIN_LAT_E7" in cached.params
    assert DEVICE_SYSID not in extension._refill


async def test_a_tag_is_not_asked_for_an_anchor_table(extension, device):
    add_rtls_cell_params(device)
    device.drop_from_list = {"POS_X"}
    await discover(extension, device)

    device.read_requests.clear()
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)

    assert not [name for name in device.read_requests if name.startswith("UWB_AN")]


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
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)
    # no targeted reads were needed: the re-dump filled the snapshot
    assert device.read_requests == []
    assert "UWB_ROLE" in cached.params
    assert "UWB_AN1_MAC" in cached.params
    assert DEVICE_SYSID not in extension._refill


async def test_refill_repairs_legacy_tag_without_role_param(extension, device):
    # a legacy tag (no UWB_ROLE param at all) cannot be classified until its
    # snapshot is count-complete — even a lost NON-identity param (POS_X)
    # keeps it unclassified, so the refill must re-dump rather than issue
    # targeted reads for names it cannot know
    add_rtls_cell_params(device)
    device.params.pop("UWB_ROLE")
    device.drop_from_list = {"ORIGIN_LAT_E7", "POS_X"}
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]

    device.drop_from_list = set()  # the retransmitted dump gets through
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)

    assert device.read_requests == []
    assert "ORIGIN_LAT_E7" in cached.params
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
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)
    assert device.read_requests == ["UWB_MAC", "UWB_AN0_MAC", "UWB_AN1_MAC"]
    # the reads of one round are spaced out, not blasted as one burst
    # the firmware's management TX queue would drop replies from
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert len(gaps) == 2
    assert all(gap >= REFILL_READ_SPACING for gap in gaps)


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


async def test_a_lane_gap_is_logged_even_from_a_lost_device(extension, device, caplog):
    """A stall longer than device_timeout ends while its board counts as lost."""
    extension.log = logging.getLogger("test.rtls")
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)
    later = time.monotonic() + 60
    extension._expire_devices(later)
    assert DEVICE_SYSID in extension._lost
    await extension._process_datagram(
        device.named_value_float("gapcomms", 7300.0, 81_000), device.address, later
    )
    (record,) = [r for r in caplog.records if "stalled" in r.message]
    assert f"sysid {DEVICE_SYSID}: comms lane stalled for 7300 ms" in record.message
    assert "uptime 81.0 s" in record.message


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
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)  # a tag
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
            builder,
            {
                "type": "X-RTLS-OTA",
                "id": DEVICE_SYSID,
                "image": str(image),
                "role": "tag",
            },
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
    extension._ota_upgrade = (
        lambda address, image_path, *, timeout=10.0, on_progress=None: "9.9.9"
    )

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
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)  # a tag
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
                builder,
                {
                    "type": "X-RTLS-OTA",
                    "id": DEVICE_SYSID,
                    "image": str(image),
                    "role": "tag",
                },
            )
            response = await extension._handle_RTLS_OTA(start, None, hub)
            assert response.body["job"]["status"] == "running"

            again = make_message(
                builder,
                {
                    "type": "X-RTLS-OTA",
                    "id": DEVICE_SYSID,
                    "image": str(image),
                    "role": "tag",
                },
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
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)  # a tag
    await discover(extension, device)

    image = tmp_path / "firmware.bin"
    image.write_bytes(b"\x00" * 16)

    def failing_upgrade(address, image_path, *, timeout=10.0, on_progress=None):
        raise RuntimeError("device went away")

    extension._ota_upgrade = failing_upgrade

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        start = make_message(
            builder,
            {
                "type": "X-RTLS-OTA",
                "id": DEVICE_SYSID,
                "image": str(image),
                "role": "tag",
            },
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


async def _inf_status(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    return response.body["status"]


async def test_lost_device_stays_listed_offline(extension, device, builder, hub):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)
    entry = (await _inf_status(extension, builder, hub))[str(DEVICE_SYSID)]
    assert entry["online"] is True
    assert entry["age"] >= 0.0

    # the device goes silent past its timeout
    events = extension._expire_devices(time.monotonic() + 1000)
    assert [event.kind for event in events] == ["lost"]

    entry = (await _inf_status(extension, builder, hub))[str(DEVICE_SYSID)]
    assert entry["online"] is False
    later = extension._inf_status(time.monotonic() + 1000)[str(DEVICE_SYSID)]
    assert later["age"] >= 999.0
    assert entry["role"] == "anchor-responder"
    assert entry["address"] == list(DEVICE_ADDRESS)
    # ...but nothing treats it as live
    assert DEVICE_SYSID not in extension._protocol.devices

    # it comes back as the same entry, live again
    await discover(extension, device)
    status = await _inf_status(extension, builder, hub)
    assert list(status) == [str(DEVICE_SYSID)]
    assert status[str(DEVICE_SYSID)]["online"] is True
    assert DEVICE_SYSID not in extension._lost


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
    # then runs the periodic flush, which broadcasts the device offline
    events = extension._expire_devices(1000.0)
    assert [event.kind for event in events] == ["lost"]
    await extension._flush_pending_inf(now=1000.0)

    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["online"] is False


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


# ---- tag<->drone association (the tag's flight-controller claim) --------


def _set_uav_ids(extension, uav_ids):
    """Install a fake UAV-id-by-system-id feed (the seam through which the
    extension reads the mavlink extension's API)."""
    extension._uav_ids_by_system_id = lambda: uav_ids


def _claim_flight_controller(device, system_id, state):
    set_fake_param(device, "FC_SYS_ID", system_id, "uint8")
    set_fake_param(device, "FC_STATE", state, "uint8")


async def _inf_entry(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    return response.body["status"][str(DEVICE_SYSID)]


async def test_uav_is_the_drone_the_tag_names(extension, device, builder, hub):
    _claim_flight_controller(device, 5, 1)
    await discover(extension, device)

    # the server has not heard from that drone yet
    assert "uav" not in await _inf_entry(extension, builder, hub)

    _set_uav_ids(extension, {5: "05", 6: "06"})
    assert extension._refresh_uav_map() is True

    entry = await _inf_entry(extension, builder, hub)
    assert entry["uav"] == "05"
    assert entry["flightController"] == {"systemId": 5, "state": "live"}


async def test_uav_stays_while_the_drone_sleeps(extension, device, builder, hub):
    _claim_flight_controller(device, 5, 1)
    _set_uav_ids(extension, {5: "05"})
    await discover(extension, device)

    set_fake_param(device, "SLEEP", 1, "uint8")
    await extension._process_datagram(
        device.heartbeat(), device.address, time.monotonic()
    )

    entry = await _inf_entry(extension, builder, hub)
    assert entry["sleeping"] is True
    assert entry["uav"] == "05"
    assert entry["flightController"] == {"systemId": 5, "state": "remembered"}


async def test_ambiguous_claim_names_no_uav(extension, device, builder, hub):
    _claim_flight_controller(device, 5, 3)
    _set_uav_ids(extension, {5: "05"})
    await discover(extension, device)

    entry = await _inf_entry(extension, builder, hub)
    assert "uav" not in entry
    assert entry["flightController"]["state"] == "ambiguous"


async def test_association_change_pushes_inf(extension, device):
    _claim_flight_controller(device, 5, 1)
    await _feed_heartbeat(extension, device, now=0.0)
    assert len(_inf_broadcasts(extension)) == 1

    # the drone becoming known is a mapping change: the poll in the
    # protocol loop pushes the device list (throttled, like gained/lost)
    _set_uav_ids(extension, {5: "05"})
    await extension._poll_uav_map(now=2.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 2
    assert broadcasts[-1]["status"][str(DEVICE_SYSID)]["uav"] == "05"

    # an unchanged association stays quiet
    await extension._poll_uav_map(now=4.0)
    assert len(_inf_broadcasts(extension)) == 2

    # a claim change alone pushes too
    _claim_flight_controller(device, 5, 2)
    await extension._process_datagram(
        device._param_value("FC_STATE", list(device.params).index("FC_STATE")),
        device.address,
        5.0,
    )
    await extension._poll_uav_map(now=6.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == 3
    entry = broadcasts[-1]["status"][str(DEVICE_SYSID)]
    assert entry["flightController"] == {"systemId": 5, "state": "remembered"}


def test_uav_ids_empty_without_mavlink_api(extension):
    # no mavlink extension (or one that is not loaded) leaves every device
    # unassociated instead of failing
    assert extension._mavlink_api is None
    assert extension._uav_ids_by_system_id() == {}
    extension._mavlink_api = SimpleNamespace(loaded=False)
    assert extension._uav_ids_by_system_id() == {}


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
    assert stats_broadcasts[-1]["stats"][str(DEVICE_SYSID)]["solveRateHz"] == 10.0


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


async def test_lost_device_stats_kept_stale_until_forgotten(
    extension, device, builder, hub
):
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=0.0)

    # the snapshot is reported before the device is lost
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert "stale" not in response.body["stats"][str(DEVICE_SYSID)]

    # drive the protocol loop's expiry path: the device times out and its
    # last snapshot stays answerable, marked stale
    extension._expire_devices(time.monotonic() + 1000)
    response = await extension._handle_RTLS_STATS(message, None, hub)
    entry = response.body["stats"][str(DEVICE_SYSID)]
    assert entry["stale"] is True
    assert entry["solveRateHz"] == 8.0

    # ...and is pushed once, so listening clients learn it went stale
    await extension._flush_pending_stats(now=10.0)
    pushed = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert pushed[-1]["stats"][str(DEVICE_SYSID)]["stale"] is True

    # forgetting the device drops the snapshot and its bookkeeping
    assert extension.forget_device(DEVICE_SYSID) == {"forgotten": True}
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert response.body["stats"] == {}
    assert DEVICE_SYSID not in extension._last_stats_broadcast
    assert DEVICE_SYSID not in extension._last_stats_sent

    # a fresh report after the device returns is no longer stale
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=20.0)
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert "stale" not in response.body["stats"][str(DEVICE_SYSID)]


# ---- cluster clock estimate ----


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


def _cluster_clock_driver(monkeypatch):
    """A cluster clock estimate and a function feeding it one cluster-clock
    report received at a given wall time."""
    from flockwave.server.ext.rtls.cluster_clock import ClusterClock

    now = [0.0]
    monkeypatch.setattr(
        "flockwave.server.ext.rtls.cluster_clock.time.time", lambda: now[0]
    )
    clock = ClusterClock()

    def report(system_id, cluster_seconds, received_at, clkok=1.0):
        now[0] = received_at
        hi = float(int(cluster_seconds // 4096) * 4096)
        clock.on_stats(
            system_id, {"clkok": clkok, "clkh": hi, "clks": cluster_seconds - hi}
        )

    return clock, report


async def test_cluster_clock_fed_by_fresh_stats_without_device_writes(
    extension, device, monkeypatch
):
    """A clkok stats snapshot feeds the exported estimate; stale cluster time
    does not, and nothing is written to the device."""
    writes = []

    async def set_param(*args, **kwds):
        writes.append(args)

    monkeypatch.setattr(extension, "set_param", set_param)
    await discover(extension, device)
    clock = extension.exports()["cluster_clock"]

    await _feed_stats(extension, device, dict(CLUSTER_STATS, clkok=0.0), now=0.0)
    assert clock.estimate() is None

    before = time.time()
    await _feed_stats(extension, device, CLUSTER_STATS, now=0.0)
    after = time.time()
    assert before - 120.5 <= clock.estimate() <= after - 120.5
    assert writes == []


def test_cluster_clock_follows_least_delayed_report(monkeypatch):
    """Reports from three tags arrive with a varying latency: the estimate is
    the least delayed report of the last 20 s, and maps wall time to cluster
    microseconds and back."""
    import random

    clock, report = _cluster_clock_driver(monkeypatch)
    rng = random.Random(3)
    wall_minus_cluster = 1_752_000_000.0

    for step in range(3 * 75):
        sampled_at = wall_minus_cluster + 100.0 + step / 3
        latency = 0.08 if step == 0 else 0.004 + rng.expovariate(1 / 0.04)
        report(step % 3 + 1, sampled_at - wall_minus_cluster, sampled_at + latency)

    assert 0.004 <= clock.estimate() - wall_minus_cluster < 0.006
    assert clock.epoch == 0

    wall = wall_minus_cluster + 200.0
    cluster_us = clock.cluster_us_at(wall)
    assert cluster_us == pytest.approx(200e6 - 5e3, abs=1e3)
    assert clock.wall_at(cluster_us) == pytest.approx(wall, abs=1e-6)


def test_cluster_clock_restart_starts_a_new_timebase(monkeypatch):
    """A cluster time far off the estimate's prediction, confirmed by the
    next distinct sample of the same device, re-seeds the estimate and bumps
    the epoch."""
    clock, report = _cluster_clock_driver(monkeypatch)
    t0 = 1_752_000_000.0

    report(1, 100.0, t0)
    report(1, 101.0, t0 + 1)
    assert clock.estimate() == pytest.approx(t0 - 100.0)

    report(1, 2.0, t0 + 2)
    assert clock.epoch == 0
    assert clock.estimate() == pytest.approx(t0 - 100.0)

    report(1, 2.5, t0 + 2.5)
    assert clock.epoch == 1
    assert clock.estimate() == pytest.approx(t0)


def test_cluster_clock_restart_clears_every_pending_deviation(monkeypatch):
    """A deviation recorded against the old timebase must not count as the
    first of two against the new one."""
    clock, report = _cluster_clock_driver(monkeypatch)
    t0 = 1_752_000_000.0

    report(1, 100.0, t0)
    report(2, 3000.0, t0)  # device 2 deviates once against the first timebase
    report(1, 2000.0, t0 + 0.5)
    report(1, 2000.5, t0 + 1)  # device 1 confirms a restart
    assert clock.epoch == 1

    report(2, 3500.0, t0 + 1.5)  # device 2's first deviation against the new one
    assert clock.epoch == 1


def test_cluster_clock_ignores_repeated_accumulated_snapshot(monkeypatch):
    """Other NAMED_VALUE_FLOAT fields repeat the cached clkh/clks pair; a
    stale pair arriving long after its sample is not a deviation."""
    clock, report = _cluster_clock_driver(monkeypatch)
    t0 = 1_752_000_000.0

    report(1, 120.5, t0)
    report(1, 120.5, t0 + 6.0)
    report(1, 120.5, t0 + 7.0)
    assert clock.epoch == 0
    assert clock.estimate() == pytest.approx(t0 - 120.5)


def test_cluster_clock_keeps_its_timebase_across_a_torn_clock_pair(monkeypatch):
    """clkh and clks are separate frames and the snapshot pairs the latest of
    each. A lost clkh frame at a 4096 s boundary pairs the old clkh with the
    new clks until the next report; crossing a boundary, the new clkh pairs
    with the old clks until the clks frame arrives. Neither is a restart."""
    from flockwave.server.ext.rtls.cluster_clock import ClusterClock

    t0 = 1_752_000_000.0
    now = [t0]
    monkeypatch.setattr(
        "flockwave.server.ext.rtls.cluster_clock.time.time", lambda: now[0]
    )
    clock = ClusterClock()

    def report(at, clkh, clks):
        now[0] = t0 + at
        clock.on_stats(1, {"clkok": 1.0, "clkh": clkh, "clks": clks})

    report(0.0, 0.0, 4095.0)
    report(1.0, 0.0, 0.0)  # clkh frame lost at the boundary
    report(1.5, 4096.0, 0.5)
    report(4096.5, 4096.0, 4095.5)
    report(4097.0, 8192.0, 4095.5)  # clkh lands before its clks
    report(4097.2, 8192.0, 0.2)
    assert clock.epoch == 0


@pytest.mark.parametrize("old_clks", [1753.1, 0.4])
def test_cluster_clock_keeps_its_timebase_when_a_tag_resets_its_clock(
    monkeypatch, old_clks
):
    """A tag whose estimator resets sends clkh=0,
    clks=0 and only then clkok=0, so two snapshots pair the zeroed halves
    with the previous clkok=1. Seen on the bench as a -5849 s restart of the
    whole estimate followed by a +5849 s one."""
    from flockwave.server.ext.rtls.cluster_clock import ClusterClock

    t0 = 1_752_000_000.0
    now = [t0]
    monkeypatch.setattr(
        "flockwave.server.ext.rtls.cluster_clock.time.time", lambda: now[0]
    )
    clock = ClusterClock()

    def report(at, system_id, clkh, clks, clkok=1.0):
        now[0] = t0 + at
        clock.on_stats(system_id, {"clkok": clkok, "clkh": clkh, "clks": clks})

    report(0.0, 2, 4096.0, old_clks + 1.0)
    report(0.5, 1, 4096.0, old_clks + 1.5)
    report(1.5, 1, 0.0, old_clks + 1.5)
    report(1.52, 1, 0.0, 0.0)
    report(1.54, 1, 0.0, 0.0, clkok=0.0)
    report(2.0, 2, 4096.0, old_clks + 3.0)
    report(3.0, 1, 4096.0, old_clks + 4.0)

    assert clock.epoch == 0
    assert clock.estimate() == pytest.approx(t0 - 4096.0 - old_clks - 1.0)


def test_cluster_clock_restart_needs_a_sample_that_kept_running(monkeypatch):
    """Two deviating samples restart the timebase only if the second one
    advanced from the first by the elapsed wall time."""
    clock, report = _cluster_clock_driver(monkeypatch)
    t0 = 1_752_000_000.0

    report(1, 5000.0, t0)
    report(1, 3.0, t0 + 1)
    report(1, 1.0, t0 + 2)  # deviates again, but ran backwards
    report(1, 900.0, t0 + 3)  # deviates again, but jumped ahead
    assert clock.epoch == 0

    report(1, 901.1, t0 + 4)
    assert clock.epoch == 1
    assert clock.estimate() == pytest.approx(t0 + 4 - 901.1)


async def test_cluster_clock_forgets_a_lost_device(extension, device):
    """A lost device's last sample is dropped, so its first report after
    rediscovery is evaluated even if it repeats the old pair."""
    await discover(extension, device)
    await _feed_stats(extension, device, CLUSTER_STATS, now=0.0)
    clock = extension._cluster_clock
    assert DEVICE_SYSID in clock._last_cluster_seconds

    events = extension._protocol.expire(time.monotonic() + 1000)
    for event in events:
        extension._handle_lost(event)
    assert DEVICE_SYSID not in clock._last_cluster_seconds


# ---- periodic management stream: uptime and flight controller -----------
#
# Besides heartbeats and stats, every board's management stream carries a
# SYSTEM_TIME (uptime) every few seconds and, on tags, unsolicited
# PARAM_EXT_VALUE frames for FC_SYS_ID / FC_STATE, each from the board's
# management address like any other datagram.


def system_time(device, uptime_ms):
    message = device.dialect.MAVLink_system_time_message(
        time_unix_usec=0, time_boot_ms=uptime_ms
    )
    return bytes(message.pack(device._mav))


def streamed_param(device, name):
    return device._param_value(name, list(device.params).index(name))


async def test_streamed_uptime_reported_and_pruned_on_loss(
    extension, device, builder, hub
):
    await discover(extension, device)
    await extension._process_datagram(
        system_time(device, 120_000), device.address, time.monotonic()
    )
    message = make_message(builder, {"type": "X-RTLS-INF"})

    response = await extension._handle_RTLS_INF(message, None, hub)
    assert response.body["status"][str(DEVICE_SYSID)]["uptimeMs"] == 120_000

    extension._handle_lost(ProtocolEvent("lost", DEVICE_SYSID))
    assert DEVICE_SYSID not in extension._uptime_ms


async def test_uptime_of_unknown_device_is_ignored(extension, device):
    await extension._process_datagram(
        system_time(device, 120_000), device.address, time.monotonic()
    )

    assert extension._uptime_ms == {}


async def test_sleeping_tag_streams_its_flight_controller(
    extension, device, builder, hub
):
    """A drone asleep since before the server started never answers with a
    useful dump in time; its streamed FC params alone must name the flight
    controller it carries."""
    set_fake_param(device, "SLEEP", 1, "uint8")
    set_fake_param(device, "FC_SYS_ID", 8, "uint8")
    set_fake_param(device, "FC_STATE", 2, "uint8")
    device.respond_to_list = False

    await discover(extension, device)
    for name in ("FC_SYS_ID", "FC_STATE"):
        await extension._process_datagram(
            streamed_param(device, name), device.address, time.monotonic()
        )

    response = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["sleeping"] is True
    assert "uav" not in entry
    assert entry["flightController"] == {"systemId": 8, "state": "remembered"}


async def test_streamed_fc_state_updates_the_claim(extension, device, builder, hub):
    set_fake_param(device, "FC_SYS_ID", 8, "uint8")
    set_fake_param(device, "FC_STATE", 2, "uint8")
    await discover(extension, device)
    message = make_message(builder, {"type": "X-RTLS-INF"})

    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["flightController"] == {"systemId": 8, "state": "remembered"}

    set_fake_param(device, "FC_STATE", 1, "uint8")
    await extension._process_datagram(
        streamed_param(device, "FC_STATE"), device.address, time.monotonic()
    )

    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["flightController"] == {"systemId": 8, "state": "live"}


async def test_flight_controller_reads_remembered_once_the_tag_sleeps(
    extension, device, builder, hub
):
    set_fake_param(device, "FC_SYS_ID", 8, "uint8")
    set_fake_param(device, "FC_STATE", 1, "uint8")
    await discover(extension, device)
    message = make_message(builder, {"type": "X-RTLS-INF"})

    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["flightController"] == {"systemId": 8, "state": "live"}

    # the cached FC_STATE still says live until the next streamed one
    set_fake_param(device, "SLEEP", 1, "uint8")
    await extension._process_datagram(
        device.heartbeat(), device.address, time.monotonic()
    )

    response = await extension._handle_RTLS_INF(message, None, hub)
    entry = response.body["status"][str(DEVICE_SYSID)]
    assert entry["flightController"] == {"systemId": 8, "state": "remembered"}


async def test_tag_without_flight_controller_identity_omits_it(
    extension, device, builder, hub
):
    await discover(extension, device)

    response = await extension._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert "flightController" not in response.body["status"][str(DEVICE_SYSID)]


async def test_stats_do_not_refresh_liveness(extension, device):
    """Only HEARTBEATs refresh last_seen in RtlsProtocol.feed; stats/param
    traffic does not."""
    await _feed_heartbeat(extension, device, now=0.0)
    await extension._process_datagram(
        device.named_value_float("rate", 8.0), device.address, 5.0
    )

    assert extension._protocol.devices[DEVICE_SYSID].last_seen == 0.0
    # default 6 s timeout: the stats at t=5 did not keep it alive
    assert [e.kind for e in extension._protocol.expire(6.5)] == ["lost"]


# ---- run() wiring ---------------------------------------------------------


class RunStubHub(StubMessageHub):
    @contextmanager
    def use_message_handlers(self, handlers):
        yield


class RunStubApp:
    def __init__(self):
        self.message_hub = RunStubHub()

    def import_api(self, name):
        return StubBeaconAPI()


def _free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("0.0.0.0", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


async def test_run_binds_the_management_socket_to_listen_port():
    """Board streams and replies to the server's own requests arrive on the
    one socket bound to listen_port."""
    port = _free_udp_port()
    ext = RtlsExtension()
    ext.app = app = RunStubApp()

    async with trio.open_nursery() as nursery:
        nursery.start_soon(
            ext.run, app, {"listen_port": port}, logging.getLogger("test-rtls")
        )
        with trio.fail_after(5):
            while ext._protocol is None:
                await trio.sleep(0.01)
        assert ext._sock.getsockname()[1] == port
        nursery.cancel_scope.cancel()


async def test_run_fails_loudly_when_listen_port_is_taken():
    """No SO_REUSEADDR: a second process on the port must stop the
    extension instead of silently splitting the boards' streams."""
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("0.0.0.0", 0))
    port = blocker.getsockname()[1]
    ext = RtlsExtension()
    ext.app = app = RunStubApp()

    try:
        with pytest.raises(RuntimeError, match=str(port)):
            await ext.run(app, {"listen_port": port}, logging.getLogger("test-rtls"))
    finally:
        blocker.close()
    assert ext._protocol is None


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
        extension,
        device,
        1.2044,
        -0.3506,
        -0.82,
        sigma=0.1204,
        time_boot_ms=1234,
        now=0.0,
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
    await _feed_pos(extension, device, float("nan"), 2.0, -0.5, time_boot_ms=1, now=0.0)
    assert not _pos_broadcasts(extension)

    # the next (finite) cycle regroups on its fresh stamp and broadcasts
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=2, now=0.0)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 1
    assert broadcasts[-1]["positions"][str(DEVICE_SYSID)]["north"] == 1.0


async def test_pos_nonfinite_sigma_does_not_blackhole_the_stream(extension, device):
    # a persistently non-finite psig (covariance blow-up) must degrade to
    # "no sigma", not silence the whole stream: the poisoned field still
    # counts for the cycle grouping
    await discover(extension, device)
    await _feed_pos(
        extension,
        device,
        1.0,
        2.0,
        -0.5,
        sigma=float("inf"),
        time_boot_ms=1,
        now=0.0,
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
    await _feed_pos(extension, device, 1.0, 2.0, -0.5, time_boot_ms=600_000, now=0.0)
    assert len(_pos_broadcasts(extension)) == 1

    # rebooted: stamps restart near zero (backstep far above the threshold)
    await _feed_pos(extension, device, 3.0, 4.0, -0.7, time_boot_ms=1500, now=0.2)
    broadcasts = _pos_broadcasts(extension)
    assert len(broadcasts) == 2
    entry = broadcasts[-1]["positions"][str(DEVICE_SYSID)]
    assert entry["north"] == 3.0
    assert entry["timeBootMs"] == 1500


async def test_pos_broadcast_does_not_block_on_a_full_hub_queue(extension, device):
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


# ---- multi-device helpers ----------------------------------------------


def wire_devices(extension, *devices):
    """Fake transport routing by address, for tests that talk to more
    than one scripted device at once."""
    table = {d.address: d for d in devices}

    async def fake_send(payload, address):
        target = table.get(tuple(address))
        if target is None:
            return
        for reply in target.handle(payload):
            await extension._process_datagram(reply, target.address, time.monotonic())

    extension._send = fake_send


def make_second_tag(dialect, *, system_id=DEVICE_SYSID + 1):
    second = FakeDevice(dialect, system_id=system_id)
    second.address = ("192.168.4.%d" % (system_id % 250), 3333)
    add_rtls_cell_params(second)
    return second


def test_configured_devices_accepts_smp_port_entries():
    """The devices config takes plain strings (hardware, SMP fixed at
    :1337) and {address, smp_port} objects (simulated devices: shared
    loopback host, per-device SMP endpoints from the sim manifest)."""
    from flockwave.server.ext.rtls.extension import _configured_devices

    targets, smp_ports = _configured_devices(
        ["10.0.0.7", {"address": "127.0.0.1:3433", "smp_port": 1437}], 3333
    )
    assert targets == [("10.0.0.7", 3333), ("127.0.0.1", 3433)]
    assert smp_ports == {("127.0.0.1", 3433): 1437}


async def test_param_write_locks_are_pruned(extension, device, autojump_clock):
    await discover(extension, device)

    await extension.set_param(DEVICE_SYSID, "UWB_CH", 7)
    assert extension._param_write_locks == {}

    device.respond_to_set = False
    with pytest.raises(trio.TooSlowError):
        await extension.set_param(DEVICE_SYSID, "UWB_CH", 8, timeout=1)
    assert extension._param_write_locks == {}


async def test_refill_repairs_frame_params(extension, device):
    # a dump-loss hole in an OPTIONAL frame param (POS_YAW_DEG, CELL_ID)
    # would keep the Verify frame rule waiting for it forever: the refill
    # repairs these too
    add_rtls_cell_params(device)
    set_fake_param(device, "POS_YAW_DEG", 30.0, "real32")
    set_fake_param(device, "CELL_ID", "default", "custom")
    device.drop_from_list = {"POS_YAW_DEG"}
    await discover(extension, device)
    cached = extension._protocol.devices[DEVICE_SYSID]
    assert "POS_YAW_DEG" not in cached.params

    device.read_requests.clear()
    await extension._poll_param_refill(time.monotonic() + REFILL_INITIAL_DELAY + 1)

    assert device.read_requests == ["POS_YAW_DEG"]
    assert "POS_YAW_DEG" in cached.params
    assert DEVICE_SYSID not in extension._refill


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
    for anchor in VERIFY_ANCHORS:
        add_anchor_device(
            extension, anchor, role=2 if anchor == 200 else 3, uwb_mac=anchor
        )
        extension._stats[anchor] = {
            "id": anchor,
            "cellConfig": SESSION_CODE,
            "txGain": 47,
            "surveyId": SURVEY_ID,
            **({} if anchor == 200 else {"rangeDeviationM": 0.01}),
        }
        extension._stats_at[anchor] = time.monotonic()
    for sysid in (DEVICE_SYSID, DEVICE_SYSID + 1):
        extension._stats[sysid] = {
            "id": sysid,
            "cellConfig": SESSION_CODE,
            "solveRateHz": 12.5,
            "solvePct": 97.0,
            "fixAgeMs": 80,
            "clockPpm": 1.2,
            "anchorMask": 0b1111,
            "clockSyncOk": True,
            "surveyId": SURVEY_ID,
            "anchorsUsed": len(VERIFY_ANCHORS),
            "anchorsExcluded": 0,
            "surveyDriftM": 0.02,
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
    pairing = next(rule for rule in response.body["rules"] if rule["id"] == "pairing")
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
    cache_frame(extension, DEVICE_SYSID, DEVICE_SYSID + 1)

    response = await verify_message(extension, builder, hub)

    body = response.body
    assert body["type"] == "X-RTLS-VERIFY"
    assert body["passed"] is True, body["rules"]
    statuses = {rule["id"]: rule["status"] for rule in body["rules"]}
    assert statuses == {
        "survey": "pass",
        "frame": "pass",
        "anchors": "pass",
        "firmware": "pass",
        "pairing": "pass",
        "yaw-source": "pass",
        "uwb": "pass",
        "cell": "pass",
        "params": "skipped",
    }


async def test_verify_flags_wrong_yaw_source(extension, device, dialect, builder, hub):
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
    cache_frame(extension, DEVICE_SYSID, DEVICE_SYSID + 1)

    response = await verify_message(extension, builder, hub)

    body = response.body
    assert body["passed"] is False
    rule = next(r for r in body["rules"] if r["id"] == "yaw-source")
    assert rule["status"] == "fail"
    assert "virtual compass" in rule["detail"]
    assert rule["devices"]["05"]["yawSource"] == 1.0


async def test_verify_flags_missing_stats(extension, device, dialect, builder, hub):
    second = wire_verify_fleet(extension, device, dialect)
    await discover(extension, device)
    await extension._process_datagram(
        second.heartbeat(), second.address, time.monotonic()
    )
    cache_frame(extension, DEVICE_SYSID, DEVICE_SYSID + 1)
    del extension._stats[DEVICE_SYSID + 1]  # one tag went silent

    response = await verify_message(extension, builder, hub)

    body = response.body
    assert body["passed"] is False
    by_id = {rule["id"]: rule for rule in body["rules"]}
    assert by_id["survey"]["status"] == "fail"
    assert f"tag {DEVICE_SYSID + 1}: no survey telemetry" in by_id["survey"]["detail"]
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
    cache_frame(extension, DEVICE_SYSID, DEVICE_SYSID + 1)
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


def cache_frame(extension, *sysids, lat=413900000, yaw=90.0):
    """Caches the coordinate-frame params (origin, show yaw) on live
    protocol devices, as the identity refill would after discovery."""
    for sysid in sysids:
        cached = extension._protocol.devices[sysid]
        set_cached_param(cached, "ORIGIN_LAT_E7", lat, "int32")
        set_cached_param(cached, "ORIGIN_LON_E7", 21500000, "int32")
        set_cached_param(cached, "ORIGIN_ALT_MM", 10000, "int32")
        set_cached_param(cached, "POS_YAW_DEG", yaw, "real32")


def test_stats_json_carries_link_quality_and_wifi():
    body = _stats_json(
        7,
        {
            **FULL_STATS,
            "urssi": -81.53,
            "nlosmask": 4.0,
            "outmask": 32.0,
            "rmse": 0.04213,
            "rssi": -61.0,
            "wdisc": 2.0,
        },
    )
    assert body["uwbRssiDbm"] == -81.5
    assert body["nlosMask"] == 4
    assert body["outvotedMask"] == 32
    assert body["residualRmsM"] == 0.0421
    assert body["wifiRssiDbm"] == -61
    assert body["wifiDisconnects"] == 2


def test_stats_json_link_quality_sentinels_are_null():
    # urssi 0: no row carried a reading; rmse -1: no solve in the interval
    body = _stats_json(7, {**FULL_STATS, "urssi": 0.0, "rmse": -1.0})
    assert body["uwbRssiDbm"] is None
    assert body["residualRmsM"] is None


def test_stats_json_without_link_quality_omits_the_fields():
    body = _stats_json(7, dict(FULL_STATS))
    for key in (
        "uwbRssiDbm",
        "nlosMask",
        "outvotedMask",
        "residualRmsM",
        "wifiRssiDbm",
        "wifiDisconnects",
    ):
        assert key not in body


async def test_stats_carry_the_paired_drones_imu_temperatures(
    extension, device, builder, hub
):
    _claim_flight_controller(device, 5, 1)
    await discover(extension, device)
    _set_uav_ids(extension, {5: "05"})
    extension._refresh_uav_map()
    temperatures = {"05": [41.5, None]}
    extension._mavlink_api = SimpleNamespace(
        loaded=True, get_imu_temperatures=lambda uav_id: temperatures.get(uav_id, [])
    )

    await _feed_stats(extension, device, FULL_STATS, now=0.0)
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert response.body["stats"][str(DEVICE_SYSID)]["imuTemperaturesC"] == [
        41.5,
        None,
    ]

    # no reading from the drone: the field is left out, not an empty list
    temperatures.clear()
    await _feed_stats(extension, device, FULL_STATS, now=1.0)
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert "imuTemperaturesC" not in response.body["stats"][str(DEVICE_SYSID)]


async def test_stats_without_a_paired_drone_omit_imu_temperatures(
    extension, device, builder, hub
):
    await discover(extension, device)
    extension._mavlink_api = SimpleNamespace(
        loaded=True, get_imu_temperatures=lambda uav_id: [40.0]
    )

    await _feed_stats(extension, device, FULL_STATS, now=0.0)
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert "imuTemperaturesC" not in response.body["stats"][str(DEVICE_SYSID)]


@pytest.mark.parametrize(
    "code,phrase",
    [
        (2, "fewer than 64 distance samples"),
        (5, "does not mirror the bottom one"),
    ],
)
# ---- anchors rendered from the tag's own fit ----------------------------


# ---- one health report per datagram ---------------------------------------


def _bundle(device, values):
    """A whole health report in one datagram, as the firmware sends it."""
    return b"".join(device.named_value_float(n, v) for n, v in values.items())


BUNDLED_REPORT = {**FULL_STATS, "urssi": -81.5, "rmse": 0.025}


async def test_a_lane_gap_reaches_the_watchers_and_changes_no_stats(extension, device):
    await discover(extension, device)
    await extension._process_datagram(
        _bundle(device, BUNDLED_REPORT), device.address, time.monotonic()
    )
    before = dict(extension._stats[DEVICE_SYSID])

    with extension._subscribed_events() as events:
        await extension._process_datagram(
            _bundle(device, {**BUNDLED_REPORT, "gapcomms": 1800.0}),
            device.address,
            time.monotonic(),
        )
        # and from a device that is not known: a stall outlasts the timeout
        stranger = FakeDevice(device.dialect, system_id=77)
        await extension._process_datagram(
            stranger.named_value_float("gaprt", 40.0), ("192.168.4.77", 3333), 0.0
        )
        kinds = []
        while True:
            try:
                kinds.append(events.receive_nowait().kind)
            except trio.WouldBlock:
                break

    assert kinds.count("lane_gap") == 2
    assert extension._stats[DEVICE_SYSID] == before
    assert 77 not in extension._stats


async def test_a_bundled_report_is_handled_once(extension, device):
    _claim_flight_controller(device, 5, 1)
    await discover(extension, device)
    _set_uav_ids(extension, {5: "05"})
    extension._refresh_uav_map()
    joins = []
    extension._mavlink_api = SimpleNamespace(
        loaded=True, get_imu_temperatures=lambda uav_id: joins.append(uav_id) or [40.0]
    )

    await extension._process_datagram(
        _bundle(device, BUNDLED_REPORT), device.address, 0.0
    )

    assert joins == ["05"]
    stats = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert len(stats) == 1
    entry = stats[0]["stats"][str(DEVICE_SYSID)]
    # the fields after the required set ride the same, first broadcast
    assert entry["uwbRssiDbm"] == -81.5
    assert entry["residualRmsM"] == 0.025
    assert entry["imuTemperaturesC"] == [40.0]


async def test_a_report_inside_the_window_is_cached_then_flushed_once(
    extension, device, builder, hub
):
    await discover(extension, device)
    await extension._process_datagram(
        _bundle(device, BUNDLED_REPORT), device.address, 0.0
    )
    newer = {**BUNDLED_REPORT, "rate": 9.0}
    await extension._process_datagram(_bundle(device, newer), device.address, 0.5)

    def stats_broadcasts():
        return [
            b
            for b in extension.app.message_hub.broadcasts
            if b["type"] == "X-RTLS-STATS"
        ]

    assert len(stats_broadcasts()) == 1
    message = make_message(builder, {"type": "X-RTLS-STATS"})
    response = await extension._handle_RTLS_STATS(message, None, hub)
    assert response.body["stats"][str(DEVICE_SYSID)]["solveRateHz"] == 9.0

    await extension._flush_pending_stats(now=1.0)
    await extension._flush_pending_stats(now=2.5)
    assert len(stats_broadcasts()) == 2
    assert stats_broadcasts()[-1]["stats"][str(DEVICE_SYSID)]["solveRateHz"] == 9.0


async def test_watchers_get_one_stats_event_per_report(extension, device):
    await discover(extension, device)
    with extension._subscribed_events() as events:
        await extension._process_datagram(
            _bundle(device, BUNDLED_REPORT), device.address, 0.0
        )
        kinds = []
        while True:
            try:
                kinds.append(events.receive_nowait().kind)
            except trio.WouldBlock:
                break

    assert kinds.count("stats") == 1


# ---- anchor health stats ------------------------------------------------

ANCHOR_STATS = {
    "vbat": 7.9123,
    "soc": 81.25,
    "ibat": 0.4321,
    "btemp": 24.56,
    "lowbat": 0.0,
    "pwrst": 5.0,
    "pwrin": 5.0,
    "chg": 3.0,
    "chgblk": 0.0,
    "uwbst": 2.0,
    "rnghz": 49.876,
    "urst": 1.0,
    # the retired motion watch of older firmware: ignored
    "moved": 1.0,
    "tilt": 3.21,
    "knocks": 4.0,
    "elink": 1.0,
    "edrop": 2.0,
    "rssi": -61.0,
    "wdisc": 3.0,
    "wtxerr": 7.0,
    "boots": 12.0,
    "rstc": 1.0,
}


def test_stats_json_anchor_fields():
    body = _stats_json(7, ANCHOR_STATS, "anchor-responder")

    assert body == {
        "id": 7,
        "batteryVoltage": 7.912,
        "batteryPercent": 81.2,
        "batteryCurrentA": 0.432,
        "batteryTemperatureC": 24.6,
        "lowBattery": False,
        "batteryWarning": False,
        "powerState": "operating",
        "powerInput": "poe",
        "charging": "cv",
        "chargeBlock": "none",
        "uwbStatus": 2,
        "rangingRateHz": 49.88,
        "radioRestarts": 1,
        "ethernetLink": True,
        "ethernetDrops": 2,
        "wifiRssiDbm": -61,
        "wifiDisconnects": 3,
        "wifiTxErrors": 7,
        "bootCount": 12,
        "resetCause": 1,
    }


@pytest.mark.parametrize(
    ("name", "key", "names"),
    [
        (
            "pwrst",
            "powerState",
            [
                "booting",
                "preview",
                "standby",
                "asleep",
                "off",
                "operating",
                "stopping",
                "shutdown",
                "fault",
            ],
        ),
        (
            "pwrin",
            "powerInput",
            ["unknown", "battery", "usb", "usb-1.5a", "usb-3a", "poe"],
        ),
        ("chg", "charging", ["off", "idle", "cc", "cv", "topoff"]),
        (
            "chgblk",
            "chargeBlock",
            [
                "none",
                "no-profile",
                "no-input",
                "charger",
                "gauge",
                "temperature",
                "gauge-inhibit",
                "pack-voltage",
                "charger-fault",
                "overcurrent",
                "stopping",
            ],
        ),
    ],
)
def test_stats_json_maps_state_codes(name, key, names):
    for code, expected in enumerate(names):
        assert _stats_json(7, {name: float(code)}, "anchor-initiator")[key] == expected
    # a code the server does not know, or not an exact integer, is unknown
    for value in (float(len(names)), -1.0, 1.5):
        assert _stats_json(7, {name: value}, "anchor-initiator")[key] == "unknown"
    # a non-finite value is dropped like every other stat
    assert key not in _stats_json(7, {name: math.nan}, "anchor-initiator")


def test_stats_json_anchor_omits_absent_fields():
    # every field is present only while its stat is: omitted means unknown
    assert _stats_json(7, {"vbat": 8.0}, "anchor-initiator") == {
        "id": 7,
        "batteryVoltage": 8.0,
    }


def test_stats_json_unknown_role_is_tag_shaped():
    # a tag's 10 s status report carries no solver stats either: only
    # UWB_ROLE switches a device to the anchor shape
    assert _stats_json(7, {"boots": 3.0, "rstc": 1.0})["solveRateHz"] == 0.0


async def test_stats_take_the_anchor_shape_once_the_role_says_anchor(
    extension, device, builder, hub
):
    device.respond_to_list = False  # the dump has not delivered UWB_ROLE
    await discover(extension, device)
    await extension._on_stats(DEVICE_SYSID, {"boots": 3.0}, 0.0)
    assert extension._stats[DEVICE_SYSID]["solveRateHz"] == 0.0

    set_fake_param(device, "UWB_ROLE", 3, "uint8")
    await extension._process_datagram(
        device._param_value("UWB_ROLE", 0), device.address, 1.0
    )
    await extension._on_stats(DEVICE_SYSID, {"boots": 3.0}, 2.0)
    assert extension._stats[DEVICE_SYSID] == {"id": DEVICE_SYSID, "bootCount": 3}


def test_stats_json_tag_keeps_its_solver_defaults():
    body = _stats_json(7, {"vbat": 8.0, "uwbst": 2.0, "boots": 3.0}, "tag")

    assert body["solveRateHz"] == 0.0
    assert body["anchorMask"] == 0
    assert body["uwbStatus"] == 2
    assert body["bootCount"] == 3


async def test_anchor_stats_broadcast_without_the_tag_stat_set(extension, device):
    add_rtls_cell_params(device, role=2, uwb_mac=1)
    await discover(extension, device)

    # stats the pinned SDK already accepts (pwrst, rnghz... need its bump)
    await extension._process_datagram(
        _bundle(device, {"vbat": 7.9, "soc": 80.0, "uwbst": 2.0}),
        device.address,
        0.0,
    )

    pushed = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert pushed[-1]["stats"][str(DEVICE_SYSID)] == {
        "id": DEVICE_SYSID,
        "batteryVoltage": 7.9,
        "batteryPercent": 80.0,
        "uwbStatus": 2,
    }


# ---- TWR summary spread -------------------------------------------------


def _twr_generation(device, time_boot_ms, *, distance=14.0, mad=0.0234, count=40):
    """One responder TWR summary generation in one datagram, as the
    firmware sends it (docs/reference/twr-summary.md)."""
    frames = (
        ("trcap", 1.0),
        ("trseq", 7.0),
        ("trmask", 1.0),
        ("twr0001", distance),
        ("twm0001", mad),
        ("twn0001", float(count)),
    )
    return b"".join(
        device.named_value_float(name, value, time_boot_ms) for name, value in frames
    )


async def test_twr_rows_carry_the_summary_spread(extension, device):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)

    await extension._process_datagram(
        _twr_generation(device, 1000), device.address, 10.0
    )
    row = extension._inf_status(10.5)[str(DEVICE_SYSID)]["twr"][0]
    assert row == {
        "peerMac": 0x0001,
        "distanceM": 14.0,
        "ageMs": 500,
        "madM": 0.0234,
        "count": 40,
    }

    # a later distance without its complete generation (a lost frame) is
    # not paired with the previous generation's spread
    await extension._process_datagram(
        device.named_value_float("twr0001", 14.2, 2000), device.address, 11.0
    )
    row = extension._inf_status(11.0)[str(DEVICE_SYSID)]["twr"][0]
    assert row["distanceM"] == 14.2
    assert "madM" not in row and "count" not in row

    # loss drops the summary with the rows
    extension._expire_devices(time.monotonic() + 1000.0)
    assert DEVICE_SYSID not in extension._twr_summary


# ---- lost devices and X-RTLS-FORGET -------------------------------------


async def test_forget_drops_a_lost_device(extension, device, builder, hub):
    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=0.0)
    extension._expire_devices(time.monotonic() + 1000)
    pushes = len(_inf_broadcasts(extension))

    response = await extension._handle_RTLS_FORGET(
        make_message(builder, {"type": "X-RTLS-FORGET", "ids": [DEVICE_SYSID, 99]}),
        None,
        hub,
    )

    assert response.body == {
        "type": "X-RTLS-FORGET",
        "result": {
            str(DEVICE_SYSID): {"forgotten": True},
            "99": {"forgotten": False, "detail": "No such device: 99"},
        },
    }
    assert await _inf_status(extension, builder, hub) == {}
    assert DEVICE_SYSID not in extension._stats
    # the device list is pushed without the forgotten device
    await extension._flush_pending_inf(now=time.monotonic() + 10.0)
    broadcasts = _inf_broadcasts(extension)
    assert len(broadcasts) == pushes + 1
    assert broadcasts[-1]["status"] == {}


async def test_forget_refuses_a_live_device(extension, device, builder, hub):
    await discover(extension, device)

    response = await extension._handle_RTLS_FORGET(
        make_message(builder, {"type": "X-RTLS-FORGET", "id": DEVICE_SYSID}),
        None,
        hub,
    )

    assert response.body["result"] == {
        str(DEVICE_SYSID): {
            "forgotten": False,
            "detail": f"Device {DEVICE_SYSID} is online",
        }
    }
    assert str(DEVICE_SYSID) in await _inf_status(extension, builder, hub)


async def test_forget_rejects_malformed_requests(extension, builder, hub):
    for body in (
        {"type": "X-RTLS-FORGET"},
        {"type": "X-RTLS-FORGET", "ids": []},
        {"type": "X-RTLS-FORGET", "ids": [True]},
        {"type": "X-RTLS-FORGET", "id": "x"},
    ):
        response = await extension._handle_RTLS_FORGET(
            make_message(builder, body), None, hub
        )
        assert response.body.get("type") == "ACK-NAK", body


async def _noop():
    return None


async def test_sleep_on_an_offline_device_says_so(
    extension, device, builder, hub, autojump_clock
):
    await discover(extension, device)
    extension._expire_devices(time.monotonic() + 1000)

    response = await extension._handle_RTLS_SLEEP(
        make_message(
            builder, {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": True}
        ),
        None,
        hub,
    )

    entry = response.body["result"][str(DEVICE_SYSID)]
    assert entry["accepted"] is False
    assert entry["detail"] == f"Device {DEVICE_SYSID} is offline"


async def test_param_set_refuses_remote_off(extension, device, builder, hub):
    await discover(extension, device)
    for value in (2, True, "1"):
        response = await extension._handle_RTLS_PARAM_SET(
            make_message(
                builder,
                {
                    "type": "X-RTLS-PARAM-SET",
                    "id": DEVICE_SYSID,
                    "name": "SLEEP",
                    "value": value,
                },
            ),
            None,
            hub,
        )
        assert response.body.get("type") == "ACK-NAK", value
    assert device.params["SLEEP"][0] == bytes([0])


# ---- retained identity across loss --------------------------------------


async def test_a_returning_anchor_keeps_its_name(extension, device, builder, hub):
    add_anchor_identity_params(device, uwb_mac=2)
    await discover(extension, device)
    assert (await _inf_status(extension, builder, hub))[str(DEVICE_SYSID)][
        "name"
    ] == "RTLS anchor A1"

    extension._expire_devices(time.monotonic() + 1000)
    # it returns, but its parameter dump has not landed (lost or slow)
    device.respond_to_list = False
    await discover(extension, device)

    entry = (await _inf_status(extension, builder, hub))[str(DEVICE_SYSID)]
    assert entry["online"] is True
    assert entry["name"] == "RTLS anchor A1"
    assert entry["role"] == "anchor-responder"
    # live checks read only the new cache
    assert extension._protocol.devices[DEVICE_SYSID].params == {}

    # once the new snapshot is complete, the retained copy is dropped
    device.respond_to_list = True
    request = extension._protocol.request_param_list(DEVICE_SYSID)
    await extension._send(*request)
    assert DEVICE_SYSID not in extension._retained_params
    entry = (await _inf_status(extension, builder, hub))[str(DEVICE_SYSID)]
    assert entry["name"] == "RTLS anchor A1"


# ---- stats freshness ----------------------------------------------------


async def test_a_stat_the_device_stopped_sending_leaves_the_body(extension, device):
    add_anchor_identity_params(device)
    await discover(extension, device)

    # the SDK ages out a stat the device stops sending (PoE pulled: no
    # chargeBlock; gauge profile removed: no soc); the body follows it
    await extension._on_stats(
        DEVICE_SYSID, {"vbat": 7.9, "soc": 80.0, "chgblk": 0.0}, 0.0
    )
    await extension._on_stats(DEVICE_SYSID, {"vbat": 7.9}, 1.0)

    assert extension._stats[DEVICE_SYSID] == {"id": DEVICE_SYSID, "batteryVoltage": 7.9}


async def test_a_silent_stats_stream_goes_stale(extension, device):
    add_anchor_identity_params(device)
    await discover(extension, device)
    await extension._on_stats(DEVICE_SYSID, {"vbat": 7.9}, 0.0)

    await extension._flush_pending_stats(now=STATS_STALE_AFTER)
    assert "stale" not in extension._stats[DEVICE_SYSID]

    # no report for longer than three periods of the slowest one
    await extension._flush_pending_stats(now=STATS_STALE_AFTER + 1.0)
    assert extension._stats[DEVICE_SYSID]["stale"] is True
    pushed = [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-STATS"
    ]
    assert pushed[-1]["stats"][str(DEVICE_SYSID)]["stale"] is True

    # the next report is fresh again
    await extension._on_stats(DEVICE_SYSID, {"vbat": 7.8}, STATS_STALE_AFTER + 2.0)
    assert "stale" not in extension._stats[DEVICE_SYSID]


# ---- OTA species from the image's build receipt -------------------------


def _signed_image(tmp_path, app, *, stale=False):
    """An image with the receipt rtls-link-zephyr's scripts/build.sh writes
    beside it."""
    import hashlib
    import json

    image = tmp_path / "zephyr.signed.bin"
    image.write_bytes(b"\x3d\xb8\xf3\x96" + b"\x00" * 60)
    digest = hashlib.sha256(b"other build" if stale else image.read_bytes())
    (tmp_path / "zephyr.signed.bin.receipt.json").write_text(
        json.dumps(
            {
                "app": app,
                "board": "axio_anchor" if app == "anchor" else "axiolight_esp32s3",
                "git": "9b1573bd",
                "sha256": digest.hexdigest(),
                "mcuboot_digest": "00" * 32,
            }
        )
    )
    return image


async def _start_ota(extension, builder, hub, image, **body):
    message = make_message(
        builder,
        {"type": "X-RTLS-OTA", "id": DEVICE_SYSID, "image": str(image)} | body,
    )
    return await extension._handle_RTLS_OTA(message, None, hub)


@pytest.mark.parametrize("declared", [None, "anchor"])
async def test_ota_refuses_an_image_whose_receipt_names_another_species(
    extension, device, builder, hub, tmp_path, declared
):
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)  # a tag
    await discover(extension, device)
    image = _signed_image(tmp_path, "anchor")

    body = {} if declared is None else {"role": declared}
    response = await _start_ota(extension, builder, hub, image, **body)

    assert response.body["type"] == "ACK-NAK"
    assert "refusing to upload a anchor image" in response.body["reason"]
    assert DEVICE_SYSID not in extension._ota_jobs


async def test_ota_receipt_overrides_a_contradicting_declared_role(
    extension, device, builder, hub, tmp_path
):
    # the operator flipped the declared role to match the device: the
    # receipt still says anchor
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)
    await discover(extension, device)
    image = _signed_image(tmp_path, "anchor")

    response = await _start_ota(extension, builder, hub, image, role="tag")

    assert response.body["type"] == "ACK-NAK"
    assert "its build receipt" in response.body["reason"]


async def test_ota_refuses_a_stale_receipt(extension, device, builder, hub, tmp_path):
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)
    await discover(extension, device)
    image = _signed_image(tmp_path, "tag", stale=True)

    response = await _start_ota(extension, builder, hub, image)

    assert response.body["type"] == "ACK-NAK"
    assert "stale receipt" in response.body["reason"]


async def test_ota_job_names_the_species_and_its_source(
    extension, device, builder, hub, tmp_path
):
    device.params["UWB_ROLE"] = (bytes([3]), PARAM_TYPE_UINT8)
    await discover(extension, device)
    extension._ota_upgrade = (
        lambda address, image_path, *, timeout=10.0, on_progress=None: "9.9.9"
    )
    receipted = _signed_image(tmp_path, "anchor")
    bare = tmp_path / "bare.bin"
    bare.write_bytes(b"\x00" * 16)

    async with trio.open_nursery() as nursery:
        extension._nursery = nursery
        response = await _start_ota(extension, builder, hub, receipted)
        assert response.body["job"]["species"] == "anchor"
        assert response.body["job"]["speciesSource"] == "receipt"
        with trio.fail_after(5):
            while extension._ota_jobs[DEVICE_SYSID]["status"] == "running":
                await trio.sleep(0.01)
        # without a receipt, the declared role is the fallback
        response = await _start_ota(extension, builder, hub, bare, role="anchor")
        assert response.body["job"]["speciesSource"] == "declared"
        with trio.fail_after(5):
            while extension._ota_jobs[DEVICE_SYSID]["status"] == "running":
                await trio.sleep(0.01)
    extension._nursery = None


# ---- audit round: stale snapshots, lost devices, OTA export, forget ------


async def test_a_stale_snapshot_does_not_pass_verify(extension, device, builder, hub):
    from flockwave.server.ext.rtls.verify import _uwb_rule

    await discover(extension, device)
    await _feed_stats(extension, device, FULL_STATS, now=time.monotonic())
    assert "no telemetry" not in _uwb_rule(extension)["detail"]

    # the tag is lost and returns before it reports again: its kept snapshot
    # is recent by harvest time, but stale
    extension._expire_devices(time.monotonic() + 1000)
    await discover(extension, device)
    assert extension._stats[DEVICE_SYSID]["stale"] is True

    rule = _uwb_rule(extension)
    assert rule["status"] == "fail"
    assert "no telemetry" in rule["detail"]


async def test_a_lost_devices_late_datagram_is_ignored(extension, device):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)
    await extension._process_datagram(
        _bundle(device, {"vbat": 7.9, "pwrst": 5.0}), device.address, 0.0
    )
    extension._expire_devices(time.monotonic() + 1000)
    kept = dict(extension._stats[DEVICE_SYSID])

    # stats and TWR frames without a heartbeat (the SDK still decodes them)
    await extension._process_datagram(
        _bundle(device, {"vbat": 6.0, "twr0001": 14.0}), device.address, 1.0
    )

    assert extension._stats[DEVICE_SYSID] == kept
    assert DEVICE_SYSID not in extension._twr


async def test_inf_sleeping_follows_an_anchors_power_state(
    extension, device, builder, hub
):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    device.params["SLEEP"] = (bytes([1]), PARAM_TYPE_UINT8)  # heartbeats STANDBY
    await discover(extension, device)
    # standby, off and asleep all heartbeat STANDBY
    assert extension._protocol.devices[DEVICE_SYSID].sleeping is True

    async def inf_sleeping(pwrst):
        await extension._process_datagram(
            device.named_value_float("pwrst", pwrst), device.address, 0.0
        )
        return (await _inf_status(extension, builder, hub))[str(DEVICE_SYSID)][
            "sleeping"
        ]

    assert await inf_sleeping(2.0) is False  # plain standby
    assert await inf_sleeping(4.0) is False  # off
    assert await inf_sleeping(3.0) is True  # asleep


async def test_start_ota_export_refuses_unchecked_images(extension, device, tmp_path):
    device.params["UWB_ROLE"] = (bytes([1]), PARAM_TYPE_UINT8)
    await discover(extension, device)
    bare = tmp_path / "bare.bin"
    bare.write_bytes(b"\x00" * 16)

    with pytest.raises(
        ValueError, match="no build receipt and no declared role; build it with"
    ):
        await extension.start_ota(DEVICE_SYSID, str(bare))
    with pytest.raises(ValueError, match="refusing to upload a anchor image"):
        await extension.start_ota(DEVICE_SYSID, str(_signed_image(tmp_path, "anchor")))
    assert DEVICE_SYSID not in extension._ota_jobs


async def test_forget_dedupes_ids_and_checks_a_single_id(
    extension, device, builder, hub
):
    await discover(extension, device)
    extension._expire_devices(time.monotonic() + 1000)

    response = await extension._handle_RTLS_FORGET(
        make_message(
            builder, {"type": "X-RTLS-FORGET", "ids": [DEVICE_SYSID, DEVICE_SYSID]}
        ),
        None,
        hub,
    )
    assert response.body["result"] == {str(DEVICE_SYSID): {"forgotten": True}}

    for body in ({"id": True}, {"id": "42"}, {"id": 0}):
        response = await extension._handle_RTLS_FORGET(
            make_message(builder, {"type": "X-RTLS-FORGET", **body}), None, hub
        )
        assert response.body["type"] == "ACK-NAK", body


async def test_commands_ignore_the_role_retained_from_before_a_loss(
    extension, device, autojump_clock
):
    add_anchor_identity_params(device, uwb_mac=2)
    await discover(extension, device)
    extension._expire_devices(time.monotonic() + 1000)
    device.respond_to_list = False
    device.respond_to_read = False
    await discover(extension, device)

    # displayed as the anchor it was...
    assert extension._device_role(DEVICE_SYSID) == "anchor-responder"
    # ...but not commanded on that memory
    assert await extension._command_role(DEVICE_SYSID, 1.0) is None


async def test_a_stat_ages_out_on_the_next_frame_and_a_lost_anchor_keeps_its_last(
    extension, device
):
    add_rtls_cell_params(device, role=3, uwb_mac=2)
    await discover(extension, device)
    t = time.monotonic()
    report = {"vbat": 7.9, "lowbat": 1.0, "pwrst": 5.0, "chgblk": 2.0}
    await extension._process_datagram(_bundle(device, report), device.address, t)
    assert extension._stats[DEVICE_SYSID]["chargeBlock"] == "no-input"

    # PoE is pulled: the next reports carry no chgblk, which ages out once
    # a frame arrives after three report periods
    later = {"vbat": 7.8, "lowbat": 1.0, "pwrst": 5.0}
    await extension._process_datagram(_bundle(device, later), device.address, t + 4.0)
    assert "chargeBlock" not in extension._stats[DEVICE_SYSID]

    # the anchor goes silent and is lost: its stale snapshot still has the
    # last battery and power state (a silent device's stats do not age)
    await extension._process_expiry(t + 1000.0, logging.getLogger("test"))
    kept = extension._stats[DEVICE_SYSID]
    assert kept["stale"] is True
    assert kept["batteryVoltage"] == 7.8
    assert kept["batteryWarning"] is True
    assert kept["powerState"] == "operating"


@pytest.mark.parametrize(
    ("lowbat", "low", "warning"),
    [(0.0, False, False), (1.0, False, True), (2.0, True, False)],
)
def test_stats_json_battery_levels(lowbat, low, warning):
    body = _stats_json(7, {"lowbat": lowbat}, "anchor-responder")
    assert body["lowBattery"] is low
    assert body["batteryWarning"] is warning
