"""Tests for the client message API of the rtls extension.

The protocol core itself (and its value codec) is covered by the test
suite of the standalone ``rtls-link`` SDK; here the message handlers
are exercised against that sans-IO core with a fake transport: the
extension's ``_send`` is replaced by a function that hands every
outbound datagram to a scripted fake device, whose MAVLink replies are
pushed back through ``_process_datagram``. No sockets are involved.
"""

import struct
import threading
import time

import pytest
import trio
import trio.testing
from rtlslink.dialect import load_dialect
from rtlslink.protocol import (
    PARAM_ACK_FAILED,
    PARAM_TYPE_CUSTOM,
    PARAM_TYPE_INT32,
    PARAM_TYPE_REAL32,
    PARAM_TYPE_UINT8,
    RTLS_COMPONENT_ID,
    RtlsProtocol,
    raw_field_bytes,
)

from flockwave.server.ext.rtls.extension import RtlsExtension
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
        }

        self.respond_to_list = True
        self.respond_to_read = True
        self.respond_to_set = True
        self.set_result = 0  # PARAM_ACK_ACCEPTED

    def heartbeat(self) -> bytes:
        d = self.dialect
        message = d.MAVLink_heartbeat_message(
            type=d.MAV_TYPE_GENERIC,
            autopilot=d.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=d.MAV_STATE_ACTIVE,
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
                    out.append(self._param_value(name, index))
            elif kind == "PARAM_EXT_REQUEST_READ" and self.respond_to_read:
                name = message.param_id
                if name in self.params:
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

    def create_notification(self, body=None):
        return body

    async def broadcast_message(self, message):
        self.broadcasts.append(message)


class StubApp:
    def __init__(self):
        self.message_hub = StubMessageHub()


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


# ---- X-RTLS-INF ---------------------------------------------------------


async def test_inf_empty_before_discovery(extension, builder, hub):
    message = make_message(builder, {"type": "X-RTLS-INF"})
    response = await extension._handle_RTLS_INF(message, None, hub)
    assert response.body == {"type": "X-RTLS-INF", "status": {}}


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
}


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
