"""The anchor cell lock (X-RTLS-LOCK, rtls/anchor_lock.py): sealing with
acknowledged warnings, the guards on anchor-changing commands, the breaks
the integrity watch records and the show-start verdict."""

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
from rtlslink.protocol import RtlsProtocol

from flockwave.server.ext.rtls.anchor_lock import (
    LOCKED_REASON,
    AnchorLock,
    cell_warnings,
)
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


def _inf_broadcasts(extension):
    return [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-INF"
    ]


async def _cell(dialect, *, anchors=((201, 2, 1), (202, 3, 2))):
    """An extension that discovered a tag (42) and the anchors given as
    (system id, UWB_ROLE, UWB_MAC), routed by address; returns it and the
    devices by system id."""
    tag = FakeDevice(dialect)
    set_fake_param(tag, "UWB_ROLE", 1, "uint8")
    devices = {tag.system_id: tag}
    for system_id, role, mac in anchors:
        anchor = FakeDevice(dialect, system_id=system_id)
        anchor.address = (f"192.168.4.{system_id}", 3333)
        set_fake_param(anchor, "UWB_ROLE", role, "uint8")
        set_fake_param(anchor, "UWB_MAC", mac, "uint16")
        devices[system_id] = anchor
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[])
    ext.app = StubApp()
    by_address = {d.address: d for d in devices.values()}

    async def fake_send(payload, address):
        device = by_address.get(address)
        for reply in device.handle(payload) if device else ():
            await ext._process_datagram(reply, device.address, time.monotonic())

    ext._send = fake_send
    for device in devices.values():
        await discover(ext, device)
    return ext, devices


HEALTHY_A0 = {"pwrst": 5.0, "boots": 3.0, "moved": 0.0, "uwbst": 1.0}
HEALTHY_A1 = {**HEALTHY_A0, "rnghz": 50.0}


async def _healthy(ext):
    await ext._on_stats(201, HEALTHY_A0, time.monotonic())
    await ext._on_stats(202, HEALTHY_A1, time.monotonic())


async def _lock(ext, builder, hub, action="lock", **body):
    message = make_message(builder, {"type": "X-RTLS-LOCK", "action": action} | body)
    response = await ext._handle_RTLS_LOCK(message, None, hub)
    return response.body


async def test_lock_a_healthy_cell(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await _healthy(ext)

    body = await _lock(ext, builder, hub)

    assert body["result"] == "locked"
    assert body["warnings"] == []
    lock = body["anchorLock"]
    assert lock["locked"] is True and lock["lockedAt"] > 0
    assert lock["anchors"] == [201, 202]  # the tag is not part of the cell
    assert lock["breaks"] == []
    inf = await ext._handle_RTLS_INF(
        make_message(builder, {"type": "X-RTLS-INF"}), None, hub
    )
    assert inf.body["anchorLock"] == lock
    assert ext.show_start_refusal() is None


async def test_lock_with_warnings_needs_acknowledgement(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await ext._on_stats(201, HEALTHY_A0, 0.0)
    await ext._on_stats(202, {**HEALTHY_A1, "lowbat": 1.0, "vbat": 6.81}, 0.0)

    body = await _lock(ext, builder, hub)
    assert body["result"] == "warnings"
    assert body["anchorLock"]["locked"] is False
    assert body["warnings"] == [
        {
            "code": "low-battery",
            "device": 202,
            "detail": "RTLS anchor 202: low battery (6.81 V)",
        }
    ]

    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"
    assert body["anchorLock"]["acknowledgedWarnings"] == body["warnings"]


async def test_status_and_unlock(dialect, builder, hub):
    ext, _ = await _cell(dialect, anchors=())
    body = await _lock(ext, builder, hub, action="status")
    assert body["result"] == "status"
    assert body["warnings"] == [
        {"code": "no-anchors", "device": None, "detail": "no anchors are known"}
    ]
    # no cell, no requirement for the show
    assert ext.show_start_refusal() is None

    await _lock(ext, builder, hub, acknowledgeWarnings=True)
    body = await _lock(ext, builder, hub, action="unlock")
    assert body["result"] == "unlocked"
    assert body["anchorLock"]["locked"] is False

    message = make_message(builder, {"type": "X-RTLS-LOCK", "action": "open"})
    response = await ext._handle_RTLS_LOCK(message, None, hub)
    assert response.body["type"] == "ACK-NAK"


def test_cell_warnings():
    def view(system_id, role, online=True, **stats):
        return {
            "systemId": system_id,
            "name": f"A{system_id}",
            "role": role,
            "online": online,
            "stats": stats,
        }

    warnings = cell_warnings(
        [
            view(1, "anchor-initiator", powerState="standby"),
            view(2, "anchor-responder", rangingRateHz=0.0, moved=True),
            view(3, "anchor-responder", uwbStatus=3, radioRestarts=2),
            view(4, "anchor-responder", online=False),
        ]
    )
    assert [(w["code"], w["device"]) for w in warnings] == [
        ("a0-not-operating", 1),
        ("a0-not-heard", 2),
        ("moved", 2),
        ("uwb-start-refused", 3),
        ("radio-restarts", 3),
        ("offline", 4),
    ]
    # an initiator that is offline, or missing, is a cell warning
    assert cell_warnings([view(2, "anchor-responder")])[0]["code"] == (
        "a0-not-operating"
    )


async def test_locked_anchors_refuse_changes_tags_do_not(
    dialect, builder, hub, autojump_clock, tmp_path
):
    ext, devices = await _cell(dialect)
    await _healthy(ext)
    await _lock(ext, builder, hub)

    def param_set(system_id, name="UWB_MAC", value=7):
        return make_message(
            builder,
            {"type": "X-RTLS-PARAM-SET", "id": system_id, "name": name, "value": value},
        )

    before = devices[202].params["UWB_MAC"]
    response = await ext._handle_RTLS_PARAM_SET(param_set(202), None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert response.body["reason"] == LOCKED_REASON
    assert devices[202].params["UWB_MAC"] == before
    response = await ext._handle_RTLS_PARAM_SET(
        param_set(DEVICE_SYSID, "UWB_CH", 9), None, hub
    )
    assert response.body["accepted"] is True

    response = await ext._handle_RTLS_SLEEP(
        make_message(
            builder,
            {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID, 202], "sleeping": True},
        ),
        None,
        hub,
    )
    assert response.body["result"]["202"]["detail"] == LOCKED_REASON
    assert response.body["result"]["202"]["accepted"] is False
    assert response.body["result"][str(DEVICE_SYSID)]["accepted"] is True

    image = tmp_path / "fw.bin"
    image.write_bytes(b"\x00" * 16)
    response = await ext._handle_RTLS_OTA(
        make_message(builder, {"type": "X-RTLS-OTA", "id": 201, "image": str(image)}),
        None,
        hub,
    )
    assert response.body["reason"] == LOCKED_REASON
    for body in (
        {"type": "X-RTLS-OTA", "id": 201},
        {"type": "X-RTLS-OTA", "image": str(image), "inspect": True},
    ):
        response = await ext._handle_RTLS_OTA(make_message(builder, body), None, hub)
        assert response.body["type"] == "X-RTLS-OTA"

    ext._expire_devices(time.monotonic() + 1000)
    response = await ext._handle_RTLS_FORGET(
        make_message(builder, {"type": "X-RTLS-FORGET", "ids": [202, DEVICE_SYSID]}),
        None,
        hub,
    )
    assert response.body["result"]["202"] == {
        "forgotten": False,
        "detail": LOCKED_REASON,
    }
    assert response.body["result"][str(DEVICE_SYSID)] == {"forgotten": True}

    # unlocked, the anchors take commands again
    await _lock(ext, builder, hub, action="unlock")
    assert ext.forget_device(202) == {"forgotten": True}


async def test_lock_breaks_once_per_occurrence(dialect, builder, hub):
    ext, devices = await _cell(dialect)
    await _healthy(ext)
    await _lock(ext, builder, hub)

    def codes():
        return [(b["code"], b["device"]) for b in ext._anchor_lock.breaks]

    # stats: a reboot (boot count), moved, leaving operating; each once
    for _ in range(2):
        await ext._on_stats(201, {**HEALTHY_A0, "boots": 4.0}, 1.0)
    for _ in range(2):
        await ext._on_stats(202, {**HEALTHY_A1, "moved": 1.0, "pwrst": 2.0}, 1.0)
    assert codes() == [("rebooted", 201), ("moved", 202), ("left-operating", 202)]

    # a snapshot param and the firmware version change
    anchor = devices[202]
    set_fake_param(anchor, "UWB_CH", 9, "int32")
    set_fake_param(anchor, "FW_VERSION", "1.2.4", "custom")
    for name in ("UWB_CH", "FW_VERSION", "UWB_CH"):
        index = list(anchor.params).index(name)
        await ext._process_datagram(
            anchor._param_value(name, index), anchor.address, time.monotonic()
        )
    assert codes()[3:] == [("param-changed", 202), ("firmware-changed", 202)]
    assert ext._anchor_lock.breaks[3]["detail"] == "RTLS anchor 202 UWB_CH 5 -> 9"

    # an anchor goes offline; the tag does not count
    ext._expire_devices(time.monotonic() + 1000)
    assert codes()[5:] == [("offline", 201), ("offline", 202)]

    # the breaks are pushed and block the show until a re-seal
    await ext._flush_pending_inf(now=time.monotonic() + 10.0)
    assert len(_inf_broadcasts(ext)[-1]["anchorLock"]["breaks"]) == 7
    assert "the anchor lock is broken" in ext.show_start_refusal()
    assert "; 4 more" in ext.show_start_refusal()
    body = await _lock(ext, builder, hub)
    assert body["result"] == "warnings"
    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"
    assert body["anchorLock"]["breaks"] == []


async def test_uptime_rewind_is_a_reboot_without_a_boot_count(dialect, builder, hub):
    ext, devices = await _cell(dialect, anchors=((201, 2, 1),))
    await ext._on_stats(201, {"pwrst": 5.0}, 0.0)
    await _lock(ext, builder, hub)
    anchor = devices[201]

    for uptime in (50_000, 60_000, 2_000):
        message = dialect.MAVLink_system_time_message(0, uptime)
        await ext._process_datagram(
            bytes(message.pack(anchor._mav)), anchor.address, time.monotonic()
        )

    assert [b["code"] for b in ext._anchor_lock.breaks] == ["rebooted"]


async def test_show_refusal_needs_a_lock(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await _healthy(ext)
    assert "the anchors are not locked" in ext.show_start_refusal()

    ext._lock_required_for_show = False
    assert ext.show_start_refusal() is None


def test_lock_persists_across_a_restart(tmp_path):
    path = tmp_path / "anchor_lock.json"
    lock = AnchorLock(path)
    lock.seal(
        {
            201: {
                "name": "RTLS anchor A0",
                "role": "anchor-initiator",
                "firmware": "1.2.3",
                "params": {"UWB_CHANNEL": 9},
                "bootCount": 3,
                "powerState": "operating",
                "moved": False,
                "online": True,
            }
        },
        [],
        1000.0,
    )
    lock.on_stats(201, {"moved": True}, 1001.0)

    restored = AnchorLock(path)
    restored.load()
    assert restored.json() == lock.json()
    assert restored.covers(201)
    # what changed while the server was down still breaks the lock
    restored.on_param(201, "UWB_CHANNEL", 5, 1002.0)
    restored.on_stats(201, {"bootCount": 4}, 1002.0)
    assert [b["code"] for b in restored.breaks] == [
        "moved",
        "param-changed",
        "rebooted",
    ]
