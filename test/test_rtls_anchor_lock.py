"""The anchor cell lock (X-RTLS-LOCK, rtls/anchor_lock.py): the firmware
owns it; the server writes LOCK to the online anchors, warns or refuses
before locking, derives the cell's state from the anchors' ``lock`` stats and
names a locked anchor's refusals."""

import time

import pytest
import trio
import trio.testing
from anchor_lock_support import (
    HEALTHY_A0,
    HEALTHY_A1,
    _cell,
    _healthy,
    _inf_broadcasts,
    _lock,
)
from rtls_support import make_message
from rtlslink.dialect import load_dialect
from rtlslink.protocol import PARAM_TYPE_UINT8

from flockwave.server.ext.rtls.anchor_lock import lock_json
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


def _lock_param(device):
    return device.params["LOCK"][0][0]


async def _report_lock(ext, system_id, locked, stats):
    await ext._on_stats(system_id, {**stats, "lock": float(locked)}, time.monotonic())


async def test_lock_writes_lock_to_every_online_anchor(dialect, builder, hub):
    ext, devices = await _cell(dialect)
    await _healthy(ext)

    body = await _lock(ext, builder, hub)

    assert body["result"] == "locked"
    assert body["reason"] is None
    assert body["warnings"] == []
    assert body["results"] == {"201": {"accepted": True}, "202": {"accepted": True}}
    assert _lock_param(devices[201]) == 1 and _lock_param(devices[202]) == 1
    assert "LOCK" not in devices[42].params  # the tag is not written

    body = await _lock(ext, builder, hub, action="unlock")
    assert body["result"] == "unlocked"
    assert _lock_param(devices[201]) == 0 and _lock_param(devices[202]) == 0


async def test_lock_is_refused_while_an_anchor_is_not_operating(dialect, builder, hub):
    ext, devices = await _cell(dialect)
    await ext._on_stats(201, HEALTHY_A0, time.monotonic())
    await ext._on_stats(202, {**HEALTHY_A1, "pwrst": 3.0}, time.monotonic())

    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)

    assert body["result"] == "refused"
    assert body["reason"] == "RTLS anchor 202 is asleep; wake it first"
    assert body["results"] == {}
    assert _lock_param(devices[201]) == 0  # nothing was written
    status = await _lock(ext, builder, hub, action="status")
    assert status["reason"] == body["reason"]


async def test_lock_with_warnings_needs_acknowledgement(dialect, builder, hub):
    ext, devices = await _cell(dialect)
    await ext._on_stats(201, HEALTHY_A0, time.monotonic())
    await ext._on_stats(
        202, {**HEALTHY_A1, "lowbat": 1.0, "moved": 1.0, "urst": 2.0}, time.monotonic()
    )

    body = await _lock(ext, builder, hub)
    assert body["result"] == "warnings"
    assert [(w["code"], w["device"]) for w in body["warnings"]] == [
        ("battery-warning", 202),
        ("moved", 202),
        ("radio-restarts", 202),
    ]
    assert _lock_param(devices[202]) == 0

    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"


async def test_lock_warnings(dialect, builder, hub):
    ext, _ = await _cell(dialect, anchors=())
    body = await _lock(ext, builder, hub, action="status")
    assert [w["code"] for w in body["warnings"]] == ["no-anchors"]

    ext, _ = await _cell(dialect)
    await ext._on_stats(201, HEALTHY_A0, time.monotonic())
    await ext._on_stats(202, {**HEALTHY_A1, "rnghz": 0.0, "lowbat": 2.0}, 0.0)
    body = await _lock(ext, builder, hub, action="status")
    assert [(w["code"], w["detail"]) for w in body["warnings"]] == [
        ("a0-not-heard", "RTLS anchor 202 does not hear A0"),
        ("low-battery", "RTLS anchor 202: low battery"),
    ]
    ext._expire_devices(time.monotonic() + 1000)
    body = await _lock(ext, builder, hub, action="status")
    assert {w["code"] for w in body["warnings"]} == {"offline"}


async def test_a_refused_write_is_reported(dialect, builder, hub):
    ext, devices = await _cell(dialect)
    await _healthy(ext)
    devices[202].set_result = 2  # PARAM_ACK_FAILED

    body = await _lock(ext, builder, hub)

    assert body["result"] == "incomplete"
    assert body["results"]["201"] == {"accepted": True}
    assert body["results"]["202"] == {
        "accepted": False,
        "detail": "RTLS anchor 202 refused LOCK=1",
    }


async def test_anchor_lock_follows_the_lock_stats(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await _report_lock(ext, 201, 1, HEALTHY_A0)
    await _report_lock(ext, 202, 0, HEALTHY_A1)
    assert lock_json(ext) == {
        "locked": False,
        "lockedCount": 1,
        "anchorCount": 2,
        "unlocked": [202],
        "showGate": {
            "required": True,
            "refused": True,
            "reason": "the anchors are not locked (1/2 online anchors locked); "
            "lock them in Control first",
        },
    }
    assert ext.show_start_refusal() == lock_json(ext)["showGate"]["reason"]

    # the anchor's lock flip pushes the device list
    pushes = len(_inf_broadcasts(ext))
    await _report_lock(ext, 202, 1, HEALTHY_A1)
    await ext._flush_pending_inf(now=time.monotonic() + 10.0)
    assert len(_inf_broadcasts(ext)) == pushes + 1
    assert _inf_broadcasts(ext)[-1]["anchorLock"]["locked"] is True
    assert ext.show_start_refusal() is None

    # the opt-out lifts the gate
    ext._lock_required_for_show = False
    assert lock_json(ext)["showGate"] == {
        "required": False,
        "refused": False,
        "reason": None,
    }


async def test_no_anchors_no_lock_and_no_requirement(dialect):
    ext, _ = await _cell(dialect, anchors=())
    assert lock_json(ext)["locked"] is False
    assert lock_json(ext)["showGate"]["required"] is False


async def test_a_locked_anchors_refusals_say_so(
    dialect, builder, hub, autojump_clock, tmp_path
):
    ext, devices = await _cell(dialect)
    await _report_lock(ext, 202, 1, HEALTHY_A1)
    devices[202].set_result = 2  # the firmware refuses every remote write

    response = await ext._handle_RTLS_PARAM_SET(
        make_message(
            builder,
            {"type": "X-RTLS-PARAM-SET", "id": 202, "name": "UWB_MAC", "value": 7},
        ),
        None,
        hub,
    )
    assert response.body["accepted"] is False
    assert response.body["detail"] == "RTLS anchor 202 is locked"

    devices[202].params["SLEEP"] = (bytes([0]), PARAM_TYPE_UINT8)
    response = await ext._handle_RTLS_SLEEP(
        make_message(builder, {"type": "X-RTLS-SLEEP", "ids": [202], "sleeping": True}),
        None,
        hub,
    )
    assert response.body["result"]["202"]["detail"] == "RTLS anchor 202 is locked"


def test_stats_carry_the_lock():
    from flockwave.server.ext.rtls.extension import _stats_json

    assert _stats_json(7, {"lock": 1.0}, "anchor-initiator") == {
        "id": 7,
        "locked": True,
    }


def test_power_states_read_naturally():
    from flockwave.server.ext.rtls.anchor_lock import state_phrase

    assert [state_phrase(s) for s in ("standby", "asleep", "off", "fault")] == [
        "is in standby",
        "is asleep",
        "is off",
        "has a power fault",
    ]


async def test_the_lock_stat_arrives_through_the_sdk(dialect):
    from anchor_lock_support import _anchor
    from rtls_support import discover

    ext, _ = await _cell(dialect, anchors=())
    anchor = _anchor(dialect, 203, role=2, mac=1)
    ext.routes[anchor.address] = anchor
    await discover(ext, anchor)

    await ext._process_datagram(
        anchor.named_value_float("lock", 1.0), anchor.address, time.monotonic()
    )

    assert ext._stats[203]["locked"] is True
    assert lock_json(ext)["locked"] is True
