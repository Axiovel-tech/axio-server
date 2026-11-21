"""The anchor cell lock (X-RTLS-LOCK, rtls/anchor_lock.py): sealing with
acknowledged warnings, the guards on anchor-changing commands, the breaks
the integrity watch records and the show-start verdict."""

import time

import pytest
import trio
import trio.testing
from anchor_lock_support import (
    HEALTHY_A0,
    HEALTHY_A1,
    _anchor,
    _cell,
    _healthy,
    _inf_broadcasts,
    _lock,
)
from rtls_support import (
    DEVICE_SYSID,
    discover,
    make_message,
    set_fake_param,
)
from rtlslink.dialect import load_dialect

from flockwave.server.ext.rtls.anchor_lock import LOCKED_REASON, AnchorLock
from flockwave.server.ext.rtls.anchor_lock_gate import lock_json
from flockwave.server.ext.rtls.extension import AnchorsLocked, RtlsExtension
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
    await ext._on_stats(202, {**HEALTHY_A1, "lowbat": 2.0, "vbat": 6.81}, 0.0)

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

    def codes():  # oldest first (the lock keeps them newest first)
        return [(b["code"], b["device"]) for b in reversed(ext._anchor_lock.breaks)]

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
    assert ext._anchor_lock.breaks[-4]["detail"] == "RTLS anchor 202 UWB_CH 5 -> 9"

    # an anchor goes offline; the tag does not count
    ext._expire_devices(time.monotonic() + 1000)
    assert codes()[5:] == [("offline", 201), ("offline", 202)]

    # the breaks are pushed and block the show until a re-seal
    await ext._flush_pending_inf(now=time.monotonic() + 10.0)
    assert len(_inf_broadcasts(ext)[-1]["anchorLock"]["breaks"]) == 7
    assert "the anchor lock is broken" in ext.show_start_refusal()
    assert "; 4 more" in ext.show_start_refusal()
    # newest first, in the list and in the reason
    newest = ext._anchor_lock.breaks[0]
    assert (newest["code"], newest["device"]) == ("offline", 202)
    assert ext.show_start_refusal().startswith(
        f"the anchor lock is broken ({newest['detail']};"
    )
    body = await _lock(ext, builder, hub)
    assert body["result"] == "warnings"
    body = await _lock(ext, builder, hub, acknowledgeWarnings=True)
    assert body["result"] == "locked"
    assert body["anchorLock"]["breaks"] == []


def _system_time(dialect, anchor, uptime_ms):
    message = dialect.MAVLink_system_time_message(0, uptime_ms)
    return bytes(message.pack(anchor._mav))


async def test_reboots_are_judged_against_the_host_time(dialect, builder, hub):
    ext, devices = await _cell(dialect, anchors=((201, 2, 1),))
    anchor = devices[201]
    t = time.monotonic()
    # the uptime cached when sealing is the baseline
    await ext._process_datagram(
        _system_time(dialect, anchor, 50_000), anchor.address, t
    )
    await ext._on_stats(201, {"pwrst": 5.0, "boots": 0.0}, t)
    await _lock(ext, builder, hub)

    def feed(uptime, at):
        return ext._process_datagram(
            _system_time(dialect, anchor, uptime), anchor.address, t + at
        )

    await feed(60_000, 10.0)  # advanced with the host time
    assert ext._anchor_lock.breaks == []
    # a cold boot: the boot counter stays 0, the uptime starts over; a second
    # reboot right after reads a higher uptime but lags the host time
    await feed(2_000, 20.0)
    await ext._on_stats(201, {"boots": 0.0}, t + 20.0)
    await feed(9_000, 40.0)
    assert [b["detail"] for b in ext._anchor_lock.breaks] == [
        "RTLS anchor 201 rebooted (uptime reset)",
        "RTLS anchor 201 rebooted (uptime reset)",
    ]


async def test_show_refusal_needs_a_lock(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await _healthy(ext)
    assert "the anchors are not locked" in ext.show_start_refusal()

    ext._lock_required_for_show = False
    assert ext.show_start_refusal() is None


# ---- audit round: guards, restarts, saving, membership, the show gate ----


async def test_a_write_queued_before_sealing_is_refused_at_dispatch(
    dialect, builder, hub
):
    ext, devices = await _cell(dialect)
    await _healthy(ext)
    # another write of the same parameter holds the per-parameter lock
    write_lock = ext._param_write_locks.setdefault((202, "UWB_MAC"), trio.Lock())
    outcome = []

    async def queued_write():
        try:
            await ext.set_param(202, "UWB_MAC", 7, "uint16")
            outcome.append("written")
        except AnchorsLocked as ex:
            outcome.append(str(ex))

    async with trio.open_nursery() as nursery:
        await write_lock.acquire()
        nursery.start_soon(queued_write)
        await trio.testing.wait_all_tasks_blocked()
        await _lock(ext, builder, hub)  # sealed while the write waits
        write_lock.release()

    assert outcome == [LOCKED_REASON]


async def test_a_write_acked_after_sealing_breaks_the_lock(dialect, builder, hub):
    ext, devices = await _cell(dialect)
    await _healthy(ext)
    anchor = devices[202]
    anchor.respond_to_set = False  # the ACK comes later, by hand
    ack = anchor._param_ack("UWB_MAC", (7).to_bytes(2, "little"), 2, 0)
    guard = ext._require_unlocked

    async def seal_after_the_guard(system_id):
        # the write passes the guard, then the operator seals the cell
        await guard(system_id)
        await _lock(ext, builder, hub)

    ext._require_unlocked = seal_after_the_guard
    async with trio.open_nursery() as nursery:

        async def ack_after_the_seal():
            await trio.testing.wait_all_tasks_blocked()
            await ext._process_datagram(ack, anchor.address, time.monotonic())

        nursery.start_soon(ack_after_the_seal)
        result = await ext.set_param(202, "UWB_MAC", 7, "uint16")

    assert result["accepted"] is True
    assert [(b["code"], b["device"]) for b in ext._anchor_lock.breaks] == [
        ("param-changed", 202)
    ]


async def test_a_target_of_unknown_role_is_classified_first(
    dialect, builder, hub, autojump_clock
):
    ext, devices = await _cell(dialect)
    await _healthy(ext)
    await _lock(ext, builder, hub)

    # a device whose parameters have not arrived
    newcomer = _anchor(dialect, 203)
    newcomer.respond_to_list = False
    newcomer.respond_to_read = False
    ext.routes[newcomer.address] = newcomer
    await discover(ext, newcomer)
    with pytest.raises(AnchorsLocked, match="role unknown while anchors are locked"):
        await ext.set_param(203, "UWB_MAC", 7, "uint16")

    # it answers the role read: an anchor is refused, a tag proceeds
    newcomer.respond_to_read = True
    with pytest.raises(AnchorsLocked, match="anchors are locked"):
        await ext.set_param(203, "UWB_MAC", 7, "uint16")
    tag = _anchor(dialect, 204, role=1)
    tag.respond_to_list = False
    ext.routes[tag.address] = tag
    await discover(ext, tag)
    result = await ext.set_param(204, "UWB_MAC", 7, "uint16")
    assert result["accepted"] is True


async def test_a_sealed_member_missing_after_a_restart_breaks_the_lock(
    dialect, builder, hub, tmp_path
):
    ext, _ = await _cell(dialect)
    await _healthy(ext)
    ext._anchor_lock.path = tmp_path / "anchor_lock.json"
    await _lock(ext, builder, hub)

    # the server restarts; only A0 (201) and the tag come back
    restarted, back = await _cell(dialect, anchors=((201, 2, 1),))
    lock = AnchorLock(tmp_path / "anchor_lock.json")
    start = time.monotonic()
    lock.load(6.0, start)
    restarted._anchor_lock = lock
    restarted._expire_devices(start + 5.0)
    assert lock.breaks == []
    for device in back.values():  # still heartbeating
        await restarted._process_datagram(
            device.heartbeat(), device.address, start + 6.0
        )
    restarted._expire_devices(start + 6.5)
    assert [(b["code"], b["device"]) for b in lock.breaks] == [("offline", 202)]
    assert "did not return after the server restarted" in lock.breaks[0]["detail"]

    # a re-seal names the missing member and needs an acknowledgement
    body = await _lock(restarted, builder, hub)
    assert body["result"] == "warnings"
    assert {"code": "missing", "device": 202} in [
        {"code": w["code"], "device": w["device"]} for w in body["warnings"]
    ]


async def test_saving_the_lock_fails_safe(dialect, builder, hub, tmp_path):
    ext, _ = await _cell(dialect)
    await _healthy(ext)
    blocker = tmp_path / "not-a-dir"
    blocker.write_text("")
    ext._anchor_lock.path = blocker / "anchor_lock.json"

    # an operator transition that cannot be saved does not happen
    message = make_message(builder, {"type": "X-RTLS-LOCK", "action": "lock"})
    response = await ext._handle_RTLS_LOCK(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert "could not be saved" in response.body["reason"]
    assert ext._anchor_lock.locked is False

    ext._anchor_lock.path = tmp_path / "anchor_lock.json"
    await _lock(ext, builder, hub)
    ext._anchor_lock.path = blocker / "anchor_lock.json"
    message = make_message(builder, {"type": "X-RTLS-LOCK", "action": "unlock"})
    response = await ext._handle_RTLS_LOCK(message, None, hub)
    assert response.body["type"] == "ACK-NAK"
    assert ext._anchor_lock.locked is True

    # a telemetry break is kept, pushed and reported, never raised
    ext._expire_devices(time.monotonic() + 1000)
    assert [b["code"] for b in ext._anchor_lock.breaks] == ["offline", "offline"]
    assert ext._anchor_lock.json()["persistError"]


async def test_an_anchor_outside_the_seal_is_a_membership_break(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await _healthy(ext)
    await _lock(ext, builder, hub)

    newcomer = _anchor(dialect, 203)
    ext.routes[newcomer.address] = newcomer
    await discover(ext, newcomer)  # its dump names it an anchor
    await discover(ext, newcomer)

    assert [(b["code"], b["device"]) for b in ext._anchor_lock.breaks] == [
        ("membership", 203)
    ]
    assert "not part of the sealed cell" in ext._anchor_lock.breaks[0]["detail"]


async def test_the_show_gate_verdict_rides_anchor_lock(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    await _healthy(ext)

    def gate():
        return lock_json(ext)["showGate"]

    assert gate() == {
        "required": True,
        "refused": True,
        "reason": ext.show_start_refusal(),
    }
    await _lock(ext, builder, hub)
    assert gate() == {"required": True, "refused": False, "reason": None}
    ext._lock_required_for_show = False
    assert gate() == {"required": False, "refused": False, "reason": None}

    empty, _ = await _cell(dialect, anchors=())
    assert lock_json(empty)["showGate"]["required"] is False


def test_each_listen_port_has_its_own_lock_file(tmp_path):
    from types import SimpleNamespace

    ext = RtlsExtension()
    ext.app = SimpleNamespace(dirs=SimpleNamespace(user_data_path=tmp_path))
    bench = ext._anchor_lock_path({})
    sim = ext._anchor_lock_path({"listen_port": 3443})
    assert bench.name == "anchor_lock-3343.json"
    assert sim.name == "anchor_lock-3443.json"
    assert bench.parent == sim.parent
    configured = tmp_path / "elsewhere" / "lock.json"
    assert ext._anchor_lock_path({"anchor_lock_file": str(configured)}) == configured


async def test_a_sleeping_responder_is_not_operating_and_a_warning_pack_is_low(
    dialect, builder, hub
):
    ext, _ = await _cell(dialect)
    await ext._on_stats(201, HEALTHY_A0, time.monotonic())
    await ext._on_stats(
        202, {**HEALTHY_A1, "pwrst": 3.0, "rnghz": 0.0, "lowbat": 1.0}, time.monotonic()
    )

    body = await _lock(ext, builder, hub, action="status")

    assert [(w["code"], w["detail"]) for w in body["warnings"]] == [
        ("not-operating", "RTLS anchor 202 is asleep"),
        ("low-battery", "RTLS anchor 202: battery below its warning level"),
    ]
