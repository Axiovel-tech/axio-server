"""Cell settings (X-RTLS-CELL, rtls/cell_settings.py): the state derived from
what each device reports running, the lock warning and Verify rule, and an
apply that writes the tags first and then the anchors, each applying the
write at once."""

import time

import pytest
import trio.testing
from anchor_lock_support import _lock
from cell_settings_support import (
    CODE_83,
    TAG,
    _cell_settings,
    _live_cell,
    _param,
    _record_writes,
    _report,
    _request,
    _states,
)
from rtls_support import set_fake_param
from rtlslink.dialect import load_dialect
from rtlslink.protocol import raw_field_bytes

from flockwave.server.ext.rtls.verify import run_verify
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

ANCHORS = (201, 202)


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


def _authorize(ext):
    class Show:
        authorized_to_start = True
        authorization_scope = AuthorizationScope.REHEARSAL

    class Api:
        def get_configuration(self):
            return Show()

    ext.app.import_api = lambda name: Api()


async def _rule(ext):
    verify = await run_verify(ext)
    return next(rule for rule in verify["rules"] if rule["id"] == "cell")


async def test_a_converged_cell_reads_ok(dialect):
    ext, _ = await _live_cell(dialect)
    assert _cell_settings(ext) == {
        "rateHz": 50,
        "blockMs": 20,
        "txGain": 47,
        "reference": 201,
        "differ": [],
        "unknown": [],
    }
    assert _states(ext) == {TAG: ("ok", None), 201: ("ok", None), 202: ("ok", None)}


async def test_states_compare_what_runs_with_what_a0_runs(dialect):
    ext, devices = await _live_cell(dialect)
    await _report(ext, devices, [202], cellcfg=float(CODE_83))
    assert _states(ext)[202] == ("differ", None)

    # no session running (standby, asleep, a failed start) reads -1
    await ext._on_stats(TAG, {"cellcfg": -1.0}, time.monotonic())
    assert ext._stats[TAG]["cellConfig"] is None
    assert _states(ext)[TAG] == ("unknown", None)

    await _report(ext, devices, [202], txgain=20.0)
    assert _states(ext)[202] == ("differ", "different TX level")


async def test_without_a0_running_nothing_compares_or_applies(dialect, builder, hub):
    ext, devices = await _live_cell(dialect)
    await _report(ext, devices, [201], cellcfg=-1.0, txgain=-1.0)
    assert _cell_settings(ext)["reference"] is None
    assert set(_states(ext).values()) == {("unknown", None)}

    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "A0 is not ranging")
    assert _param(devices[202], "UWB_BLOCK_MS") == 20

    lock = await _lock(ext, builder, hub)
    assert [(w["code"], w["device"], w["detail"]) for w in lock["warnings"]] == [
        ("cell-settings", None, "Cell settings: no A0 to compare")
    ]
    rule = await _rule(ext)
    assert (rule["status"], rule["detail"]) == (
        "fail",
        "Cell settings: no A0 to compare",
    )

    ext._protocol.devices.pop(201)
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "A0 is offline")


async def test_lock_warns_and_verify_fails_on_a_device_not_ok(dialect, builder, hub):
    ext, devices = await _live_cell(dialect)
    await _report(ext, devices, [202], cellcfg=float(CODE_83))
    ext._stats.pop(TAG)

    body = await _lock(ext, builder, hub)

    assert body["result"] == "warnings"
    assert [(w["code"], w["detail"]) for w in body["warnings"]] == [
        ("cell-settings", f"RTLS tag {TAG}: cell settings not reported"),
        ("cell-settings", "RTLS anchor 202: different cell settings"),
    ]
    rule = await _rule(ext)
    assert (rule["status"], rule["severity"]) == ("fail", "error")
    assert rule["detail"].endswith("RTLS anchor 202: different cell settings")


async def test_apply_writes_the_tags_then_the_anchors_and_runs_at_once(
    dialect, builder, hub
):
    ext, devices = await _live_cell(dialect)
    order, write = [], ext.set_param

    async def set_param(system_id, name, *args, **kwargs):
        order.append(system_id)
        return await write(system_id, name, *args, **kwargs)

    ext.set_param = set_param

    body = await _request(ext, builder, hub, rateHz=83)

    assert (body["result"], body["reason"]) == ("applied", None)
    assert body["results"] == {
        str(sid): {"written": "changed"} for sid in (TAG, *ANCHORS)
    }
    assert order[0] == TAG and set(order[1:]) == set(ANCHORS)
    # a preset writes the block only
    assert all(_param(device, "UWB_BLOCK_MS") == 12 for device in devices.values())
    assert body["cellSettings"]["rateHz"] == 83

    await _report(ext, devices)
    assert _cell_settings(ext)["differ"] == []
    assert set(_states(ext).values()) == {("ok", None)}


async def test_a_tag_that_refuses_stops_before_the_anchors(dialect, builder, hub):
    ext, devices = await _live_cell(dialect)
    devices[TAG].set_result = 2  # its drone is armed

    body = await _request(ext, builder, hub, rateHz=83)

    assert (body["result"], body["reason"]) == (
        "incomplete",
        "Not every tag took the change; no anchor was written",
    )
    assert body["results"] == {
        str(TAG): {
            "written": "failed",
            "detail": f"RTLS tag {TAG} refused UWB_BLOCK_MS=12",
        }
    }
    assert all(_param(devices[sid], "UWB_BLOCK_MS") == 20 for sid in ANCHORS)


async def test_a_tag_that_does_not_answer_stops_too(
    dialect, builder, hub, autojump_clock
):
    ext, devices = await _live_cell(dialect)
    devices[TAG].respond_to_set = False

    body = await _request(ext, builder, hub, txGain=20, rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"][str(TAG)]["detail"] == (
        f"RTLS tag {TAG}: no answer to UWB_BLOCK_MS=12"
    )
    assert _param(devices[201], "UWB_TX_GAIN") == 47


async def test_a_preset_on_other_slots_is_refused_by_that_device(dialect, builder, hub):
    # 202 runs 16 slots of 1500 RSTU: the 12 ms block cannot hold that round
    ext, devices = await _live_cell(dialect)
    await ext.set_param(202, "UWB_SLOTS_RR", 16, "uint8")
    await ext.set_param(202, "UWB_SLOT_RSTU", 1500, "uint16")
    await _report(ext, devices, [202])

    body = await _request(ext, builder, hub, rateHz=83)

    assert (body["result"], body["reason"]) == (
        "incomplete",
        "Not every anchor took the change",
    )
    assert body["results"]["202"]["detail"] == (
        "RTLS anchor 202 refused UWB_BLOCK_MS=12"
    )
    assert body["results"]["201"] == {"written": "changed"}
    await _report(ext, devices)
    assert _states(ext)[202] == ("differ", None)
    assert _states(ext)[201] == ("ok", None)


async def test_tx_level_goes_to_anchors_only(dialect, builder, hub):
    ext, devices = await _live_cell(dialect)
    writes = _record_writes(ext)

    body = await _request(ext, builder, hub, txGain=30)

    assert body["result"] == "applied"
    assert writes[TAG] == [("UWB_BLOCK_MS", 20)]
    assert writes[202] == [("UWB_BLOCK_MS", 20), ("UWB_TX_GAIN", 30)]
    assert "UWB_TX_GAIN" not in devices[TAG].params
    await _report(ext, devices)
    assert _cell_settings(ext)["txGain"] == 30
    assert _states(ext)[202] == ("ok", None)


async def test_an_empty_request_brings_in_a_device_left_behind(dialect, builder, hub):
    ext, devices = await _live_cell(dialect)
    devices[202].set_result = 2
    await _request(ext, builder, hub, rateHz=83, txGain=30)
    devices[202].set_result = 0
    await _report(ext, devices)
    assert _cell_settings(ext)["differ"] == [202]
    writes = _record_writes(ext)

    body = await _request(ext, builder, hub)

    assert body["result"] == "applied"
    assert writes[202] == [("UWB_BLOCK_MS", 12), ("UWB_TX_GAIN", 30)]
    await _report(ext, devices)
    assert _cell_settings(ext)["differ"] == []


async def test_a_device_changed_elsewhere_is_written_again(dialect, builder, hub):
    # 202 was set to 12 ms by another client: the server's cache still
    # holds A0's 20 ms, so only the write itself can bring it back
    ext, devices = await _live_cell(dialect)
    set_fake_param(devices[202], "UWB_BLOCK_MS", 12, "uint16")
    await _report(ext, devices, [202])
    assert _states(ext)[202] == ("differ", None)

    body = await _request(ext, builder, hub)

    assert body["results"]["202"] == {"written": "changed"}
    await _report(ext, devices)
    assert _states(ext)[202] == ("ok", None)


async def test_an_earlier_writes_ack_does_not_count(
    dialect, builder, hub, autojump_clock
):
    # the tag's 12 ms write is lost; what comes back is the ack of an
    # earlier 30 ms write, which must not start the anchor phase
    ext, devices = await _live_cell(dialect)
    tag, handle = devices[TAG], devices[TAG].handle
    parser = dialect.MAVLink(None)

    def stale(data):
        message = (parser.parse_buffer(data) or [None])[0]
        if message is not None and message.get_type() == "PARAM_EXT_SET":
            value = raw_field_bytes(message, "param_value") or b""
            old = (30).to_bytes(2, "little") + value[2:]
            return [tag._param_ack(message.param_id, old, message.param_type, 0)]
        return handle(data)

    tag.handle = stale

    body = await _request(ext, builder, hub, rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"][str(TAG)]["detail"] == (
        f"RTLS tag {TAG}: no answer to UWB_BLOCK_MS=12"
    )
    assert all(_param(devices[sid], "UWB_BLOCK_MS") == 20 for sid in ANCHORS)


async def test_an_earlier_writes_ack_before_the_right_one_is_skipped(
    dialect, builder, hub
):
    # a late ack of an earlier 30 ms write arrives first, then the 12 ms ack
    ext, devices = await _live_cell(dialect)
    tag, handle = devices[TAG], devices[TAG].handle
    parser = dialect.MAVLink(None)

    def late_then_right(data):
        message = (parser.parse_buffer(data) or [None])[0]
        replies = handle(data)
        if message is not None and message.get_type() == "PARAM_EXT_SET":
            value = raw_field_bytes(message, "param_value") or b""
            old = (30).to_bytes(2, "little") + value[2:]
            replies = [tag._param_ack(message.param_id, old, message.param_type, 0)]
            replies += handle(data)
        return replies

    tag.handle = late_then_right

    body = await _request(ext, builder, hub, rateHz=83)

    assert body["result"] == "applied"
    assert all(_param(devices[sid], "UWB_BLOCK_MS") == 12 for sid in ANCHORS)


async def test_an_asleep_anchor_is_written_and_runs_it_at_its_start(
    dialect, builder, hub
):
    ext, devices = await _live_cell(dialect)
    await _report(ext, devices, [202], pwrst=3.0, cellcfg=-1.0, txgain=-1.0)
    assert _states(ext)[202] == ("unknown", None)

    body = await _request(ext, builder, hub, rateHz=33)

    assert body["results"]["202"] == {"written": "changed"}
    assert _param(devices[202], "UWB_BLOCK_MS") == 30
    await _report(ext, devices)  # it starts operating
    assert set(_states(ext).values()) == {("ok", None)}


async def test_refusals_write_nothing(dialect, builder, hub):
    ext, devices = await _live_cell(dialect)
    await _report(ext, devices, [202], lock=1.0)
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "Unlock the anchors first")

    await _report(ext, devices, [202])
    _authorize(ext)
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "A show is authorized")

    ext._cell_applying = True
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "An apply is running")
    ext._cell_applying = False

    ext._protocol.devices.clear()
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "No online devices")
    assert all(_param(device, "UWB_BLOCK_MS") == 20 for device in devices.values())


async def test_invalid_requests_are_rejected(dialect, builder, hub):
    ext, _ = await _live_cell(dialect)
    invalid = (
        {"rateHz": 100},
        {"rateHz": []},
        {"rateHz": 83.0},
        {"txGain": 128},
        {"txGain": -1},
        {"txGain": True},
        {"rateHz": None},
        {"txGain": None},
    )
    for body in invalid:
        response = await _request(ext, builder, hub, **body)
        assert response["type"] == "ACK-NAK", body


def _role_lost_from_dump(*, answers):
    def prepare(devices):
        devices[202].drop_from_list = {"UWB_ROLE"}
        devices[202].respond_to_read = answers

    return prepare


async def test_a_role_not_cached_is_read_once(dialect, builder, hub, autojump_clock):
    prepare = _role_lost_from_dump(answers=True)
    ext, devices = await _live_cell(dialect, prepare=prepare)
    body = await _request(ext, builder, hub, txGain=30)
    assert body["result"] == "applied"
    assert _param(devices[202], "UWB_TX_GAIN") == 30

    prepare = _role_lost_from_dump(answers=False)
    ext, devices = await _live_cell(dialect, prepare=prepare)
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == (
        "refused",
        "202: role not read yet",
    )
    assert _param(devices[201], "UWB_BLOCK_MS") == 20


async def test_a_device_whose_role_is_not_read_is_unknown(dialect):
    prepare = _role_lost_from_dump(answers=False)
    ext, devices = await _live_cell(dialect, prepare=prepare)
    await _report(ext, devices, [202], txgain=20.0)
    assert _states(ext)[202] == ("unknown", "role not read yet")
    assert 202 in _cell_settings(ext)["unknown"]


def _block_lost_from_a0_dump(*, answers):
    def prepare(devices):
        devices[201].drop_from_list = {"UWB_BLOCK_MS"}
        devices[201].respond_to_read = answers

    return prepare


async def test_a_left_out_rate_needs_a0s_block(dialect, builder, hub, autojump_clock):
    prepare = _block_lost_from_a0_dump(answers=True)
    ext, devices = await _live_cell(dialect, prepare=prepare)
    writes = _record_writes(ext)
    body = await _request(ext, builder, hub, txGain=30)
    assert body["result"] == "applied"
    assert writes[TAG] == [("UWB_BLOCK_MS", 20)]

    prepare = _block_lost_from_a0_dump(answers=False)
    ext, devices = await _live_cell(dialect, prepare=prepare)
    body = await _request(ext, builder, hub, txGain=30)
    assert (body["result"], body["reason"]) == (
        "refused",
        "A0's settings not read yet",
    )
    assert _param(devices[202], "UWB_TX_GAIN") == 47

    # a request that names the rate needs no read
    body = await _request(ext, builder, hub, rateHz=83)
    assert body["result"] == "applied"


async def test_a0s_role_and_block_lost_are_both_read(
    dialect, builder, hub, autojump_clock
):
    def prepare(devices):
        devices[201].drop_from_list = {"UWB_ROLE", "UWB_BLOCK_MS"}

    ext, devices = await _live_cell(dialect, prepare=prepare)
    writes = _record_writes(ext)
    body = await _request(ext, builder, hub, txGain=30)
    assert body["result"] == "applied"
    assert writes[TAG] == [("UWB_BLOCK_MS", 20)]


async def test_a_cached_cell_reads_nothing(dialect, builder, hub):
    ext, _ = await _live_cell(dialect)

    async def get_param(*args, **kwargs):
        raise AssertionError("read")

    ext.get_param = get_param
    assert (await _request(ext, builder, hub, rateHz=83))["result"] == "applied"


async def test_a_device_lost_before_its_write_reads_offline(dialect, builder, hub):
    ext, _ = await _live_cell(dialect)
    write = ext.set_param

    async def set_param(system_id, *args, **kwargs):
        if system_id == 202:
            ext._protocol.devices.pop(202)
        return await write(system_id, *args, **kwargs)

    ext.set_param = set_param
    body = await _request(ext, builder, hub, rateHz=83)

    assert body["results"]["202"]["detail"] == "RTLS anchor 202: offline"
