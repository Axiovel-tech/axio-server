"""Cell settings (X-RTLS-CELL, rtls/cell_settings.py): the state derived from
each device's stored parameters and reported ``cellcfg``, the apply that
writes, rolls back on a refusal and reboots the running devices with A0 last."""

import time

import pytest
import trio.testing
from anchor_lock_support import CODE_50, HEALTHY_A0, HEALTHY_A1, SESSION, _cell, _lock
from rtls_support import make_message
from rtlslink import ota
from rtlslink.cell_settings import code
from rtlslink.dialect import load_dialect

from flockwave.server.ext.rtls.verify import run_verify
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

CODE_83 = code({**{k: v for k, (v, _) in SESSION.items()}, "UWB_BLOCK_MS": 12})
TAG = 42


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
def resets(monkeypatch):
    """The SMP resets the apply sends, in order, as (host, port)."""
    sent = []
    monkeypatch.setattr(ota, "reset", lambda host, port: sent.append((host, port)))
    return sent


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


async def _running(ext, codes, tx=47, **stats):
    """Each device reports the code (anchors also the TX level) it runs;
    anchors operating unless ``stats`` say otherwise."""
    for system_id, running in codes.items():
        body = {"cellcfg": float(running)}
        if system_id != TAG:
            healthy = HEALTHY_A0 if system_id == 201 else HEALTHY_A1
            body = {**healthy, **stats, **body, "txgain": float(tx)}
        await ext._on_stats(system_id, body, time.monotonic())


async def _converged_cell(dialect, running=CODE_50):
    ext, devices = await _cell(dialect)
    await _running(ext, {TAG: running, 201: running, 202: running})
    return ext, devices


async def _request(ext, builder, hub, **body):
    message = make_message(builder, {"type": "X-RTLS-CELL", "action": "status"} | body)
    response = await ext._handle_RTLS_CELL(message, None, hub)
    return response.body


def _param(device, name):
    return int.from_bytes(device.params[name][0], "little")


def _states(ext):
    return {
        int(sid): (entry.get("cellSettings"), entry.get("cellSettingsDetail"))
        for sid, entry in ext._inf_status(time.monotonic()).items()
    }


async def test_a_converged_cell_reads_ok(dialect, builder, hub):
    ext, _ = await _converged_cell(dialect)
    body = await _request(ext, builder, hub)
    assert body["result"] == "status" and body["reason"] is None
    assert body["cellSettings"] == {
        "rateHz": 50,
        "blockMs": 20,
        "txGain": 47,
        "reference": 201,
        "differ": [],
        "restart": [],
        "unknown": [],
    }
    assert _states(ext) == {TAG: ("ok", None), 201: ("ok", None), 202: ("ok", None)}


async def test_states_compare_what_runs_with_what_a0_runs(dialect):
    ext, _ = await _cell(dialect)
    ext._stats.pop(TAG)
    await _running(ext, {201: CODE_50, 202: CODE_83})
    # 202 stores A0's settings but runs others; the tag has not reported
    assert _states(ext)[202] == ("restart", None)
    assert _states(ext)[TAG] == ("unknown", None)

    # A0 stores 83 Hz but still runs 50 Hz: a peer running 83 Hz is not with it
    await ext.set_param(201, "UWB_BLOCK_MS", 12, "uint16")
    await ext.set_param(202, "UWB_BLOCK_MS", 12, "uint16")
    await _running(ext, {TAG: CODE_50})
    states = _states(ext)
    assert states[201] == ("restart", None)
    assert states[202] == ("differ", None)
    assert states[TAG] == ("ok", None)
    # storing what A0 does not store: a reboot would not bring it in
    await ext.set_param(202, "UWB_BLOCK_MS", 30, "uint16")
    assert _states(ext)[202] == ("differ", None)


async def test_a_different_tx_level_differs(dialect):
    ext, _ = await _converged_cell(dialect)
    await ext.set_param(202, "UWB_TX_GAIN", 20, "uint8")
    assert _states(ext)[202] == ("differ", None)
    await _running(ext, {202: CODE_50}, tx=20)
    assert _states(ext)[202] == ("differ", "different TX level")


async def test_apply_writes_then_restarts_with_a0_last(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)

    body = await _request(ext, builder, hub, action="apply", rateHz=83)

    assert body["result"] == "applied" and body["reason"] is None
    assert body["results"] == {
        str(sid): {"written": "changed", "restarted": True} for sid in (TAG, 201, 202)
    }
    assert all(_param(device, "UWB_BLOCK_MS") == 12 for device in devices.values())
    assert resets[-1] == ("192.168.4.201", 1337)
    assert len(resets) == 3
    # nothing runs the new settings until the reboots report it
    assert body["cellSettings"]["rateHz"] == 83
    assert body["cellSettings"]["restart"] == [TAG, 201, 202]


async def test_apply_again_restarts_only_what_still_runs_the_old_settings(
    dialect, builder, hub, resets
):
    ext, _ = await _converged_cell(dialect)
    await _request(ext, builder, hub, action="apply", rateHz=83)
    await _running(ext, {TAG: CODE_83, 202: CODE_83})
    resets.clear()

    body = await _request(ext, builder, hub, action="apply", rateHz=83)

    assert body["result"] == "applied"
    assert body["results"][str(TAG)] == {"written": "unchanged", "restarted": False}
    assert body["results"]["201"] == {"written": "unchanged", "restarted": True}
    assert resets == [("192.168.4.201", 1337)]


async def test_a_refused_write_rolls_every_write_back(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    devices[TAG].set_result = 2  # an armed tag refuses the write

    body = await _request(ext, builder, hub, action="apply", rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"][str(TAG)]["written"] == "failed"
    assert body["results"]["201"] == {"written": "unchanged", "restarted": False}
    assert all(_param(device, "UWB_BLOCK_MS") == 20 for device in devices.values())
    assert resets == []


async def test_an_asleep_device_is_written_not_restarted(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    await _running(ext, {202: CODE_50}, pwrst=3.0)

    body = await _request(ext, builder, hub, action="apply", rateHz=33)

    assert body["results"]["202"] == {"written": "changed", "restarted": False}
    assert _param(devices[202], "UWB_BLOCK_MS") == 30
    assert ("192.168.4.202", 1337) not in resets


async def test_tx_level_goes_to_anchors_only(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)

    body = await _request(ext, builder, hub, action="apply", txGain=60)

    assert body["result"] == "applied"
    assert _param(devices[201], "UWB_TX_GAIN") == 60
    assert _param(devices[202], "UWB_TX_GAIN") == 60
    assert "UWB_TX_GAIN" not in devices[TAG].params
    # a changed TX level runs only after a reboot, though the code is the same
    assert body["results"]["202"] == {"written": "changed", "restarted": True}
    assert body["results"][str(TAG)] == {"written": "unchanged", "restarted": False}


async def test_apply_is_refused_while_locked_or_a_show_is_authorized(
    dialect, builder, hub, resets
):
    ext, devices = await _converged_cell(dialect)
    await _running(ext, {202: CODE_50}, lock=1.0)
    body = await _request(ext, builder, hub, action="apply", rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "Unlock the anchors first")

    await _running(ext, {202: CODE_50})

    class Show:
        authorized_to_start = True
        authorization_scope = AuthorizationScope.REHEARSAL

    class Api:
        def get_configuration(self):
            return Show()

    ext.app.import_api = lambda name: Api()
    body = await _request(ext, builder, hub, action="apply", rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "A show is authorized")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_invalid_requests_are_rejected(dialect, builder, hub):
    ext, _ = await _converged_cell(dialect)
    for body in ({"rateHz": 100}, {"txGain": 128}, {"txGain": -1}):
        message = make_message(
            builder, {"type": "X-RTLS-CELL", "action": "apply", **body}
        )
        response = await ext._handle_RTLS_CELL(message, None, hub)
        assert response.body["type"] == "ACK-NAK", body


async def test_lock_warns_and_verify_fails_on_a_device_that_differs(
    dialect, builder, hub
):
    ext, _ = await _converged_cell(dialect)
    await ext.set_param(202, "UWB_BLOCK_MS", 30, "uint16")

    body = await _lock(ext, builder, hub)
    assert body["result"] == "warnings"
    assert [(w["code"], w["detail"]) for w in body["warnings"]] == [
        ("cell-settings", "RTLS anchor 202: different cell settings")
    ]
    verify = await run_verify(ext)
    rule = next(rule for rule in verify["rules"] if rule["id"] == "cell")
    assert (rule["status"], rule["severity"]) == ("fail", "error")
    assert rule["detail"] == "RTLS anchor 202: different cell settings"


def _lose_acks(device, dialect, name, *, after=0):
    """``device`` stores every write of ``name`` but the acks after the first
    ``after`` are lost."""
    handle, parser, acked = device.handle, dialect.MAVLink(None), [0]

    def lossy(data):
        out = []
        for reply in handle(data):
            message = (parser.parse_buffer(reply) or [None])[0]
            if message is not None and message.get_type() == "PARAM_EXT_ACK":
                if message.param_id == name:
                    acked[0] += 1
                    if acked[0] > after:
                        continue
            out.append(reply)
        return out

    device.handle = lossy


async def test_an_unanswered_write_is_rolled_back_too(
    dialect, builder, hub, resets, autojump_clock
):
    ext, devices = await _converged_cell(dialect)
    _lose_acks(devices[202], dialect, "UWB_BLOCK_MS")

    body = await _request(ext, builder, hub, action="apply", rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"]["202"]["written"] == "failed"
    # 202 stored 12 ms though its ack was lost: the rollback restores it
    assert all(_param(device, "UWB_BLOCK_MS") == 20 for device in devices.values())
    assert resets == []


async def test_rollback_attempts_every_restore(
    dialect, builder, hub, resets, autojump_clock
):
    ext, devices = await _converged_cell(dialect)
    devices[TAG].set_result = 2  # an armed tag refuses
    _lose_acks(devices[202], dialect, "UWB_TX_GAIN", after=1)

    body = await _request(ext, builder, hub, action="apply", rateHz=83, txGain=60)

    # the TX restore's ack is lost; the block is restored all the same
    assert _param(devices[202], "UWB_BLOCK_MS") == 20
    assert _param(devices[202], "UWB_TX_GAIN") == 47
    assert body["results"]["202"]["detail"].startswith("rollback failed: ")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_one_apply_at_a_time(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    ext._cell_applying = True
    body = await _request(ext, builder, hub, action="apply", rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "An apply is running")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


def _dropped_from_dump(name, *, answers=True):
    def prepare(devices):
        devices[202].drop_from_list = {name}
        devices[202].respond_to_read = answers

    return prepare


async def test_a_setting_missing_from_the_dump_is_read_before_writing(
    dialect, builder, hub, resets
):
    ext, devices = await _cell(dialect, prepare=_dropped_from_dump("UWB_TX_GAIN"))
    await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})

    body = await _request(ext, builder, hub, action="apply", txGain=60)

    assert body["result"] == "applied"
    assert _param(devices[202], "UWB_TX_GAIN") == 60


async def test_a_setting_that_cannot_be_read_refuses_the_whole_apply(
    dialect, builder, hub, resets, autojump_clock
):
    for name in ("UWB_ROLE", "UWB_BLOCK_MS"):
        prepare = _dropped_from_dump(name, answers=False)
        ext, devices = await _cell(dialect, prepare=prepare)
        await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})

        body = await _request(ext, builder, hub, action="apply", rateHz=83)

        assert body["result"] == "refused", name
        assert body["reason"].endswith("202: settings not read yet"), name
        assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_a_preset_sets_block_slots_and_slot_length(dialect, builder, hub, resets):
    ext, devices = await _cell(dialect)
    for system_id in devices:
        await ext.set_param(system_id, "UWB_SLOTS_RR", 16, "uint8")
        await ext.set_param(system_id, "UWB_SLOT_RSTU", 1000, "uint16")
    assert (await _request(ext, builder, hub))["cellSettings"]["rateHz"] is None

    await _request(ext, builder, hub, action="apply", rateHz=83)

    for device in devices.values():
        assert _param(device, "UWB_BLOCK_MS") == 12
        assert _param(device, "UWB_SLOTS_RR") == 12
        assert _param(device, "UWB_SLOT_RSTU") == 1200


async def test_a_show_authorized_during_the_writes_stops_the_resets(
    dialect, builder, hub, resets
):
    ext, _ = await _converged_cell(dialect)
    authorized = []

    class Api:
        def get_configuration(self):
            class Show:
                authorized_to_start = bool(authorized)
                authorization_scope = AuthorizationScope.REHEARSAL

            return Show()

    ext.app.import_api = lambda name: Api()
    write = ext.set_param

    async def set_param(*args, **kwargs):
        authorized.append(True)  # another client authorizes rehearsal meanwhile
        return await write(*args, **kwargs)

    ext.set_param = set_param
    body = await _request(ext, builder, hub, action="apply", rateHz=83)

    assert body["result"] == "incomplete"
    assert body["reason"] == "A show is authorized; nothing was restarted"
    assert resets == []
    assert body["cellSettings"]["restart"] == [TAG, 201, 202]


async def test_resets_follow_the_state_after_the_writes(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    # 202 stores another channel; A0's power state is unknown
    await ext.set_param(202, "UWB_CHANNEL", 5, "uint8")
    await ext._on_stats(201, {"cellcfg": float(CODE_50), "txgain": 47.0}, 0.0)
    write = ext.set_param

    async def set_param(system_id, *args, **kwargs):
        ack = await write(system_id, *args, **kwargs)
        if system_id == TAG:  # the tag rebooted on its own right after its write
            await _running(ext, {TAG: CODE_83})
        return ack

    ext.set_param = set_param
    body = await _request(ext, builder, hub, action="apply", rateHz=83)

    assert body["result"] == "applied"
    assert resets == []
    assert _states(ext)[202] == ("differ", None)


async def test_no_online_device_refuses(dialect, builder, hub, resets):
    ext, _ = await _converged_cell(dialect)
    ext._protocol.devices.clear()
    body = await _request(ext, builder, hub, action="apply", rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "No online devices")


async def test_lock_warns_on_a_device_that_has_not_reported(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    ext._stats.pop(TAG)
    await _running(ext, {201: CODE_50, 202: CODE_50})

    body = await _lock(ext, builder, hub)

    assert [(w["code"], w["detail"]) for w in body["warnings"]] == [
        ("cell-settings", f"RTLS tag {TAG}: cell settings not reported")
    ]
