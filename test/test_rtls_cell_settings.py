"""Cell settings (X-RTLS-CELL, rtls/cell_state.py and rtls/cell_settings.py):
the state derived from each device's stored parameters and reported
``cellcfg``, the lock warning and Verify rule, and an apply that writes and
reboots the running devices with A0 last."""

import pytest
import trio.testing
from anchor_lock_support import _cell, _lock
from cell_settings_support import (
    CODE_50,
    CODE_83,
    TAG,
    _a_cell_without_a0,
    _boots,
    _cell_settings,
    _converged_cell,
    _dropped_from_dump,
    _param,
    _record_writes,
    _request,
    _running,
    _schedule,
    _states,
    _steps,
)
from rtlslink import ota
from rtlslink.dialect import load_dialect

from flockwave.server.ext.rtls.verify import run_verify
from flockwave.server.ext.show.config import AuthorizationScope
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
def resets(monkeypatch):
    """The SMP resets the apply sends, in order, as (host, port)."""
    sent = []
    monkeypatch.setattr(ota, "reset", lambda host, port: sent.append((host, port)))
    return sent


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


async def test_a_converged_cell_reads_ok(dialect):
    ext, _ = await _converged_cell(dialect)
    assert _cell_settings(ext) == {
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

    body = await _request(ext, builder, hub, rateHz=83)

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
    await _request(ext, builder, hub, rateHz=83)
    await _running(ext, {TAG: CODE_83, 202: CODE_83})
    resets.clear()

    body = await _request(ext, builder, hub, rateHz=83)

    assert body["result"] == "applied"
    assert body["results"][str(TAG)] == {"written": "unchanged", "restarted": False}
    assert body["results"]["201"] == {"written": "unchanged", "restarted": True}
    assert resets == [("192.168.4.201", 1337)]


async def test_an_asleep_device_is_written_not_restarted(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    await _running(ext, {202: CODE_50}, pwrst=3.0)

    body = await _request(ext, builder, hub, rateHz=33)

    assert body["results"]["202"] == {"written": "changed", "restarted": False}
    assert _param(devices[202], "UWB_BLOCK_MS") == 30
    assert ("192.168.4.202", 1337) not in resets


async def test_tx_level_goes_to_anchors_only(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)

    body = await _request(ext, builder, hub, txGain=60)

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
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "Unlock the anchors first")

    await _running(ext, {202: CODE_50})

    class Show:
        authorized_to_start = True
        authorization_scope = AuthorizationScope.REHEARSAL

    class Api:
        def get_configuration(self):
            return Show()

    ext.app.import_api = lambda name: Api()
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "A show is authorized")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_invalid_requests_are_rejected(dialect, builder, hub):
    ext, _ = await _converged_cell(dialect)
    for body in ({"rateHz": 100}, {"txGain": 128}, {"txGain": -1}):
        response = await _request(ext, builder, hub, **body)
        assert response["type"] == "ACK-NAK", body


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


async def test_a_preset_sets_block_slots_and_slot_length(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    for system_id in devices:
        await ext.set_param(system_id, "UWB_SLOTS_RR", 16, "uint8")
        await ext.set_param(system_id, "UWB_SLOT_RSTU", 1000, "uint16")
    assert _cell_settings(ext)["rateHz"] is None
    start = _schedule(devices[TAG])
    writes = _record_writes(ext)

    await _request(ext, builder, hub, rateHz=83)

    for device in devices.values():
        assert _param(device, "UWB_BLOCK_MS") == 12
        assert _param(device, "UWB_SLOTS_RR") == 12
        assert _param(device, "UWB_SLOT_RSTU") == 1200
    # the shorter block comes after the round shrinks: a device cut off
    # midway still boots
    assert writes[TAG][0] == ("UWB_SLOTS_RR", 12)
    assert all(map(_boots, _steps(start, writes[TAG])))


async def test_a_longer_block_is_written_first(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    for system_id in devices:
        await ext.set_param(system_id, "UWB_BLOCK_MS", 12, "uint16")
        await ext.set_param(system_id, "UWB_SLOTS_RR", 10, "uint8")
    writes = _record_writes(ext)

    await _request(ext, builder, hub, rateHz=50)

    assert [name for name, _ in writes[TAG]] == ["UWB_BLOCK_MS", "UWB_SLOTS_RR"]


async def test_no_online_device_refuses(dialect, builder, hub, resets):
    ext, _ = await _converged_cell(dialect)
    ext._protocol.devices.clear()
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "No online devices")


async def test_lock_warns_on_a_device_that_has_not_reported(dialect, builder, hub):
    ext, _ = await _cell(dialect)
    ext._stats.pop(TAG)
    await _running(ext, {201: CODE_50, 202: CODE_50})

    body = await _lock(ext, builder, hub)

    assert [(w["code"], w["detail"]) for w in body["warnings"]] == [
        ("cell-settings", f"RTLS tag {TAG}: cell settings not reported")
    ]


async def test_without_a0_nothing_is_applied_and_lock_and_verify_say_so(
    dialect, builder, hub, resets
):
    ext, devices = await _a_cell_without_a0(dialect)

    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "A0 is offline")
    assert _param(devices[202], "UWB_BLOCK_MS") == 20 and resets == []

    lock = await _lock(ext, builder, hub)
    assert [(w["code"], w["device"], w["detail"]) for w in lock["warnings"]] == [
        ("cell-settings", None, "Cell settings: no A0 to compare")
    ]
    verify = await run_verify(ext)
    rule = next(rule for rule in verify["rules"] if rule["id"] == "cell")
    assert (rule["status"], rule["detail"]) == (
        "fail",
        "Cell settings: no A0 to compare",
    )


async def test_without_a_reference_tx_the_tx_comparison_is_unknown(dialect):
    ext, _ = await _converged_cell(dialect)
    ext._stats[201].pop("txGain")
    assert _states(ext)[202] == ("unknown", None)
    assert _states(ext)[TAG] == ("ok", None)


async def test_a_cached_cell_reads_nothing(dialect, builder, hub, resets):
    ext, _ = await _converged_cell(dialect)

    async def get_param(*args, **kwargs):
        raise AssertionError("read")

    ext.get_param = get_param
    assert (await _request(ext, builder, hub, rateHz=83))["result"] == "applied"


async def test_the_role_comes_from_the_live_device(dialect, builder, hub, resets):
    # 202 was a tag before; the anchor now at its id has not listed its role
    prepare = _dropped_from_dump("UWB_ROLE", answers=False)
    ext, devices = await _cell(dialect, prepare=prepare)
    ext._retained_params[202] = {"UWB_ROLE": 1}
    devices[202].respond_to_read = True
    await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})

    body = await _request(ext, builder, hub, txGain=60)

    assert body["result"] == "applied"
    assert _param(devices[202], "UWB_TX_GAIN") == 60
