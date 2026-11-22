"""Cell settings (X-RTLS-CELL, rtls/cell_settings.py): the state derived from
each device's stored parameters and reported ``cellcfg``, the apply that
writes, rolls back on a refusal and reboots the running devices with A0 last."""

import time

import pytest
from anchor_lock_support import HEALTHY_A0, HEALTHY_A1, _cell, _lock
from rtls_support import make_message, set_fake_param
from rtlslink.cell_settings import code
from rtlslink.dialect import load_dialect

from flockwave.server.ext.rtls import cell_settings
from flockwave.server.ext.rtls.verify import run_verify
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder

SESSION = {
    "UWB_CHANNEL": 9,
    "UWB_SESSION_ID": 0x11223344,
    "UWB_SLOT_RSTU": 1200,
    "UWB_SLOTS_RR": 12,
}
CODE_50 = code({**SESSION, "UWB_BLOCK_MS": 20})
CODE_83 = code({**SESSION, "UWB_BLOCK_MS": 12})
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

    async def reset(host, port):
        sent.append((host, port))

    monkeypatch.setattr(cell_settings, "_reset_device", reset)
    return sent


def _session_params(devices, block_ms=20):
    for system_id, device in devices.items():
        set_fake_param(device, "UWB_CHANNEL", 9, "uint8")
        set_fake_param(device, "UWB_SESSION_ID", 0x11223344, "uint32")
        set_fake_param(device, "UWB_BLOCK_MS", block_ms, "uint16")
        set_fake_param(device, "UWB_SLOT_RSTU", 1200, "uint16")
        set_fake_param(device, "UWB_SLOTS_RR", 12, "uint8")
        if system_id != TAG:
            set_fake_param(device, "UWB_TX_GAIN", 47, "uint8")


async def _running(ext, codes):
    """Each device reports the code it runs (anchors operating)."""
    for system_id, running in codes.items():
        stats = {"cellcfg": float(running)}
        if system_id != TAG:
            stats.update(HEALTHY_A0 if system_id == 201 else HEALTHY_A1)
        await ext._on_stats(system_id, stats, time.monotonic())


async def _converged_cell(dialect, running=CODE_50):
    ext, devices = await _cell(dialect, prepare=_session_params)
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


async def test_states_follow_stored_and_running_codes(dialect):
    ext, _ = await _cell(dialect, prepare=_session_params)
    await _running(ext, {201: CODE_50, 202: CODE_83})
    # 202 stores A0's settings but runs others; the tag has not reported
    assert _states(ext)[202] == ("restart", None)
    assert _states(ext)[TAG] == ("unknown", None)

    await ext.set_param(202, "UWB_BLOCK_MS", 30, "uint16")
    await ext.set_param(201, "UWB_TX_GAIN", 20, "uint8")
    states = _states(ext)
    assert states[202] == ("differ", None)
    # TX is not in the code: A0 stays ok, 202 now differs from its level
    assert states[201] == ("ok", None)
    await ext.set_param(202, "UWB_BLOCK_MS", 20, "uint16")
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
    await ext._on_stats(
        202, {**HEALTHY_A1, "pwrst": 3.0, "cellcfg": float(CODE_50)}, 0.0
    )

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
    await ext._on_stats(
        202, {**HEALTHY_A1, "lock": 1.0, "cellcfg": float(CODE_50)}, 0.0
    )
    body = await _request(ext, builder, hub, action="apply", rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "Unlock the anchors first")

    await ext._on_stats(202, {**HEALTHY_A1, "cellcfg": float(CODE_50)}, 0.0)

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
