"""Cell settings apply (X-RTLS-CELL) when something goes wrong: refused, lost
or unanswered writes and their rollback, settings not read yet, and the state
changing while the apply runs."""

import pytest
import trio.testing
from anchor_lock_support import _cell
from cell_settings_support import (
    CODE_50,
    CODE_83,
    TAG,
    _boots,
    _converged_cell,
    _dropped_from_dump,
    _lose_acks,
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


async def test_a_refused_write_rolls_every_write_back(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    devices[TAG].set_result = 2  # an armed tag refuses the write

    body = await _request(ext, builder, hub, rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"][str(TAG)]["written"] == "failed"
    assert body["results"]["201"] == {"written": "unchanged", "restarted": False}
    assert all(_param(device, "UWB_BLOCK_MS") == 20 for device in devices.values())
    assert resets == []


async def test_an_unanswered_write_is_rolled_back_too(
    dialect, builder, hub, resets, autojump_clock
):
    ext, devices = await _converged_cell(dialect)
    _lose_acks(devices[202], dialect, "UWB_BLOCK_MS")

    body = await _request(ext, builder, hub, rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"]["202"]["written"] == "failed"
    # 202 stored 12 ms though its ack was lost: the rollback restores it
    assert all(_param(device, "UWB_BLOCK_MS") == 20 for device in devices.values())
    assert resets == []


async def test_every_stored_step_boots_on_the_way_and_back(
    dialect, builder, hub, resets, autojump_clock
):
    # 8 slots of 2 ms in 20 ms: twelve slots of that length outgrow the block
    ext, devices = await _converged_cell(dialect)
    for system_id in devices:
        await ext.set_param(system_id, "UWB_SLOTS_RR", 8, "uint8")
        await ext.set_param(system_id, "UWB_SLOT_RSTU", 2400, "uint16")
    start = _schedule(devices[TAG])
    _lose_acks(devices[TAG], dialect, "UWB_SLOTS_RR")
    writes = _record_writes(ext)

    body = await _request(ext, builder, hub, rateHz=83)

    # the tag stored its last write, the slot count, though the ack was lost
    assert body["result"] == "incomplete"
    steps = _steps(start, writes[TAG])
    assert (12, 12, 1200) in steps
    assert steps[-1] == tuple(start.values())
    assert all(map(_boots, steps))


async def test_rollback_attempts_every_restore(
    dialect, builder, hub, resets, autojump_clock
):
    ext, devices = await _converged_cell(dialect)
    devices[TAG].set_result = 2  # an armed tag refuses
    _lose_acks(devices[202], dialect, "UWB_TX_GAIN", after=1)

    body = await _request(ext, builder, hub, rateHz=83, txGain=60)

    # the TX restore's ack is lost; the block is restored all the same
    assert _param(devices[202], "UWB_BLOCK_MS") == 20
    assert _param(devices[202], "UWB_TX_GAIN") == 47
    assert body["results"]["202"]["detail"].startswith("rollback failed: ")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_one_apply_at_a_time(dialect, builder, hub, resets):
    ext, devices = await _converged_cell(dialect)
    ext._cell_applying = True
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "An apply is running")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_a_setting_missing_from_the_dump_is_read_before_writing(
    dialect, builder, hub, resets
):
    ext, devices = await _cell(dialect, prepare=_dropped_from_dump("UWB_TX_GAIN"))
    await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})

    body = await _request(ext, builder, hub, txGain=60)

    assert body["result"] == "applied"
    assert _param(devices[202], "UWB_TX_GAIN") == 60


async def test_a_setting_that_cannot_be_read_refuses_the_whole_apply(
    dialect, builder, hub, resets, autojump_clock
):
    for name in ("UWB_ROLE", "UWB_BLOCK_MS"):
        prepare = _dropped_from_dump(name, answers=False)
        ext, devices = await _cell(dialect, prepare=prepare)
        await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})

        body = await _request(ext, builder, hub, rateHz=83)

        assert body["result"] == "refused", name
        assert body["reason"].endswith("202: settings not read yet"), name
        assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


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
    body = await _request(ext, builder, hub, rateHz=83)

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
    body = await _request(ext, builder, hub, rateHz=83)

    assert resets == []
    assert _states(ext)[202] == ("differ", None)
    # A0 was written but may be running: it is left for the operator
    assert (body["result"], body["reason"]) == (
        "incomplete",
        "Not every device restarted",
    )
    assert body["results"]["201"]["detail"] == "restart needed"
    assert "detail" not in body["results"][str(TAG)]
    assert body["results"]["202"]["detail"] == "different channel or session than A0"


async def test_an_awake_device_that_does_not_report_is_not_called_applied(
    dialect, builder, hub, resets
):
    ext, _ = await _converged_cell(dialect)
    ext._stats.pop(TAG)

    body = await _request(ext, builder, hub, rateHz=83)

    assert body["result"] == "incomplete"
    assert body["results"][str(TAG)] == {
        "written": "changed",
        "restarted": False,
        "detail": "restart needed",
    }
    assert len(resets) == 2


async def test_refusals_are_checked_after_the_reads(
    dialect, builder, hub, resets, autojump_clock
):
    ext, devices = await _cell(dialect, prepare=_dropped_from_dump("UWB_TX_GAIN"))
    await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})
    read = ext.get_param

    async def get_param(*args, **kwargs):
        await _running(ext, {201: CODE_50}, lock=1.0)  # locked during the read
        return await read(*args, **kwargs)

    ext.get_param = get_param
    body = await _request(ext, builder, hub, txGain=60)
    assert (body["result"], body["reason"]) == ("refused", "Unlock the anchors first")

    ext, devices = await _cell(dialect, prepare=_dropped_from_dump("UWB_TX_GAIN"))
    await _running(ext, {TAG: CODE_50, 201: CODE_50, 202: CODE_50})

    async def get_param_then_lose_all(*args, **kwargs):
        ext._protocol.devices.clear()
        raise KeyError(args[0])

    ext.get_param = get_param_then_lose_all
    body = await _request(ext, builder, hub, rateHz=83)
    assert (body["result"], body["reason"]) == ("refused", "No online devices")
    assert _param(devices[201], "UWB_BLOCK_MS") == 20 and resets == []


async def test_a_device_lost_before_its_write_reads_offline(
    dialect, builder, hub, resets
):
    ext, _ = await _converged_cell(dialect)
    write = ext.set_param

    async def set_param(system_id, *args, **kwargs):
        if system_id == 202:
            ext._protocol.devices.pop(202)
        return await write(system_id, *args, **kwargs)

    ext.set_param = set_param
    body = await _request(ext, builder, hub, rateHz=83)

    assert body["results"]["202"]["detail"] == "RTLS anchor 202: offline"
