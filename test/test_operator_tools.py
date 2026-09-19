"""Operator reads, ordered writes, arming gates and fresh sensor results."""

from types import SimpleNamespace

import pytest
import trio
from rtlslink.protocol import RtlsDevice

from flockwave.server.ext.mavlink.operator_tools import (
    parameter_operation,
    run_operation,
    telemetry_snapshot,
    wire_value,
)
from flockwave.server.ext.rtls.association import associations


class UAV:
    id = "11"
    system_id = 11
    is_connected = True

    def __init__(self):
        self.values = {"A": (1.0, 6), "B": (2.0, 6), "C": (3.0, 9)}
        self.writes = []
        self.reads = []
        self.armed = False
        self.clamp = False
        self._operator_lock = trio.Lock()
        self.driver = SimpleNamespace(
            send_packet_with_retries=self.message,
            _operator_limiter=trio.CapacityLimiter(2),
        )

    async def get_parameter_info(self, name):
        self.reads.append(name)
        await trio.lowlevel.checkpoint()
        if name not in self.values:
            raise RuntimeError("missing parameter")
        return self.values[name]

    async def get_parameter(self, name, fetch=False):
        return (await self.get_parameter_info(name))[0]

    async def _set_parameter_single(self, name, value, *, param_type):
        self.writes.append(name)
        self.values[name] = (99.0 if self.clamp else value, param_type)

    async def message(self, packet, target, **kwds):
        return SimpleNamespace(
            base_mode=128 if self.armed else 0,
            system_status=3,
            custom_mode=4,
            MCU_temperature=5100,
            temperature=4200,
            get_srcSystem=lambda: 11,
            get_srcComponent=lambda: 1,
        )


async def test_partial_reads_cannot_be_compared_or_applied():
    uav = UAV()
    result = await parameter_operation(uav, "apply", values={"A": 5, "MISSING": 9})
    assert not result["complete"]
    assert result["values"] == {"A": 1}
    assert result["errors"] == {"MISSING": "missing parameter"}
    assert not uav.writes


async def test_apply_orders_only_changes_and_verifies_readback():
    uav = UAV()
    result = await parameter_operation(uav, "apply", values={"B": 5, "A": 1, "C": 0.1})
    assert result["complete"]
    assert uav.writes == ["B", "C"]
    assert [row["status"] for row in result["changes"]] == [
        "applied",
        "unchanged",
        "applied",
    ]
    assert uav.reads.count("B") == 2
    assert uav.reads.count("A") == 1


async def test_arming_and_rejected_value_stop_ordered_writes():
    uav = UAV()
    uav.armed = True
    result = await parameter_operation(uav, "apply", values={"A": 5, "B": 6})
    assert not result["complete"] and not uav.writes
    uav.armed = False
    uav.clamp = True
    result = await parameter_operation(uav, "apply", values={"A": 5, "B": 6})
    assert not result["complete"]
    assert uav.writes == ["A"]
    assert result["changes"][0]["actual"] == 99


async def test_invalid_later_value_prevents_prior_changes():
    uav = UAV()
    with pytest.raises(ValueError, match="fractional"):
        await parameter_operation(uav, "apply", values={"A": 5, "B": 6.1})
    assert not uav.writes


async def test_explicit_write_order_survives_sorted_json_keys():
    uav = UAV()
    result = await parameter_operation(
        uav, "apply", names=["B", "A"], values={"A": 5, "B": 6}
    )
    assert result["complete"] and uav.writes == ["B", "A"]
    with pytest.raises(ValueError, match="match desired"):
        await parameter_operation(uav, "apply", names=["A"], values={"A": 1, "B": 2})


async def test_temperatures_have_units_sources_and_age():
    result = await telemetry_snapshot(UAV())
    assert result["temperatures"]["MCU"]["value"] == 51.0
    assert result["temperatures"]["IMU"]["value"] == 42.0
    assert result["temperatures"]["barometer"]["unit"] == "degC"
    assert result["temperatures"]["MCU"]["source"] == "MCU_STATUS"
    assert result["temperatures"]["MCU"]["age_s"] >= 0
    assert result["unavailable"] == []
    assert result["armed"] is False


@pytest.mark.parametrize("value", [16_777_217, 2_147_483_647])
async def test_integer_rounding_rejected_before_any_writes(value):
    uav = UAV()
    with pytest.raises(ValueError, match="represented exactly"):
        await parameter_operation(uav, "apply", values={"A": 5, "B": value})
    assert not uav.writes
    assert wire_value(16_777_216, 6) == 16_777_216


async def test_read_deadline_preserves_success_and_identifies_missing(autojump_clock):
    uav = UAV()
    original = uav.get_parameter_info

    async def read(name):
        if name == "B":
            await trio.sleep_forever()
        return await original(name)

    uav.get_parameter_info = read
    result = await run_operation(uav, "read", names=["A", "B"], timeout=1)
    assert not result["complete"]
    assert result["values"] == {"A": 1}
    assert set(result["errors"]) == {"B"}
    assert result["phase"] == "reading"
    assert "write" not in result["error"]


async def test_enclosing_command_deadline_still_returns_partial_result(autojump_clock):
    uav = UAV()
    original = uav.get_parameter_info

    async def read(name):
        if name == "B":
            await trio.sleep_forever()
        return await original(name)

    uav.get_parameter_info = read
    with trio.fail_after(0.5):
        result = await run_operation(uav, "read", names=["A", "B"], timeout=120)
    assert result["values"] == {"A": 1}
    assert not result["complete"]


async def test_telemetry_deadline_retains_fresh_sample(monkeypatch, autojump_clock):
    from flockwave.server.ext.mavlink import operator_tools

    async def message(uav, name, message_id):
        if name != "MCU_STATUS":
            await trio.sleep_forever()
        return SimpleNamespace(MCU_temperature=5100)

    monkeypatch.setattr(operator_tools, "request_message", message)
    result = await run_operation(UAV(), "telemetry", names=["MCU", "IMU"], timeout=1)
    assert not result["complete"]
    assert result["temperatures"]["MCU"]["value"] == 51
    assert result["temperatures"]["MCU"]["age_s"] >= 0
    assert result["unavailable"] == ["IMU"]
    assert "armed" not in result


@pytest.mark.parametrize("gate", ["uav", "fleet"])
async def test_expired_queued_apply_never_starts(gate, autojump_clock):
    uav = UAV()
    lock = uav._operator_lock if gate == "uav" else trio.CapacityLimiter(1)
    if gate == "fleet":
        uav.driver._operator_limiter = lock
    results = []

    async def apply():
        results.append(await run_operation(uav, "apply", values={"A": 5}, timeout=1))

    async with lock:
        async with trio.open_nursery() as nursery:
            nursery.start_soon(apply)
    assert results[0]["phase"] == "queued"
    assert not results[0]["complete"]
    assert not uav.reads and not uav.writes


async def test_apply_deadline_preserves_prior_write_and_marks_pending(autojump_clock):
    uav = UAV()
    original = uav._set_parameter_single

    async def write(name, value, *, param_type):
        await original(name, value, param_type=param_type)
        if name == "B":
            await trio.sleep_forever()

    uav._set_parameter_single = write
    result = await run_operation(uav, "apply", values={"A": 5, "B": 6}, timeout=1)
    assert not result["complete"]
    assert result["changes"][0]["status"] == "applied"
    assert result["changes"][1]["status"] == "unverified"
    assert "unknown outcome" in result["error"]


def device(tag, fc, state=2, host=None):
    value = RtlsDevice(tag, 197, (host or f"127.0.0.{tag}", 3333), 10)
    value.params = {"FC_SYS_ID": bytes([fc]), "FC_STATE": bytes([state])}
    value.param_types = {"FC_SYS_ID": 1, "FC_STATE": 1}
    value.param_received_at = {"FC_SYS_ID": 10, "FC_STATE": 10}
    return value


def test_sleep_identity_survives_fc_disconnection_without_inventing_server_id():
    devices = {194: device(194, 11)}
    mapping, metadata = associations(devices, {}, {"0011": 11}, now=11, max_age=6)
    assert mapping == {194: "0011"}
    assert metadata[194]["state"] == "remembered"
    mapping, metadata = associations(devices, {}, {}, now=11, max_age=6)
    assert mapping == {}
    assert metadata[194]["system_id"] == 11


def test_duplicate_pairing_and_live_source_mismatch_cannot_target_a_drone():
    devices = {194: device(194, 11), 195: device(195, 11)}
    mapping, metadata = associations(devices, {}, {"11": 11}, now=11, max_age=6)
    assert mapping == {}
    assert all(row["state"] == "ambiguous" for row in metadata.values())
    mapping, metadata = associations(
        {194: devices[194]}, {"11": ("127.0.0.9", 14550)}, {"11": 11}, now=11, max_age=6
    )
    assert mapping == {}
    assert metadata[194]["state"] == "ambiguous"


def test_old_live_metadata_is_remembered():
    mapping, metadata = associations(
        {194: device(194, 11, 1)}, {}, {"11": 11}, now=20, max_age=6
    )
    assert mapping == {194: "11"}
    assert metadata[194]["state"] == "remembered"
