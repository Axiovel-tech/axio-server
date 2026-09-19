"""Verified operator operations over the server's existing MAVLink connection."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from time import monotonic
from typing import TYPE_CHECKING, Literal, cast

import trio

from .types import spec

if TYPE_CHECKING:
    from .driver import MAVLinkUAV


@dataclass
class ParameterSnapshot:
    values: dict[str, float] = field(default_factory=dict)
    types: dict[str, int] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)

    @property
    def complete(self) -> bool:
        return not self.errors


def validate_names(names: list[str]) -> None:
    if not names or len(names) > 128:
        raise ValueError("select between 1 and 128 parameters")
    if any(
        not name
        or len(name) > 16
        or not name.isascii()
        or not all(char.isalnum() or char == "_" for char in name)
        for name in names
    ):
        raise ValueError(
            "parameter names must be ASCII identifiers of at most 16 bytes"
        )


def wire_value(value: float, param_type: int) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(value)
    ):
        raise ValueError("parameter values must be finite numbers")
    if 1 <= param_type <= 8:
        bits = (8, 8, 16, 16, 32, 32, 64, 64)[param_type - 1]
        signed = param_type % 2 == 0
        minimum = -(2 ** (bits - 1)) if signed else 0
        maximum = 2 ** (bits - int(signed)) - 1
        if value != int(value) or not minimum <= value <= maximum:
            raise ValueError(
                "integer parameter value is fractional or outside its wire bounds"
            )
    try:
        # ArduPilot uses the C-cast float32 PARAM_VALUE representation.
        encoded = struct.unpack("<f", struct.pack("<f", value))[0]
    except (OverflowError, struct.error) as error:
        raise ValueError("parameter value exceeds float32 range") from error
    if 1 <= param_type <= 8 and encoded != value:
        raise ValueError("integer parameter cannot be represented exactly on the wire")
    return encoded


async def read_parameters(
    uav: MAVLinkUAV, names: list[str], result: ParameterSnapshot | None = None
) -> ParameterSnapshot:
    validate_names(names)
    if result is None:
        result = ParameterSnapshot()
    result.errors.update(dict.fromkeys(names, "parameter reply not received"))
    limiter = trio.CapacityLimiter(4)

    async def read(name: str) -> None:
        async with limiter:
            try:
                with trio.fail_after(3):
                    value, type_code = await uav.get_parameter_info(name)
                result.values[name], result.types[name] = value, type_code
                result.errors.pop(name, None)
            except (trio.TooSlowError, OSError, RuntimeError) as error:
                result.errors[name] = str(error) or "parameter reply timed out"

    async with trio.open_nursery() as nursery:
        for name in dict.fromkeys(names):
            nursery.start_soon(read, name)
    return result


async def request_message(uav: MAVLinkUAV, name: str, message_id: int):
    """One-shot requests preserve stream rates and wait for the actual message."""
    message = await uav.driver.send_packet_with_retries(
        spec.command_long(
            command=512,
            confirmation=0,
            param1=message_id,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        ),
        uav,
        wait_for_response=getattr(spec, name.lower())(),
        timeout=0.7,
        retries=2,
    )

    if message.get_srcSystem() != uav.system_id or message.get_srcComponent() != 1:
        raise ValueError("reply was not from the selected flight controller")
    return message


async def require_disarmed(uav: MAVLinkUAV) -> None:
    heartbeat = await request_message(uav, "HEARTBEAT", 0)
    if heartbeat.base_mode & 128:
        raise ValueError("parameter changes require a disarmed flight controller")


async def parameter_operation(
    uav: MAVLinkUAV,
    operation: Literal["read", "compare", "apply"],
    names: list[str] | None = None,
    values: dict[str, float] | None = None,
    result: dict[str, object] | None = None,
) -> dict[str, object]:
    desired = values or {}
    selected = (names or list(desired)) if operation != "read" else names or []
    validate_names(selected)
    if operation != "read":
        if len(set(selected)) != len(selected) or set(selected) != set(desired):
            raise ValueError("ordered parameter names must match desired values")
        for value in desired.values():
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError("desired parameter values must be finite numbers")
    if result is None:
        result = {}
    before = ParameterSnapshot()
    result["before" if operation == "apply" else "values"] = before.values
    result.update(
        types=before.types,
        errors=before.errors,
        complete=False,
        phase="reading",
    )
    await read_parameters(uav, selected, before)
    result["complete"] = before.complete
    if operation == "read" or not before.complete:
        return result
    expected = {
        name: wire_value(desired[name], before.types[name]) for name in selected
    }
    differences = {
        name: {"actual": before.values[name], "expected": value}
        for name, value in expected.items()
        if before.values[name] != value
    }
    result["initial_differences" if operation == "apply" else "differences"] = (
        differences
    )
    if operation == "compare":
        result["consistent"] = not differences
        return result
    changes: list[dict[str, object]] = []
    result["changes"] = changes
    result["phase"] = "applying"
    for name, value in expected.items():
        if before.values[name] == value:
            changes.append({"name": name, "status": "unchanged", "actual": value})
            continue
        change: dict[str, object] = {
            "name": name,
            "before": before.values[name],
            "requested": value,
            "status": "not-started",
        }
        changes.append(change)
        try:
            await require_disarmed(uav)
            change["status"] = "unverified"
            await uav._set_parameter_single(name, value, param_type=before.types[name])
            actual = await uav.get_parameter(name, fetch=True)
            change["actual"] = actual
            if actual != value:
                raise ValueError("fresh readback differs from requested value")
            change["status"] = "applied"
        except (trio.TooSlowError, ValueError, RuntimeError, OSError) as error:
            if change["status"] == "not-started" or "actual" in change:
                change["status"] = "failed"
            change["error"] = str(error) or "write/readback timed out; outcome unknown"
            result["complete"] = False
            break
    return result


# Temperature fields are centidegrees Celsius, except named MCU temp which
# is not standardized and is intentionally not guessed here.
_TEMPERATURE_MESSAGES = (
    ("MCU_STATUS", 11039, "MCU", False),
    ("RAW_IMU", 27, "IMU", True),
    ("SCALED_IMU2", 116, "IMU2", True),
    ("SCALED_IMU3", 129, "IMU3", True),
    ("SCALED_PRESSURE", 29, "barometer", False),
    ("SCALED_PRESSURE2", 137, "barometer2", False),
)


async def telemetry_snapshot(
    uav: MAVLinkUAV,
    selected: list[str] | None = None,
    result: dict[str, object] | None = None,
) -> dict[str, object]:
    """Fresh temperatures and heartbeat. Missing sensors remain unavailable."""
    selected = selected or ["MCU", "IMU", "barometer"]
    if not set(selected) <= {item[2] for item in _TEMPERATURE_MESSAGES}:
        raise ValueError("unknown temperature sensor")
    sensors: dict[str, dict[str, float | str | None]] = {}
    if result is None:
        result = {}
    result.update(complete=False, phase="telemetry", temperatures=sensors)
    for name, _, label, _ in _TEMPERATURE_MESSAGES:
        if label in selected:
            sensors[label] = {
                "value": None,
                "unit": "degC",
                "source": name,
                "error": "sensor reply not received",
            }
    received: dict[str, float] = {}
    limiter = trio.CapacityLimiter(2)

    async def read(name: str, message_id: int, label: str, zero_missing: bool) -> None:
        async with limiter:
            started = monotonic()
            try:
                message = await request_message(uav, name, message_id)
                raw = getattr(
                    message,
                    "MCU_temperature" if name == "MCU_STATUS" else "temperature",
                    None,
                )
                if raw is None or (zero_missing and raw == 0):
                    raise ValueError("temperature unavailable in this sensor message")
                received[label] = monotonic()
                sensors[label] = {
                    "value": raw / 100.0,
                    "unit": "degC",
                    "source": name,
                    "age_s": 0.0,
                    "request_s": monotonic() - started,
                }
            except (trio.TooSlowError, ValueError, RuntimeError, OSError) as error:
                sensors[label] = {
                    "value": None,
                    "unit": "degC",
                    "source": name,
                    "error": str(error) or "sensor did not respond",
                }

    try:
        async with trio.open_nursery() as nursery:
            for item in _TEMPERATURE_MESSAGES:
                if item[2] in selected:
                    nursery.start_soon(read, *item)
        heartbeat = await request_message(uav, "HEARTBEAT", 0)
        result.update(
            complete=True,
            armed=bool(heartbeat.base_mode & 128),
            system_status=heartbeat.system_status,
            custom_mode=heartbeat.custom_mode,
        )
    finally:
        for label, stamp in received.items():
            sensors[label]["age_s"] = monotonic() - stamp
        result["unavailable"] = [
            label for label, data in sensors.items() if data["value"] is None
        ]
    return result


async def run_operation(
    uav: MAVLinkUAV,
    operation: str,
    *,
    names: list[str] | None = None,
    values: dict[str, float] | None = None,
    timeout: float = 30,
) -> dict[str, object]:
    if not 0 < timeout <= 120:
        raise ValueError("timeout must be within 0..120 seconds")
    if not uav.is_connected:
        raise ValueError("flight controller is disconnected")
    started = monotonic()
    result: dict[str, object] = {"complete": False, "phase": "queued"}
    # The command manager can have a shorter configured deadline. Return the
    # partial result before its enclosing cancellation discards it.
    timeout = min(
        timeout, max(0, trio.current_effective_deadline() - trio.current_time() - 0.25)
    )
    with trio.move_on_after(timeout) as deadline:
        async with uav._operator_lock, uav.driver._operator_limiter:
            if not uav.is_connected:
                raise ValueError("flight controller is disconnected")
            if operation == "telemetry":
                await telemetry_snapshot(uav, names, result)
            elif operation in ("read", "compare", "apply"):
                await parameter_operation(
                    uav,
                    cast(Literal["read", "compare", "apply"], operation),
                    names,
                    values,
                    result,
                )
            else:
                raise ValueError("unsupported operator operation")
    if deadline.cancelled_caught:
        result.update(
            complete=False,
            error="deadline reached; unverified writes have an unknown outcome"
            if result["phase"] == "applying"
            else "deadline reached",
        )
    return {
        "uav": uav.id,
        "system_id": uav.system_id,
        "elapsed_s": monotonic() - started,
        **result,
    }
