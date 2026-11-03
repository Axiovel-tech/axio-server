"""Heartbeat and parameter access for the collection guards."""

from __future__ import annotations

import trio

from .transport import FCTransport, MAVLinkMessage

__all__ = ("ARMED", "heartbeat", "read_parameter", "write_parameter")

ARMED = 128
"""MAV_MODE_FLAG_SAFETY_ARMED in HEARTBEAT.base_mode."""


async def heartbeat(transport: FCTransport, timeout: float = 5) -> MAVLinkMessage:
    with transport.subscribe("HEARTBEAT") as beats:
        with trio.move_on_after(timeout):
            return await beats.receive()
    heard = getattr(transport, "heard_systems", None)
    raise trio.TooSlowError(
        f"no heartbeat from the autopilot of system {transport.system_id}"
        + (f"; heard systems {sorted(heard)}" if heard else "")
    )


async def _exchange(
    transport: FCTransport, name: str, request: str, fields: dict, attempts: int
) -> tuple[float, int]:
    with transport.subscribe("PARAM_VALUE") as values:
        for _ in range(attempts):
            await transport.send(request, param_id=name.encode(), **fields)
            with trio.move_on_after(1.5):
                async for message in values:
                    if message.param_id == name:
                        return message.param_value, message.param_type
    raise trio.TooSlowError(f"no PARAM_VALUE for {name}")


async def read_parameter(
    transport: FCTransport, name: str, attempts: int = 4
) -> tuple[float, int]:
    """Fresh value and wire type of one parameter."""
    return await _exchange(
        transport, name, "PARAM_REQUEST_READ", {"param_index": -1}, attempts
    )


async def write_parameter(
    transport: FCTransport, name: str, value: float, param_type: int
) -> None:
    """Sets a parameter and requires the flight controller to echo the value."""
    actual, _ = await _exchange(
        transport,
        name,
        "PARAM_SET",
        {"param_value": value, "param_type": param_type},
        attempts=4,
    )
    if actual != value:
        actual, _ = await read_parameter(transport, name)
    if actual != value:
        raise RuntimeError(f"{name} reads back {actual:g}, not {value:g}")
