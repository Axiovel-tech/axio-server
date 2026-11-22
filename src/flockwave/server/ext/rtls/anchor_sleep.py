"""Sleep and wake of an anchor (X-RTLS-SLEEP), verified from the
``powerState`` the anchor reports after the SLEEP write.

An anchor reboots when it enters or leaves operating (8-15 s off the
network) and falls asleep in place from standby; it sends a health report
on every power-state change. Anchors read SLEEP 0 awake, 1 asleep, 2 off
(only the button starts it again) and 3 plain standby; the server writes
only 0 and 1. The firmware refuses a request by rewriting SLEEP to the state
it stays in.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink.protocol import RtlsDevice

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "ANCHOR_SLEEP_TIMEOUT",
    "lacks_param",
    "set_anchor_sleep",
    "sleep_error",
)

SLEEP_PARAM = "SLEEP"

#: default deadline of an anchor's sleep/wake transaction, in seconds; an
#: anchor that reboots is back on the network after 8-15 s
ANCHOR_SLEEP_TIMEOUT = 25.0

#: seconds after an accepted SLEEP write before it is read back: the firmware
#: queues the request on its comms lane and, when the power state machine
#: refuses it a tick later, rewrites SLEEP to the state it stays in
ANCHOR_SLEEP_SETTLE = 2.0

#: spacing of the SLEEP readbacks until one is answered, in seconds
ANCHOR_SLEEP_POLL = 1.0

#: per-request timeout of a readback, in seconds (a rebooting anchor does
#: not answer; the transaction deadline bounds the wait)
ANCHOR_READ_TIMEOUT = 0.5

#: SLEEP value while asleep (tags and anchors alike)
SLEEP_ASLEEP = 1

#: timeout of the SLEEP write's acknowledgement, in seconds
ANCHOR_WRITE_TIMEOUT = 5.0


def sleep_error(requested: bool, detail: str) -> dict[str, Any]:
    """Uniform per-device error entry for an X-RTLS-SLEEP result."""
    return {
        "requested": requested,
        "accepted": False,
        "sleeping": None,
        "detail": detail,
    }


def lacks_param(device: RtlsDevice, name: str) -> bool:
    """Whether a device's count-complete parameter snapshot proves it has
    no parameter ``name`` (an incomplete one proves nothing)."""
    return (
        device.param_count is not None
        and len(device.params) >= device.param_count
        and name not in device.params
    )


async def set_anchor_sleep(
    ext: "RtlsExtension", system_id: int, sleeping: bool, *, timeout: float
) -> dict[str, Any]:
    """Puts an anchor to sleep or wakes it, and verifies the outcome within
    ``timeout`` seconds: a stats report after the SLEEP write whose
    ``powerState`` is ``asleep`` for a sleep, ``operating`` for a wake. An
    anchor reports either only once it is in that state, so no report from
    before the transition matches. An anchor without a fresh ``powerState``
    is not commanded."""
    device = ext._require_protocol().devices.get(system_id)
    if device is None:
        raise KeyError(system_id)
    if lacks_param(device, SLEEP_PARAM):
        return sleep_error(
            sleeping, "the anchor has no sleep mode (no SLEEP parameter)"
        )
    if ext._power_state(system_id) is None:
        return sleep_error(
            sleeping,
            "the anchor does not report its power state; update its firmware",
        )
    # this request supersedes an earlier one's pin
    ext._sleep_pins.pop(system_id, None)
    acked = False
    with trio.move_on_after(timeout):
        with ext._subscribed_events() as events:
            ack = await ext.set_param(
                system_id,
                SLEEP_PARAM,
                int(sleeping),
                "uint8",
                timeout=min(timeout, ANCHOR_WRITE_TIMEOUT),
            )
            if not ack["accepted"]:
                return sleep_error(
                    sleeping, f"parameter write rejected (code {ack['result']})"
                )
            acked = True
            readback = await _await_outcome(ext, system_id, sleeping, events)
        if readback is not None:
            return {
                "requested": sleeping,
                "accepted": False,
                "sleeping": readback == SLEEP_ASLEEP,
                "detail": _refusal(ext, system_id, sleeping, readback),
            }
        ext._mark_sleeping(system_id, sleeping)
        await ext._on_inf_change(time.monotonic())
        return {
            "requested": sleeping,
            "accepted": True,
            "sleeping": sleeping,
            "detail": "asleep" if sleeping else "awake",
        }
    if not acked:
        raise trio.TooSlowError
    return sleep_error(
        sleeping,
        f"the anchor accepted the request but was not seen "
        f"{'asleep' if sleeping else 'awake'} within {timeout:g} s "
        "(it may still be rebooting)",
    )


async def _await_outcome(
    ext: "RtlsExtension",
    system_id: int,
    sleeping: bool,
    events: trio.MemoryReceiveChannel,
) -> Optional[int]:
    """Follows the anchor's reports after an accepted SLEEP write (no
    deadline of its own) until one shows the target ``powerState`` (returns
    None) or SLEEP reads back another value (returns it: a refusal). SLEEP is
    read every ANCHOR_SLEEP_POLL seconds, starting ANCHOR_SLEEP_SETTLE
    seconds after the write, until one read is answered."""
    target = "asleep" if sleeping else "operating"
    read_at = trio.current_time() + ANCHOR_SLEEP_SETTLE
    while True:
        with trio.move_on_at(read_at):
            async for event in events:
                if (
                    event.kind == "stats"
                    and event.system_id == system_id
                    and ext._power_state(system_id) == target
                ):
                    return None
        read_at = trio.current_time() + ANCHOR_SLEEP_POLL
        value = await _read_sleep(ext, system_id)
        if value is not None:
            if value != int(sleeping):
                return value
            read_at = math.inf


async def _read_sleep(ext: "RtlsExtension", system_id: int) -> Optional[int]:
    """SLEEP as the anchor reads it back, or None when it does not answer
    within ANCHOR_READ_TIMEOUT (offline or rebooting)."""
    try:
        result = await ext.get_param(
            system_id, SLEEP_PARAM, timeout=ANCHOR_READ_TIMEOUT
        )
    except (KeyError, trio.TooSlowError):
        return None
    return int(result["value"])


def _refusal(
    ext: "RtlsExtension", system_id: int, sleeping: bool, readback: int
) -> str:
    """Why an anchor refused a sleep/wake request, from the state its SLEEP
    reads back and its last stats."""
    stats = ext._stats.get(system_id, {})
    if readback == 2:
        return "refused: the anchor is off; only its button starts it again"
    if sleeping:
        state = stats.get("powerState")
        return "sleep refused by the anchor" + (
            f" in its {state} state" if state else ""
        )
    voltage = stats.get("batteryVoltage")
    return (
        "wake refused by the anchor: its pack is low or has no fresh voltage reading"
        + (f" (last {voltage:.2f} V)" if voltage is not None else "")
    )
