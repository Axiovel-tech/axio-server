"""Sleep and wake of an anchor (X-RTLS-SLEEP): a SLEEP write whose
acknowledgement is the answer.

The anchor refuses a request it would not act on now (PARAM_ACK_FAILED) and
otherwise takes it as a request; its ``powerState`` then shows the
transition (rtls-link-zephyr docs/reference/anchor-power-path.md). The server
neither waits for the transition nor reads SLEEP back.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from rtlslink.protocol import RtlsDevice

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "ANCHOR_WRITE_TIMEOUT",
    "lacks_param",
    "set_anchor_sleep",
    "sleep_error",
)

SLEEP_PARAM = "SLEEP"

#: timeout of the SLEEP write's acknowledgement, in seconds
ANCHOR_WRITE_TIMEOUT = 5.0

#: MAV_PARAM_EXT_ACK FAILED: how the firmware refuses a SLEEP write
PARAM_ACK_FAILED = 2


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
    """Writes an anchor's SLEEP and returns the answer of its
    acknowledgement. ``sleeping`` is None: the anchor's ``powerState`` says
    when it gets there.

    Raises KeyError for an unknown device and trio.TooSlowError when the
    acknowledgement does not arrive within ``timeout`` seconds."""
    device = ext._require_protocol().devices.get(system_id)
    if device is None:
        raise KeyError(system_id)
    if lacks_param(device, SLEEP_PARAM):
        return sleep_error(
            sleeping, "the anchor has no sleep mode (no SLEEP parameter)"
        )
    ack = await ext.set_param(
        system_id, SLEEP_PARAM, int(sleeping), "uint8", timeout=timeout
    )
    if ack["accepted"]:
        return {
            "requested": sleeping,
            "accepted": True,
            "sleeping": None,
            "detail": "sleep accepted" if sleeping else "wake accepted",
        }
    if ack["result"] != PARAM_ACK_FAILED:
        return sleep_error(sleeping, f"parameter write rejected (code {ack['result']})")
    return sleep_error(sleeping, _refusal(ext, system_id, sleeping))


def _refusal(ext: "RtlsExtension", system_id: int, sleeping: bool) -> str:
    """Why an anchor refused a sleep/wake request, from its last stats."""
    stats = ext._stats.get(system_id, {})
    state = stats.get("powerState")
    if state == "off":
        return "refused: the anchor is off; only its button starts it again"
    if not sleeping and state in ("asleep", "standby"):
        voltage = stats.get("batteryVoltage")
        return (
            "wake refused by the anchor: its pack is low or has no fresh voltage reading"
            + (f" (last {voltage:.2f} V)" if voltage is not None else "")
        )
    action = "sleep" if sleeping else "wake"
    return f"{action} refused by the anchor" + (
        f" in its {state} state" if state else ""
    )
