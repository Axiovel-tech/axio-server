"""Sleep and wake of an anchor (X-RTLS-SLEEP), verified from what the anchor
reports after the SLEEP write: its ``powerState``, or, on firmware without
``pwrst``, its heartbeat state and SLEEP readback.

An anchor reboots when it enters or leaves operating (8-15 s off the
network) and falls asleep in place from standby. Anchors read SLEEP 0 awake,
1 asleep, 2 off (only the button starts it again) and 3 plain standby; the
server writes only 0 and 1.
"""

from __future__ import annotations

import dataclasses
import time
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink.client import UptimeSample
from rtlslink.protocol import ProtocolEvent, RtlsDevice

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "ANCHOR_SLEEP_TIMEOUT",
    "MAV_STATE_ACTIVE",
    "MAV_STATE_STANDBY",
    "lacks_param",
    "set_anchor_sleep",
    "sleep_error",
)

#: heartbeat system_status values (MAV_STATE_*): STANDBY in any standby (asleep,
#: plain standby, off), ACTIVE while awake; safe mode reports EMERGENCY
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4

SLEEP_PARAM = "SLEEP"

#: default deadline of an anchor's sleep/wake transaction, in seconds; an
#: anchor that reboots is back on the network after 8-15 s
ANCHOR_SLEEP_TIMEOUT = 25.0

#: seconds after an accepted SLEEP write before it is read back: the firmware
#: queues the request on its comms lane and, when the power state machine
#: refuses it a tick later, rewrites SLEEP to the state it stays in
ANCHOR_SLEEP_SETTLE = 2.0

#: spacing of the SLEEP readbacks and uptime probes while a transition is
#: awaited, in seconds
ANCHOR_SLEEP_POLL = 1.0

#: per-request timeout of those readbacks and probes, in seconds (a rebooting
#: anchor does not answer; the transaction deadline bounds the wait)
ANCHOR_PROBE_TIMEOUT = 0.5

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


@dataclasses.dataclass
class AnchorTransition:
    """What an anchor reported since a sleep/wake request was sent to it."""

    system_id: int
    #: the requested state: True asleep, False awake
    expected: bool
    #: whether the anchor already reported the target before the request
    already: bool
    #: the uptime sampled before the request, None when unknown
    before: Optional[UptimeSample] = None
    #: system_status of the latest heartbeat since the request
    status: Optional[int] = None
    #: whether the anchor rebooted since the request
    rebooted: bool = False
    #: the latest SLEEP value read back since the request, and whether that
    #: read came after the reboot
    readback: Optional[int] = None
    readback_after_reboot: bool = False
    #: whether the outcome is the reported ``powerState`` (firmware with
    #: ``pwrst``) rather than the heartbeat state
    by_power_state: bool = False
    #: the ``powerState`` of the latest stats report since the request
    power_state: Optional[str] = None
    #: whether the transition takes a reboot (into or out of operating)
    needs_reboot: bool = True

    @property
    def target_power_state(self) -> str:
        return "asleep" if self.expected else "operating"

    @property
    def target_status(self) -> int:
        return MAV_STATE_STANDBY if self.expected else MAV_STATE_ACTIVE

    def follow_power_state(self, initial: str) -> None:
        """Judge the outcome by ``powerState``, starting from ``initial``."""
        self.by_power_state = True
        target = self.target_power_state
        self.already = initial == target
        self.needs_reboot = not self.already and "operating" in (initial, target)

    def observe(
        self, event: ProtocolEvent, status: Optional[int], power_state: Optional[str]
    ) -> None:
        """One event, with the anchor's system_status as the SDK holds it
        (the event says only STANDBY or not; safe mode is neither) and its
        ``powerState`` as the extension decoded it from the stats."""
        if event.system_id != self.system_id:
            return
        if event.kind == "stats" and power_state is not None:
            self.power_state = power_state
        elif event.kind in ("heartbeat", "discovered"):
            self.status = status
            # rediscovery: the anchor was off the network past its timeout
            self.rebooted = self.rebooted or event.kind == "discovered"

    def observe_uptime(self, sample: Optional[UptimeSample]) -> None:
        """A reboot is an uptime that advanced less than the host time
        elapsed since the sample before the request (the SDK's rule): a
        lower uptime alone misses a second reboot right after another."""
        if (
            sample is not None
            and self.before is not None
            and sample.follows_reboot(self.before)
        ):
            self.rebooted = True

    @property
    def reached(self) -> bool:
        """Whether the latest report since the request shows the target:
        the power state, else an explicit STANDBY or ACTIVE heartbeat."""
        if self.by_power_state:
            return self.power_state == self.target_power_state
        return self.status == self.target_status

    @property
    def awaits_reboot(self) -> bool:
        """The target is reported, but the reboot it needs is not proven."""
        return self.reached and self.needs_reboot and not self.rebooted

    @property
    def wants_readback(self) -> bool:
        """A SLEEP readback is due: once before the reboot (a refusal reads
        back the state the anchor stays in), and, without ``powerState``,
        once after a sleep's reboot (STANDBY also covers off)."""
        if not self.rebooted:
            return self.readback is None
        return (
            not self.by_power_state
            and self.expected
            and self.reached
            and not self.readback_after_reboot
        )

    def outcome(self) -> Optional[str]:
        """``"confirmed"``, ``"unverified"`` (an asleep anchor cannot be told
        from plain standby without ``powerState``) or None while pending."""
        if not self.reached:
            return None
        if self.by_power_state:
            return "confirmed" if self.rebooted or not self.needs_reboot else None
        if self.rebooted:
            if not self.expected:
                return "confirmed"
            asleep = self.readback_after_reboot and self.readback == SLEEP_ASLEEP
            return "confirmed" if asleep else None
        if self.already and self.readback == int(self.expected):
            return "unverified" if self.expected else "confirmed"
        return None


async def set_anchor_sleep(
    ext: "RtlsExtension", system_id: int, sleeping: bool, *, timeout: float
) -> dict[str, Any]:
    """Puts an anchor to sleep or wakes it, and verifies the outcome within
    ``timeout`` seconds.

    The outcome is the anchor's ``powerState``: ``asleep`` for a sleep,
    ``operating`` for a wake; the reboot of a transition into or out of
    operating (a rediscovery or a rewound uptime) is waited for. Firmware
    without ``pwrst`` is judged by an explicit STANDBY or ACTIVE heartbeat
    after that reboot, a sleep also by SLEEP reading back 1; there a sleep
    without a reboot is reported unverified. A refused request causes no
    reboot and reads back as the state the anchor stays in."""
    device = ext._require_protocol().devices.get(system_id)
    if device is None:
        raise KeyError(system_id)
    if lacks_param(device, SLEEP_PARAM):
        return sleep_error(
            sleeping, "the anchor has no sleep mode (no SLEEP parameter)"
        )
    # this request supersedes an earlier one's pin, which would mask the
    # heartbeats it is judged by
    ext._sleep_pins.pop(system_id, None)
    watch = AnchorTransition(
        system_id,
        sleeping,
        already=device.system_status
        == (MAV_STATE_STANDBY if sleeping else MAV_STATE_ACTIVE),
    )
    power_state = ext._power_state(system_id)
    if power_state is not None:
        watch.follow_power_state(power_state)
    acked = False
    with trio.move_on_after(timeout):
        with ext._subscribed_events() as events:
            watch.before = await probe_uptime(ext, system_id)
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
            result = await _await_transition(ext, watch, events)
        if result["accepted"] and result["sleeping"] is not None:
            ext._mark_sleeping(system_id, sleeping)
            await ext._on_inf_change(time.monotonic())
        return result
    if not acked:
        raise trio.TooSlowError
    return sleep_error(
        sleeping,
        f"the anchor accepted the request but was not seen "
        f"{'asleep' if sleeping else 'awake'} within {timeout:g} s "
        "(it may still be rebooting)",
    )


async def _await_transition(
    ext: "RtlsExtension",
    watch: AnchorTransition,
    events: trio.MemoryReceiveChannel,
) -> dict[str, Any]:
    """Follows the anchor's events after an accepted SLEEP write until the
    outcome is known or the anchor refuses (no deadline of its own). The
    readbacks and uptime probes run every ANCHOR_SLEEP_POLL seconds,
    starting ANCHOR_SLEEP_SETTLE seconds after the write."""
    next_poll = trio.current_time() + ANCHOR_SLEEP_SETTLE
    while (outcome := watch.outcome()) is None:
        if trio.current_time() >= next_poll:
            next_poll = trio.current_time() + ANCHOR_SLEEP_POLL
            refusal = await _poll(ext, watch)
            if refusal is not None:
                return refusal
        with trio.move_on_after(ANCHOR_PROBE_TIMEOUT):
            event = await events.receive()
            device = ext._require_protocol().devices.get(watch.system_id)
            watch.observe(
                event,
                device.system_status if device else None,
                ext._power_state(watch.system_id),
            )
    if outcome == "unverified":
        return {
            "requested": True,
            "accepted": True,
            "sleeping": None,
            "detail": "sleep accepted but unverified: this anchor's firmware "
            "does not report its power state, and asleep reads the same as "
            "standby",
        }
    return {
        "requested": watch.expected,
        "accepted": True,
        "sleeping": watch.expected,
        "detail": "asleep" if watch.expected else "awake",
    }


async def _poll(
    ext: "RtlsExtension", watch: AnchorTransition
) -> Optional[dict[str, Any]]:
    """One readback/probe round of :func:`_await_transition`; returns the
    refusal result when SLEEP reads back another state."""
    system_id = watch.system_id
    if watch.wants_readback:
        value = await read_param_or_none(ext, system_id, SLEEP_PARAM)
        if value is not None:
            watch.readback = int(value)
            watch.readback_after_reboot = watch.rebooted
            if watch.readback != int(watch.expected):
                return {
                    "requested": watch.expected,
                    "accepted": False,
                    "sleeping": watch.readback == SLEEP_ASLEEP,
                    "detail": _refusal(ext, watch),
                }
    if watch.awaits_reboot:
        watch.observe_uptime(await probe_uptime(ext, system_id))
    return None


def _refusal(ext: "RtlsExtension", watch: AnchorTransition) -> str:
    """Why an anchor refused a sleep/wake request, from the state its SLEEP
    reads back and its last stats."""
    stats = ext._stats.get(watch.system_id, {})
    if watch.readback == 2:
        return "refused: the anchor is off; only its button starts it again"
    if watch.expected:
        state = stats.get("powerState") or ("standby" if watch.readback == 3 else None)
        return "sleep refused by the anchor" + (
            f" in its {state} state" if state else ""
        )
    voltage = stats.get("batteryVoltage")
    return (
        "wake refused by the anchor: its pack is low or has no fresh voltage reading"
        + (f" (last {voltage:.2f} V)" if voltage is not None else "")
    )


async def read_param_or_none(
    ext: "RtlsExtension",
    system_id: int,
    name: str,
    timeout: float = ANCHOR_PROBE_TIMEOUT,
) -> Any:
    """One parameter's value, or None when the device does not answer
    within ``timeout`` (offline or rebooting)."""
    try:
        result = await ext.get_param(system_id, name, timeout=timeout)
    except (KeyError, trio.TooSlowError):
        return None
    return result["value"]


async def probe_uptime(ext: "RtlsExtension", system_id: int) -> Optional[UptimeSample]:
    """The device's uptime from a correlated TIMESYNC request, bounded by the
    host time of the request and of the reply, or None when it does not
    answer within ANCHOR_PROBE_TIMEOUT."""
    protocol = ext._require_protocol()
    token = time.monotonic_ns()
    with ext._subscribed_events() as events:
        request = protocol.request_uptime(system_id, token)
        if request is None:
            return None
        requested_at = trio.current_time()
        await ext._send(*request)
        with trio.move_on_after(ANCHOR_PROBE_TIMEOUT):
            while True:
                event = await events.receive()
                if (
                    event.kind == "timesync"
                    and event.system_id == system_id
                    and event.data.get("token") == token
                ):
                    return UptimeSample(
                        int(event.data["uptime_ms"]), requested_at, trio.current_time()
                    )
    return None
