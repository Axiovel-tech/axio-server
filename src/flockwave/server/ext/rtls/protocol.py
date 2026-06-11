"""Sans-IO core of the rtls-link management protocol.

Speaks MAVLink 2 on the device's management UDP channel (default :3333):
the server announces itself with GCS heartbeats (the device's UDP link
learns its peer from inbound datagrams and follows the latest source),
devices answer with their own heartbeats, and configuration runs over
the PARAM_EXT subprotocol.

No sockets, no clocks, no framework: feed datagrams in with
:meth:`feed`, collect outbound datagrams from :meth:`poll`. This makes
the protocol testable offline and verifiable against the real firmware
from any event loop (the bundled ``selfcheck.py`` drives it with plain
asyncio; the extension drives it from Trio).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

__all__ = ("RtlsProtocol", "RtlsDevice", "ProtocolEvent")

#: rtls-link identity convention (matches the firmware defaults)
RTLS_COMPONENT_ID = 197

#: PARAM_EXT type codes (MAV_PARAM_EXT_TYPE_*)
PARAM_TYPE_UINT8 = 1
PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL32 = 9
PARAM_TYPE_CUSTOM = 11


@dataclass
class RtlsDevice:
    system_id: int
    component_id: int
    address: tuple[str, int]
    last_seen: float = 0.0
    params: dict[str, bytes] = field(default_factory=dict)


@dataclass
class ProtocolEvent:
    kind: str  # "discovered" | "lost" | "param_value" | "param_ack"
    system_id: int
    data: dict[str, Any] = field(default_factory=dict)


class RtlsProtocol:
    """State machine for one management-channel conversation set."""

    def __init__(
        self,
        mavlink_module,
        *,
        targets: list[tuple[str, int]],
        broadcast_targets: Optional[list[tuple[str, int]]] = None,
        system_id: int = 254,
        component_id: int = 190,
        heartbeat_interval: float = 2.0,
        device_timeout: float = 6.0,
    ):
        self._mav_module = mavlink_module
        self._mav = mavlink_module.MAVLink(None, srcSystem=system_id,
                                           srcComponent=component_id)
        self._parser = mavlink_module.MAVLink(None)
        self._parser.robust_parsing = True
        self._targets = list(targets)
        self._broadcast_targets = list(broadcast_targets or [])
        self._heartbeat_interval = heartbeat_interval
        self._device_timeout = device_timeout
        self._next_heartbeat = 0.0
        self.devices: dict[int, RtlsDevice] = {}

    # ---- outbound ----

    def poll(self, now: float) -> list[tuple[bytes, tuple[str, int]]]:
        """Returns the datagrams to send right now, with expiry events as
        a side effect (collect them via :meth:`expire`)."""
        out: list[tuple[bytes, tuple[str, int]]] = []
        if now >= self._next_heartbeat:
            self._next_heartbeat = now + self._heartbeat_interval
            payload = self._heartbeat_bytes()
            for address in self._targets + self._broadcast_targets:
                out.append((payload, address))
            # known devices are refreshed directly even if they were
            # discovered outside the static target list
            for device in self.devices.values():
                if device.address not in self._targets:
                    out.append((payload, device.address))
        return out

    def expire(self, now: float) -> list[ProtocolEvent]:
        events = []
        for system_id in list(self.devices):
            if now - self.devices[system_id].last_seen > self._device_timeout:
                del self.devices[system_id]
                events.append(ProtocolEvent("lost", system_id))
        return events

    def request_param_list(self, system_id: int) -> Optional[tuple[bytes, tuple[str, int]]]:
        device = self.devices.get(system_id)
        if device is None:
            return None
        message = self._mav_module.MAVLink_param_ext_request_list_message(
            target_system=system_id, target_component=device.component_id)
        return (self._pack(message), device.address)

    def set_param(
        self, system_id: int, name: str, value: bytes, param_type: int
    ) -> Optional[tuple[bytes, tuple[str, int]]]:
        device = self.devices.get(system_id)
        if device is None:
            return None
        message = self._mav_module.MAVLink_param_ext_set_message(
            target_system=system_id, target_component=device.component_id,
            param_id=name.encode(), param_value=value.ljust(128, b"\x00"),
            param_type=param_type)
        return (self._pack(message), device.address)

    # ---- inbound ----

    def feed(self, data: bytes, address: tuple[str, int], now: float) -> list[ProtocolEvent]:
        events: list[ProtocolEvent] = []
        for message in self._parser.parse_buffer(data) or []:
            events.extend(self._handle(message, address, now))
        return events

    # ---- internals ----

    def _handle(self, message, address, now) -> list[ProtocolEvent]:
        kind = message.get_type()
        system_id = message.get_srcSystem()

        if kind == "HEARTBEAT":
            if message.get_srcComponent() != RTLS_COMPONENT_ID:
                return []
            device = self.devices.get(system_id)
            if device is None:
                self.devices[system_id] = RtlsDevice(
                    system_id, message.get_srcComponent(), address, now)
                return [ProtocolEvent("discovered", system_id,
                                      {"address": address})]
            device.last_seen = now
            device.address = address
            return []

        if kind == "PARAM_EXT_VALUE":
            device = self.devices.get(system_id)
            name = _param_id_str(message.param_id)
            value = _param_value_bytes(message.param_value)
            if device is not None:
                device.params[name] = value
            return [ProtocolEvent("param_value", system_id, {
                "name": name, "value": value, "type": message.param_type,
                "index": message.param_index, "count": message.param_count,
            })]

        if kind == "PARAM_EXT_ACK":
            return [ProtocolEvent("param_ack", system_id, {
                "name": _param_id_str(message.param_id),
                "value": _param_value_bytes(message.param_value),
                "result": message.param_result,
            })]

        return []

    def _heartbeat_bytes(self) -> bytes:
        module = self._mav_module
        message = module.MAVLink_heartbeat_message(
            type=module.MAV_TYPE_GCS, autopilot=module.MAV_AUTOPILOT_INVALID,
            base_mode=0, custom_mode=0, system_status=module.MAV_STATE_ACTIVE,
            mavlink_version=3)
        return self._pack(message)

    def _pack(self, message) -> bytes:
        return bytes(message.pack(self._mav))


def _param_id_str(param_id) -> str:
    if isinstance(param_id, bytes):
        return param_id.split(b"\x00")[0].decode(errors="replace")
    return str(param_id).split("\x00")[0]


def _param_value_bytes(param_value) -> bytes:
    if isinstance(param_value, bytes):
        return param_value
    if isinstance(param_value, str):
        return param_value.encode("latin-1", errors="replace")
    return bytes(param_value)
