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

import struct
from dataclasses import dataclass, field
from typing import Any, Optional, Union

__all__ = (
    "RtlsProtocol",
    "RtlsDevice",
    "ProtocolEvent",
    "decode_param_value",
    "encode_param_value",
    "param_type_from_name",
    "param_type_to_name",
    "raw_field_bytes",
)

#: rtls-link identity convention (matches the firmware defaults)
RTLS_COMPONENT_ID = 197

#: PARAM_EXT type codes (MAV_PARAM_EXT_TYPE_*)
PARAM_TYPE_UINT8 = 1
PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL32 = 9
PARAM_TYPE_CUSTOM = 11

#: PARAM_EXT_ACK result codes (PARAM_ACK_*)
PARAM_ACK_ACCEPTED = 0
PARAM_ACK_VALUE_UNSUPPORTED = 1
PARAM_ACK_FAILED = 2
PARAM_ACK_IN_PROGRESS = 3

#: struct formats for the numeric MAV_PARAM_EXT_TYPE codes; values are
#: encoded byte-wise (little endian) in the first bytes of the value field
_PARAM_TYPE_FORMATS: dict[int, str] = {
    1: "<B",
    2: "<b",
    3: "<H",
    4: "<h",
    5: "<I",
    6: "<i",
    7: "<Q",
    8: "<q",
    9: "<f",
    10: "<d",
}

_PARAM_TYPE_NAMES: dict[int, str] = {
    1: "uint8",
    2: "int8",
    3: "uint16",
    4: "int16",
    5: "uint32",
    6: "int32",
    7: "uint64",
    8: "int64",
    9: "real32",
    10: "real64",
    11: "custom",
}

_PARAM_NAMES_TO_TYPE = {name: code for code, name in _PARAM_TYPE_NAMES.items()}


def param_type_to_name(param_type: int) -> str:
    """Returns the symbolic name of a MAV_PARAM_EXT_TYPE code (e.g.
    ``"uint8"``); unknown codes are returned as their decimal string."""
    return _PARAM_TYPE_NAMES.get(param_type, str(param_type))


def param_type_from_name(value: Union[int, str]) -> int:
    """Inverse of :func:`param_type_to_name`; also accepts the numeric
    code itself (as int or decimal string)."""
    if isinstance(value, int):
        return value
    name = str(value).strip().lower()
    if name in _PARAM_NAMES_TO_TYPE:
        return _PARAM_NAMES_TO_TYPE[name]
    try:
        return int(name)
    except ValueError:
        raise ValueError(f"unknown parameter type: {value!r}") from None


def decode_param_value(data: bytes, param_type: int) -> Union[int, float, str]:
    """Decodes a raw PARAM_EXT value field into a plain Python (and thus
    JSON-serializable) value.

    Numeric types are little-endian in the first bytes of the field; the
    transport may strip trailing NUL bytes, so the buffer is zero-padded
    back to the width of the type. Custom values are decoded as UTF-8 up
    to the first NUL."""
    fmt = _PARAM_TYPE_FORMATS.get(param_type)
    if fmt is not None:
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, data[:size].ljust(size, b"\x00"))[0]
    return data.split(b"\x00")[0].decode("utf-8", errors="replace")


def encode_param_value(value: Any, param_type: int) -> bytes:
    """Encodes a plain Python value into the raw PARAM_EXT byte encoding
    for the given MAV_PARAM_EXT_TYPE code. Inverse of
    :func:`decode_param_value`."""
    fmt = _PARAM_TYPE_FORMATS.get(param_type)
    if fmt is not None:
        if param_type in (9, 10):  # real32 / real64
            return struct.pack(fmt, float(value))
        return struct.pack(fmt, int(value))
    if isinstance(value, bytes):
        return value
    return str(value).encode("utf-8")


@dataclass
class RtlsDevice:
    system_id: int
    component_id: int
    address: tuple[str, int]
    last_seen: float = 0.0
    params: dict[str, bytes] = field(default_factory=dict)
    param_types: dict[str, int] = field(default_factory=dict)
    param_count: Optional[int] = None


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
        self._mav = mavlink_module.MAVLink(
            None, srcSystem=system_id, srcComponent=component_id
        )
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

    def request_param_list(
        self, system_id: int
    ) -> Optional[tuple[bytes, tuple[str, int]]]:
        device = self.devices.get(system_id)
        if device is None:
            return None
        message = self._mav_module.MAVLink_param_ext_request_list_message(
            target_system=system_id, target_component=device.component_id
        )
        return (self._pack(message), device.address)

    def request_param_read(
        self, system_id: int, name: str
    ) -> Optional[tuple[bytes, tuple[str, int]]]:
        device = self.devices.get(system_id)
        if device is None:
            return None
        message = self._mav_module.MAVLink_param_ext_request_read_message(
            target_system=system_id,
            target_component=device.component_id,
            param_id=name.encode(),
            param_index=-1,
        )
        return (self._pack(message), device.address)

    def set_param(
        self, system_id: int, name: str, value: bytes, param_type: int
    ) -> Optional[tuple[bytes, tuple[str, int]]]:
        device = self.devices.get(system_id)
        if device is None:
            return None
        message = self._mav_module.MAVLink_param_ext_set_message(
            target_system=system_id,
            target_component=device.component_id,
            param_id=name.encode(),
            param_value=value[:128].ljust(128, b"\x00"),
            param_type=param_type,
        )
        return (self._pack(message), device.address)

    # ---- inbound ----

    def feed(
        self, data: bytes, address: tuple[str, int], now: float
    ) -> list[ProtocolEvent]:
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
                    system_id, message.get_srcComponent(), address, now
                )
                return [ProtocolEvent("discovered", system_id, {"address": address})]
            device.last_seen = now
            device.address = address
            return []

        if kind == "PARAM_EXT_VALUE":
            device = self.devices.get(system_id)
            name = _param_id_str(message.param_id)
            value = _raw_param_value(message)
            if device is not None:
                device.params[name] = value
                device.param_types[name] = message.param_type
                if message.param_count:
                    device.param_count = message.param_count
            return [
                ProtocolEvent(
                    "param_value",
                    system_id,
                    {
                        "name": name,
                        "value": value,
                        "type": message.param_type,
                        "index": message.param_index,
                        "count": message.param_count,
                    },
                )
            ]

        if kind == "PARAM_EXT_ACK":
            return [
                ProtocolEvent(
                    "param_ack",
                    system_id,
                    {
                        "name": _param_id_str(message.param_id),
                        "value": _raw_param_value(message),
                        "result": message.param_result,
                    },
                )
            ]

        return []

    def _heartbeat_bytes(self) -> bytes:
        module = self._mav_module
        message = module.MAVLink_heartbeat_message(
            type=module.MAV_TYPE_GCS,
            autopilot=module.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=module.MAV_STATE_ACTIVE,
            mavlink_version=3,
        )
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


def _raw_payload(message) -> Optional[bytes]:
    """Returns the raw (wire) payload of a parsed pymavlink message, or
    ``None`` if the wire representation is not available."""
    buf = message.get_msgbuf()
    if not buf:
        return None
    buf = bytes(buf)
    length = buf[1]
    if buf[0] == 0xFD:  # MAVLink 2: 10-byte header
        return buf[10 : 10 + length]
    if buf[0] == 0xFE:  # MAVLink 1: 6-byte header
        return buf[6 : 6 + length]
    return None


def raw_field_bytes(message, name: str) -> Optional[bytes]:
    """Extracts a ``char[]`` field of a parsed pymavlink message as raw
    bytes, straight from the wire payload.

    pymavlink decodes ``char[]`` fields into NUL-terminated UTF-8
    strings, which destroys binary PARAM_EXT values (interior NULs
    truncate, bytes above 0x7F become U+FFFD), so the decoded attribute
    cannot be trusted for anything but ASCII. MAVLink 2 trailing-zero
    truncation is undone by zero-padding the payload to its full size."""
    cls = type(message)
    unpacker = getattr(cls, "unpacker", None)
    fields = getattr(cls, "ordered_fieldnames", None)
    if unpacker is None or fields is None:
        return None
    payload = _raw_payload(message)
    if payload is None:
        return None
    payload = payload.ljust(unpacker.size, b"\x00")[: unpacker.size]
    try:
        value = unpacker.unpack(payload)[fields.index(name)]
    except (struct.error, ValueError):
        return None
    return value if isinstance(value, bytes) else None


def _raw_param_value(message) -> bytes:
    """Returns the ``param_value`` field of a PARAM_EXT message as raw
    bytes, preferring the wire payload over pymavlink's lossy string
    decode (see :func:`raw_field_bytes`)."""
    value = raw_field_bytes(message, "param_value")
    if value is not None:
        return value
    return _param_value_bytes(message.param_value)
