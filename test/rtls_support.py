"""Shared fakes for the rtls extension tests: a scripted rtls-link device and
stand-ins for the server's message hub, app and beacon registry."""

import struct
import time
from contextlib import contextmanager

from rtlslink.protocol import (
    PARAM_TYPE_CUSTOM,
    PARAM_TYPE_INT32,
    PARAM_TYPE_REAL32,
    PARAM_TYPE_UINT8,
    RTLS_COMPONENT_ID,
    encode_param_value,
    param_type_from_name,
    raw_field_bytes,
)

DEVICE_SYSID = 42


DEVICE_ADDRESS = ("192.168.4.42", 3333)


class FakeDevice:
    """Scripted rtls-link device: parses the datagrams the server sends
    and produces the MAVLink replies a real firmware would."""

    def __init__(self, dialect, system_id: int = DEVICE_SYSID):
        self.dialect = dialect
        self.system_id = system_id
        self.address = DEVICE_ADDRESS
        self._mav = dialect.MAVLink(
            None, srcSystem=system_id, srcComponent=RTLS_COMPONENT_ID
        )
        self._parser = dialect.MAVLink(None)
        self._parser.robust_parsing = True

        # name -> (raw value bytes, MAV_PARAM_EXT_TYPE code)
        self.params: dict[str, tuple[bytes, int]] = {
            "MAV_SYS_ID": (bytes([system_id]), PARAM_TYPE_UINT8),
            "UWB_CH": (struct.pack("<i", 5), PARAM_TYPE_INT32),
            "POS_X": (struct.pack("<f", 1.5), PARAM_TYPE_REAL32),
            "FW_VERSION": (b"1.2.3", PARAM_TYPE_CUSTOM),
            "SLEEP": (bytes([0]), PARAM_TYPE_UINT8),
        }

        self.respond_to_list = True
        self.respond_to_read = True
        #: parameter names omitted from full-list responses, simulating
        #: the datagrams a real dump loses to UDP under the startup burst
        self.drop_from_list: set[str] = set()
        #: every parameter name the server asked for with a targeted
        #: PARAM_EXT_REQUEST_READ, in arrival order (recorded even while
        #: ``respond_to_read`` is off)
        self.read_requests: list[str] = []
        self.respond_to_set = True
        self.set_result = 0  # PARAM_ACK_ACCEPTED
        #: emulate the firmware's arming gate: a SLEEP=1 write is acked
        #: but flipped back to 0, as the real device does while armed
        self.refuse_sleep = False

    @property
    def sleeping(self) -> bool:
        return self.params["SLEEP"][0][:1] == b"\x01"

    def heartbeat(self) -> bytes:
        d = self.dialect
        # a sleeping device advertises STANDBY, like the firmware
        status = d.MAV_STATE_STANDBY if self.sleeping else d.MAV_STATE_ACTIVE
        message = d.MAVLink_heartbeat_message(
            type=d.MAV_TYPE_GENERIC,
            autopilot=d.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=status,
            mavlink_version=3,
        )
        return bytes(message.pack(self._mav))

    def named_value_float(
        self, name: str, value: float, time_boot_ms: int = 0
    ) -> bytes:
        """One NAMED_VALUE_FLOAT, as the firmware emits per health stat."""
        message = self.dialect.MAVLink_named_value_float_message(
            time_boot_ms=time_boot_ms,
            name=name.encode(),
            value=value,
        )
        return bytes(message.pack(self._mav))

    def handle(self, data: bytes) -> list[bytes]:
        out: list[bytes] = []
        for message in self._parser.parse_buffer(data) or []:
            kind = message.get_type()
            if kind == "PARAM_EXT_REQUEST_LIST" and self.respond_to_list:
                for index, name in enumerate(self.params):
                    if name in self.drop_from_list:
                        continue
                    out.append(self._param_value(name, index))
            elif kind == "PARAM_EXT_REQUEST_READ":
                name = message.param_id
                self.read_requests.append(name)
                if self.respond_to_read and name in self.params:
                    index = list(self.params).index(name)
                    out.append(self._param_value(name, index))
            elif kind == "PARAM_EXT_SET" and self.respond_to_set:
                name = message.param_id
                value = raw_field_bytes(message, "param_value") or b""
                previous = self.params.get(name, (b"", 0))[0]
                if self.set_result == 0 and name in self.params:
                    width = len(self.params[name][0])
                    self.params[name] = (value[:width], message.param_type)
                out.append(
                    self._param_ack(name, value, message.param_type, self.set_result)
                )
                if name == "SLEEP" and self.set_result == 0:
                    self.after_sleep_write(previous[:1], value[:1])
        return out

    def after_sleep_write(self, previous: bytes, value: bytes) -> None:
        """What the firmware does after acking a SLEEP write."""
        if self.refuse_sleep and self.sleeping:
            # the arming gate runs after the ack on the firmware
            self.params["SLEEP"] = (bytes([0]), PARAM_TYPE_UINT8)

    def _param_value(self, name: str, index: int) -> bytes:
        value, param_type = self.params[name]
        message = self.dialect.MAVLink_param_ext_value_message(
            param_id=name.encode(),
            param_value=value,
            param_type=param_type,
            param_count=len(self.params),
            param_index=index,
        )
        return bytes(message.pack(self._mav))

    def _param_ack(
        self, name: str, value: bytes, param_type: int, result: int
    ) -> bytes:
        message = self.dialect.MAVLink_param_ext_ack_message(
            param_id=name.encode(),
            param_value=value,
            param_type=param_type,
            param_result=result,
        )
        return bytes(message.pack(self._mav))


class StubMessageHub:
    """Captures the notifications that the extension broadcasts."""

    def __init__(self):
        self.broadcasts = []
        #: when set, enqueue_broadcast_message emulates a full TX queue by
        #: dropping the notification, like the real hub does on overflow
        self.full = False

    def create_notification(self, body=None):
        return body

    async def broadcast_message(self, message):
        self.broadcasts.append(message)

    def enqueue_broadcast_message(self, message):
        # the real hub's send_nowait drops the message when the queue is
        # full instead of blocking the caller
        if not self.full:
            self.broadcasts.append(message)


class StubApp:
    def __init__(self):
        self.message_hub = StubMessageHub()


class StubBeaconBasicProperties:
    def __init__(self, name=""):
        self.name = name


class StubBeacon:
    def __init__(self, id):
        self.id = id
        self.basic_properties = StubBeaconBasicProperties()
        self.position = None
        self.active = False

    def update_status(self, position=None, heading=None, active=None):
        if position is not None:
            self.position = position
        if active is not None:
            self.active = bool(active)


class StubBeaconAPI:
    """Stands in for the beacon-registry API the extension imports: ``use``
    yields a beacon and removes it again when its context exits, mirroring the
    real registry-backed context manager."""

    def __init__(self):
        self.beacons = {}

    @contextmanager
    def use(self, beacon_id):
        beacon = StubBeacon(beacon_id)
        self.beacons[beacon_id] = beacon
        try:
            yield beacon
        finally:
            self.beacons.pop(beacon_id, None)


async def discover(extension, device):
    """Feeds a device heartbeat into the extension; the extension then
    auto-fetches the parameter list, exactly as the run loop would."""
    await extension._process_datagram(
        device.heartbeat(), device.address, time.monotonic()
    )


def make_message(builder, body):
    return builder.create_message(body)


def set_fake_param(device, name, value, param_type_name):
    """Write a parameter into a FakeDevice's wire store (raw bytes + type)."""
    param_type = param_type_from_name(param_type_name)
    device.params[name] = (encode_param_value(value, param_type), param_type)
