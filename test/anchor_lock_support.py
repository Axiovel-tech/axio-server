"""A small anchor cell for the lock tests: a tag and scripted anchors behind
one extension, routed by address."""

import time

from rtls_support import FakeDevice, StubApp, discover, make_message, set_fake_param
from rtlslink.cell_settings import code
from rtlslink.protocol import RtlsProtocol

from flockwave.server.ext.rtls.extension import RtlsExtension


def _inf_broadcasts(extension):
    return [
        b for b in extension.app.message_hub.broadcasts if b["type"] == "X-RTLS-INF"
    ]


#: the 50 Hz session every device of the cell stores, and its cell code
SESSION = {
    "UWB_CHANNEL": (9, "uint8"),
    "UWB_SESSION_ID": (0x11223344, "uint32"),
    "UWB_BLOCK_MS": (20, "uint16"),
    "UWB_SLOT_RSTU": (1200, "uint16"),
    "UWB_SLOTS_RR": (12, "uint8"),
}
CODE_50 = code({name: value for name, (value, _) in SESSION.items()})


async def _cell(dialect, *, anchors=((201, 2, 1), (202, 3, 2)), prepare=None):
    """An extension that discovered a tag (42) and the anchors given as
    (system id, UWB_ROLE, UWB_MAC), routed by address, all storing and the
    tag running the 50 Hz session; returns it and the devices by system id.
    ``prepare`` sees the devices before discovery."""
    tag = FakeDevice(dialect)
    set_fake_param(tag, "UWB_ROLE", 1, "uint8")
    devices = {tag.system_id: tag}
    for system_id, role, mac in anchors:
        devices[system_id] = _anchor(dialect, system_id, role, mac)
    for device in devices.values():
        for name, (value, param_type) in SESSION.items():
            set_fake_param(device, name, value, param_type)
    if prepare is not None:
        prepare(devices)
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[])
    ext.app = StubApp()
    by_address = {d.address: d for d in devices.values()}

    async def fake_send(payload, address):
        device = by_address.get(address)
        for reply in device.handle(payload) if device else ():
            await ext._process_datagram(reply, device.address, time.monotonic())

    ext._send = fake_send
    ext.routes = by_address  # lets a test add a device later
    for device in devices.values():
        await discover(ext, device)
    await ext._on_stats(tag.system_id, {"cellcfg": float(CODE_50)}, time.monotonic())
    return ext, devices


def _anchor(dialect, system_id, role=3, mac=9):
    anchor = FakeDevice(dialect, system_id=system_id)
    anchor.address = (f"192.168.4.{system_id}", 3333)
    set_fake_param(anchor, "UWB_ROLE", role, "uint8")
    set_fake_param(anchor, "UWB_MAC", mac, "uint16")
    set_fake_param(anchor, "LOCK", 0, "uint8")
    set_fake_param(anchor, "UWB_TX_GAIN", 47, "uint8")
    return anchor


HEALTHY_A0 = {
    "pwrst": 5.0,
    "boots": 3.0,
    "moved": 0.0,
    "uwbst": 1.0,
    "cellcfg": float(CODE_50),
    "txgain": 47.0,
}


HEALTHY_A1 = {**HEALTHY_A0, "rnghz": 50.0}


async def _healthy(ext):
    await ext._on_stats(201, HEALTHY_A0, time.monotonic())
    await ext._on_stats(202, HEALTHY_A1, time.monotonic())


async def _lock(ext, builder, hub, action="lock", **body):
    message = make_message(builder, {"type": "X-RTLS-LOCK", "action": action} | body)
    response = await ext._handle_RTLS_LOCK(message, None, hub)
    return response.body
