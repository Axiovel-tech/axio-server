"""Optional live bridge test against rtls-link native_sim.

Set RTLS_FW to the firmware native_sim ``zephyr.exe`` to enable this test.
It verifies the server-side anchor beacon bridge over real UDP/PARAM_EXT.
"""

from __future__ import annotations

import logging
import os
import socket
import subprocess
import time
from contextlib import contextmanager

import pytest
import trio
from rtlslink import RtlsClient

from flockwave.server.ext.rtls.extension import RtlsExtension

pytestmark = pytest.mark.trio

RTLS_PORT = 3333
CELL_WRITES = [
    ("ORIGIN_LAT_E7", 413900000, "int32"),
    ("ORIGIN_LON_E7", 21500000, "int32"),
    ("ORIGIN_ALT_MM", 10000, "int32"),
    ("UWB_AN0_X", -10.0, "real32"),
    ("UWB_AN0_Y", -10.0, "real32"),
    ("UWB_AN0_Z", 0.0, "real32"),
    ("UWB_AN1_X", 10.0, "real32"),
    ("UWB_AN1_Y", 10.0, "real32"),
    ("UWB_AN1_Z", -4.8, "real32"),
    ("UWB_AN_COUNT", 2, "uint8"),
]


class StubMessageHub:
    def __init__(self):
        self.broadcasts = []
        self.handlers = {}

    @contextmanager
    def use_message_handlers(self, handlers):
        self.handlers.update(handlers)
        try:
            yield
        finally:
            self.handlers.clear()

    def create_notification(self, body=None):
        return body

    async def broadcast_message(self, message):
        self.broadcasts.append(message)


class StubBeaconBasicProperties:
    def __init__(self):
        self.name = ""


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


class StubApp:
    def __init__(self):
        self.message_hub = StubMessageHub()
        self.beacon_api = StubBeaconAPI()

    def import_api(self, name):
        if name == "beacon":
            return self.beacon_api
        raise KeyError(name)

    def handle_registry_full_error(self, owner, description):
        raise RuntimeError(description)


async def test_native_sim_anchor_beacon_bridge(tmp_path):
    fw_bin = os.environ.get("RTLS_FW")
    if not fw_bin or not os.path.exists(fw_bin):
        pytest.skip("set RTLS_FW to a native_sim zephyr.exe to run this test")
    if not _udp_port_available(RTLS_PORT):
        pytest.skip(f"UDP :{RTLS_PORT} is already in use")

    log = tmp_path / "native_sim.log"
    log_file = log.open("w")
    firmware = subprocess.Popen(
        [fw_bin, f"-flash={tmp_path}/flash.bin"],
        cwd=tmp_path,
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )
    try:
        system_id = await _configure_native_sim_cell()

        app = StubApp()
        extension = RtlsExtension()
        extension.app = app
        logger = logging.getLogger("test.rtls.native_sim")

        async with trio.open_nursery() as nursery:
            nursery.start_soon(
                extension.run,
                app,
                {"devices": [f"127.0.0.1:{RTLS_PORT}"], "broadcast": []},
                logger,
            )

            await _wait_for(lambda: "rtls::default::anchor_1" in app.beacon_api.beacons)
            anchor1 = app.beacon_api.beacons["rtls::default::anchor_1"]
            assert anchor1.active is False
            assert anchor1.position.lat == pytest.approx(41.3900898)
            assert anchor1.position.lon == pytest.approx(2.1501197)
            assert anchor1.position.amsl == pytest.approx(14.8)

            result = await extension.set_param(system_id, "UWB_AN1_X", 20.0, "real32")

            assert result["accepted"] is True
            await _wait_for(
                lambda: (
                    anchor1.position is not None
                    and anchor1.position.lat == pytest.approx(41.3901797)
                )
            )
            assert anchor1.position.lon == pytest.approx(2.1501197)
            assert anchor1.position.amsl == pytest.approx(14.8)
            nursery.cancel_scope.cancel()
    finally:
        firmware.terminate()
        try:
            firmware.wait(timeout=5)
        except subprocess.TimeoutExpired:
            firmware.kill()
        log_file.close()


async def _configure_native_sim_cell() -> int:
    async with RtlsClient(
        targets=[("127.0.0.1", RTLS_PORT)],
        broadcast=(),
        heartbeat_interval=0.2,
        device_timeout=5.0,
    ) as client:
        devices = await client.discover(timeout=5.0)
        assert devices, "native_sim RTLS device was not discovered"
        client.protocol._next_heartbeat = time.monotonic() + 30.0
        device = devices[0]
        for name, value, param_type in CELL_WRITES:
            result = await client.param_set(
                device, name, value, param_type, timeout=3.0
            )
            assert result["accepted"], f"{name} was rejected: {result}"
        return device.system_id


async def _wait_for(predicate, timeout=10.0) -> None:
    with trio.fail_after(timeout):
        while not predicate():
            await trio.sleep(0.1)


def _udp_port_available(port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()
