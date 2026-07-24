"""Live end-to-end check of the X-RTLS-GEO geometry sync against the
REAL tag firmware (native_sim).

Not collected by pytest (no ``test_`` prefix): it boots a real firmware
process and talks to it over a real UDP socket. Run manually:

    RTLS_FW_TAG=.../tag/tag/zephyr/zephyr.exe \\
    .venv/bin/python test/e2e/rtls_geometry_live.py

Bench isolation: the sim is booted on a NON-default management port
(``RTLS_LINK_SIM_MGMT_PORT``, default 13333 here) so a live skybrushd
on the same host — which broadcast-probes :3333 — never discovers it,
and the sim never hears the live server. The native_sim image compiles
without the phone-home module (it needs DHCP), so there is no :3343
advertisement leak either. Override with ``RTLS_E2E_MGMT_PORT`` if
13333 is taken.

Sequence:

1. boot the tag image, wait for discovery + a complete param snapshot;
2. inject a synthetic *reference* tag (cache-only device) carrying a
   full cell geometry that differs from the image's persisted one;
3. ``check`` -> the real tag must report inconsistent;
4. ``sync`` (reboot off: native_sim has no SMP reset surface) -> every
   write must be acked by the real firmware;
5. ``check`` -> consistent;
6. drop the server-side param cache and re-list from the device ->
   ``check`` must STILL be consistent, proving the values live on the
   device rather than in the server's optimism;
7. restart the firmware process on the same flash file -> re-list ->
   ``check`` must still be consistent, proving the geometry survives
   the reboot that ``sync`` performs on hardware.
"""

from __future__ import annotations

import logging
import os
import subprocess
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import trio

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from rtlslink import encode_param_value, param_type_from_name  # noqa: E402

from flockwave.server.ext.rtls.extension import RtlsExtension  # noqa: E402
from flockwave.server.ext.rtls.geometry import (  # noqa: E402
    run_adopt,
    run_check,
    run_sync,
)

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger("e2e")

MGMT_PORT = int(os.environ.get("RTLS_E2E_MGMT_PORT", "13333"))
REFERENCE_SYSID = 251

#: the reference geometry pushed onto the real tag: values deliberately
#: unlike any firmware default, all within the registry's bounds
REFERENCE_GEOMETRY = [
    ("UWB_ROLE", 1, "uint8"),
    ("ORIGIN_LAT_E7", 413900000, "int32"),
    ("ORIGIN_LON_E7", 21500000, "int32"),
    ("ORIGIN_ALT_MM", 12000, "int32"),
    ("POS_YAW_DEG", 30.0, "real32"),
    ("UWB_AN_COUNT", 4, "uint8"),
    ("UWB_AN0_X", -10.0, "real32"),
    ("UWB_AN0_Y", -8.0, "real32"),
    ("UWB_AN0_Z", 0.0, "real32"),
    ("UWB_AN0_MAC", 1, "uint16"),
    ("UWB_AN0_BIAS_M", 0.05, "real32"),
    ("UWB_AN1_X", 10.0, "real32"),
    ("UWB_AN1_Y", -8.0, "real32"),
    ("UWB_AN1_Z", 0.0, "real32"),
    ("UWB_AN1_MAC", 2, "uint16"),
    ("UWB_AN1_BIAS_M", 0.0, "real32"),
    ("UWB_AN2_X", 10.0, "real32"),
    ("UWB_AN2_Y", 8.0, "real32"),
    ("UWB_AN2_Z", -2.5, "real32"),
    ("UWB_AN2_MAC", 3, "uint16"),
    ("UWB_AN2_BIAS_M", 0.0, "real32"),
    ("UWB_AN3_X", -10.0, "real32"),
    ("UWB_AN3_Y", 8.0, "real32"),
    ("UWB_AN3_Z", -2.5, "real32"),
    ("UWB_AN3_MAC", 4, "uint16"),
    ("UWB_AN3_BIAS_M", 0.0, "real32"),
]


class _StubHub:
    def __init__(self):
        self.broadcasts = []

    @contextmanager
    def use_message_handlers(self, handlers):
        yield

    def create_notification(self, body=None):
        return body

    async def broadcast_message(self, message):
        self.broadcasts.append(message)

    def enqueue_broadcast_message(self, message):
        self.broadcasts.append(message)


class _StubApp:
    def __init__(self):
        self.message_hub = _StubHub()

    def import_api(self, name):
        raise KeyError(name)  # no sibling extensions in this harness


def _boot(binary: str, flash_dir: str, name: str) -> subprocess.Popen:
    env = os.environ.copy()
    env["RTLS_LINK_SIM_NO_SITL"] = "1"  # tags: skip ArduPilot TCP retries
    env["RTLS_LINK_SIM_MGMT_PORT"] = str(MGMT_PORT)  # bench isolation
    log_file = open(Path(flash_dir) / f"{name}.log", "w")
    return subprocess.Popen(
        [binary, f"-flash={flash_dir}/{name}.bin"],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )


def _make_reference(now: float) -> SimpleNamespace:
    device = SimpleNamespace(
        system_id=REFERENCE_SYSID,
        address=("127.0.0.9", MGMT_PORT),
        last_seen=now,
        params={},
        param_types={},
        param_count=len(REFERENCE_GEOMETRY),
    )
    for name, value, type_name in REFERENCE_GEOMETRY:
        param_type = param_type_from_name(type_name)
        device.params[name] = encode_param_value(value, param_type)
        device.param_types[name] = param_type
    return device


async def _wait_discovery(ext: RtlsExtension, timeout: float = 20.0) -> int:
    with trio.fail_after(timeout):
        while True:
            for system_id in ext._get_devices():
                if system_id != REFERENCE_SYSID:
                    return system_id
            await trio.sleep(0.2)


async def _wait_snapshot(ext: RtlsExtension, sysid: int, timeout: float = 20.0):
    """Waits until the device's param snapshot is count-complete (the
    discovery dump plus any refill has landed)."""
    with trio.fail_after(timeout):
        while True:
            device = ext._get_devices().get(sysid)
            if (
                device is not None
                and device.param_count is not None
                and len(device.params) >= device.param_count
            ):
                return device
            await trio.sleep(0.2)


async def _relist(ext: RtlsExtension, sysid: int) -> None:
    """Drops the server-side param cache of the device and re-fills it
    from the live device, so a subsequent check reflects device truth."""
    device = ext._get_devices()[sysid]
    device.params.clear()
    device.param_types.clear()
    device.param_count = None
    await ext.get_param_list(sysid, timeout=30.0)


async def _drive(binary: str) -> None:
    with tempfile.TemporaryDirectory(prefix="rtls-e2e-geo-") as flash_dir:
        proc = _boot(binary, flash_dir, "tag")
        try:
            ext = RtlsExtension()
            app = _StubApp()
            ext.app = app  # the extension framework sets this before run()
            async with trio.open_nursery() as nursery:

                async def run_ext():
                    await ext.run(
                        app,
                        {
                            "devices": ["127.0.0.1"],
                            "broadcast": [],
                            "port": MGMT_PORT,
                            "advertisement_port": 0,
                            "register_beacons": False,
                            "heartbeat_interval": 0.5,
                            # generous: the synthetic reference device
                            # never heartbeats and must not be expired
                            "device_timeout": 600,
                        },
                        logging.getLogger("rtls"),
                    )

                nursery.start_soon(run_ext)
                sysid = await _wait_discovery(ext)
                log.info("discovered tag sysid %d", sysid)
                await _wait_snapshot(ext, sysid)
                log.info("param snapshot complete")

                ext._require_protocol().devices[REFERENCE_SYSID] = (
                    _make_reference(time.monotonic())
                )

                # 3. adopt the synthetic reference as the canonical
                # geometry, then the factory-fresh tag must disagree
                await run_adopt(ext, reference=REFERENCE_SYSID)
                report = await run_check(ext)
                entry = report["devices"][str(sysid)]
                assert report["consistent"] is False, report
                assert entry["status"] in ("mismatch", "incomplete"), entry
                log.info("pre-sync check: %s", entry["status"])

                # 4. sync: every write must be acked by the real firmware
                result = await run_sync(ext, reboot=False)
                entry = result["devices"][str(sysid)]
                assert entry["status"] == "synced", entry
                assert not entry["failures"], entry
                assert entry["written"], entry
                assert entry["written"][-1] == "UWB_AN_COUNT", entry
                log.info(
                    "sync: wrote %d params (%d already consistent)",
                    len(entry["written"]),
                    len(entry["skipped"]),
                )

                # 5. the tag must now agree (server-cache view)
                report = await run_check(ext)
                assert report["consistent"] is True, report

                # 6. and it must STILL agree when the cache is rebuilt
                # from the live device (device truth, not server optimism)
                await _relist(ext, sysid)
                report = await run_check(ext)
                assert report["consistent"] is True, report
                log.info("post-sync check from a fresh device dump: consistent")

                # 7. the geometry must survive a firmware reboot (this is
                # what sync's SMP reset relies on with hardware)
                proc.terminate()
                proc.wait(timeout=5)
                time.sleep(1)  # let the old mgmt port close
                proc = _boot(binary, flash_dir, "tag")
                await _wait_discovery(ext)
                await _relist(ext, sysid)
                report = await run_check(ext)
                assert report["consistent"] is True, report
                log.info("post-reboot check: geometry persisted")

                nursery.cancel_scope.cancel()
        finally:
            proc.terminate()
            proc.wait(timeout=5)


def main() -> int:
    tag = os.environ.get("RTLS_FW_TAG")
    if not tag or not Path(tag).exists():
        print("set RTLS_FW_TAG to a built native_sim tag binary")
        return 2

    try:
        from rtlslink.sim import warn_if_stale
    except ImportError:
        pass
    else:
        report = warn_if_stale(tag, log)
        if report:
            print(f"WARNING: {report}", file=sys.stderr)

    trio.run(_drive, tag)
    print("E2E OK: X-RTLS-GEO check/sync verified against live tag firmware")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
