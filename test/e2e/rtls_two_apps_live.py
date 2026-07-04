"""Live end-to-end check of the rtls extension against the REAL
rtls-link firmware binaries (native_sim), post tag/anchor split.

Not collected by pytest (no ``test_`` prefix): it boots real firmware
processes and talks to them over a real UDP socket. Run manually:

    RTLS_FW_TAG=.../tag/tag/zephyr/zephyr.exe \\
    RTLS_FW_ANCHOR=.../anchor/anchor/zephyr/zephyr.exe \\
    .venv/bin/python test/e2e/rtls_two_apps_live.py

Sequence (each image booted un-orchestrated on the default mgmt port
3333, so run it on a host where :3333 is free):

1. anchor image: discovery -> UWB_ROLE == 3 (responder default);
   UWB_ROLE=2 accepted (initiator, in-bounds); UWB_ROLE=1 (species
   flip) -> accepted=False + VALUE_UNSUPPORTED + clamped value;
   verify_species("tag") raises, verify_species("anchor") passes.
2. tag image: discovery -> UWB_ROLE == 1 (image-pinned);
   UWB_ROLE=2 -> accepted=False; health stats (rate/solvepct/...)
   arrive as X-RTLS-STATS broadcasts.
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

import trio

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from flockwave.server.ext.rtls.extension import RtlsExtension  # noqa: E402
from rtlslink.protocol import (  # noqa: E402
    PARAM_ACK_ACCEPTED,
    PARAM_ACK_VALUE_UNSUPPORTED,
)

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger("e2e")

MGMT_PORT = 3333


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


class _StubApp:
    def __init__(self):
        self.message_hub = _StubHub()


def _boot(binary: str, flash_dir: str, name: str) -> subprocess.Popen:
    env = os.environ.copy()
    env["RTLS_LINK_SIM_NO_SITL"] = "1"  # tags: skip ArduPilot TCP retries
    log_file = open(Path(flash_dir) / f"{name}.log", "w")
    return subprocess.Popen(
        [binary, f"-flash={flash_dir}/{name}.bin"],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )


async def _wait_discovery(ext: RtlsExtension, timeout: float = 15.0) -> int:
    with trio.fail_after(timeout):
        while True:
            devices = ext._get_devices()
            if devices:
                return next(iter(devices))
            await trio.sleep(0.2)


async def _drive(binary: str, name: str, checks) -> None:
    with tempfile.TemporaryDirectory(prefix=f"rtls-e2e-{name}-") as flash_dir:
        proc = _boot(binary, flash_dir, name)
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
                            "heartbeat_interval": 0.5,
                            "device_timeout": 5,
                        },
                        logging.getLogger("rtls"),
                    )

                nursery.start_soon(run_ext)
                sysid = await _wait_discovery(ext)
                log.info("%s: discovered sysid %d", name, sysid)
                await checks(ext, app, sysid)
                nursery.cancel_scope.cancel()
        finally:
            proc.terminate()
            proc.wait(timeout=5)


async def _check_anchor(ext: RtlsExtension, app: _StubApp, sysid: int) -> None:
    role = await ext.get_param(sysid, "UWB_ROLE")
    assert role["value"] == 3, f"anchor image must default to responder, got {role}"

    ok = await ext.set_param(sysid, "UWB_ROLE", 2)
    assert ok["accepted"] and ok["value"] == 2, f"initiator set must pass: {ok}"

    clamped = await ext.set_param(sysid, "UWB_ROLE", 1)
    assert clamped["accepted"] is False, f"species flip must not ack accepted: {clamped}"
    assert clamped["result"] == PARAM_ACK_VALUE_UNSUPPORTED, clamped
    assert clamped["value"] == 2, f"ack must carry the clamped value: {clamped}"
    log.info("anchor: species flip correctly refused (%s)", clamped)

    try:
        await ext.verify_species(sysid, "tag")
    except ValueError as ex:
        log.info("anchor: verify_species('tag') refused: %s", ex)
    else:
        raise AssertionError("verify_species must refuse a tag image for an anchor")
    await ext.verify_species(sysid, "anchor")
    log.info("anchor: verify_species('anchor') passed")


async def _check_tag(ext: RtlsExtension, app: _StubApp, sysid: int) -> None:
    role = await ext.get_param(sysid, "UWB_ROLE")
    assert role["value"] == 1, f"tag image must be pinned to tag, got {role}"

    clamped = await ext.set_param(sysid, "UWB_ROLE", 2)
    assert clamped["accepted"] is False and clamped["value"] == 1, clamped
    log.info("tag: species flip correctly refused (%s)", clamped)

    await ext.verify_species(sysid, "tag")
    try:
        await ext.verify_species(sysid, "anchor")
    except ValueError:
        pass
    else:
        raise AssertionError("verify_species must refuse an anchor image for a tag")

    # health stats (NAMED_VALUE_FLOAT cycle) reach the broadcast path
    with trio.fail_after(15):
        while not any(
            b.get("type") == "X-RTLS-STATS" for b in app.message_hub.broadcasts
        ):
            await trio.sleep(0.2)
    stats = next(
        b for b in app.message_hub.broadcasts if b.get("type") == "X-RTLS-STATS"
    )
    body = next(iter(stats["stats"].values()))
    assert "solveRateHz" in body and "clockPpm" in body, body
    log.info("tag: stats broadcast OK (%s)", body)


def main() -> int:
    tag = os.environ.get("RTLS_FW_TAG")
    anchor = os.environ.get("RTLS_FW_ANCHOR")
    if not tag or not anchor or not Path(tag).exists() or not Path(anchor).exists():
        print("set RTLS_FW_TAG / RTLS_FW_ANCHOR to built native_sim binaries")
        return 2

    trio.run(_drive, anchor, "anchor", _check_anchor)
    time.sleep(1)  # let the old mgmt port close
    trio.run(_drive, tag, "tag", _check_tag)
    print("E2E OK: anchor + tag images verified against the live extension")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
