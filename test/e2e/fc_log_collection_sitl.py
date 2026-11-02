"""End-to-end check of ``axio-fc-logs`` against two ArduPilot SITL vehicles
and a live server.

Not collected by pytest (no ``test_`` prefix). Run from the server checkout,
with the stack environment exported (``ARDUPILOT_SITL``, ``SITL_DEFAULTS``):

    heavy .venv/bin/python test/e2e/fc_log_collection_sitl.py --work /tmp/fc-logs-e2e

Each vehicle reaches the server over UDP (SERIAL0), offers a direct link on
SERIAL1 (tcp:5762 + 10 * instance) and a side channel on SERIAL2 for the
checks. Scenarios: parallel collection through the server, direct
collection, cancel and resume, crash and resume, a corrupted partial, an
armed refusal and arming mid-transfer. Results go to ``<work>/results.json``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from contextlib import ExitStack, contextmanager
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

SERVER_PORT = 25000
MAVLINK_PORT = 24550
VEHICLES = {"03": 0, "04": 1}
CLI = [sys.executable, "-m", "flockwave.server.ext.mavlink.log_collection.cli"]


def system_id(uav: str) -> int:
    return int(uav)


def port(uav: str, serial: int) -> int:
    return 5760 + 10 * VEHICLES[uav] + {1: 2, 2: 3}[serial]


class Side:
    """Synchronous checks over a vehicle's SERIAL2."""

    def __init__(self, uav: str):
        self.uav = uav
        self.link = mavutil.mavlink_connection(
            f"tcp:127.0.0.1:{port(uav, 2)}", source_system=240
        )
        self.link.wait_heartbeat(timeout=60)

    def param(self, name: str, value: float | None = None) -> float:
        while self.link.recv_match(blocking=False) is not None:
            pass
        for _ in range(10):
            if value is None:
                self.link.mav.param_request_read_send(
                    system_id(self.uav), 1, name.encode(), -1
                )
            else:
                self.link.mav.param_set_send(
                    system_id(self.uav), 1, name.encode(), value, 2
                )
            reply = self.link.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if reply and reply.param_id == name:
                if value is None or reply.param_value == value:
                    return reply.param_value
        raise TimeoutError(name)

    def arm(self, armed: bool) -> None:
        for _ in range(10):
            self.link.mav.command_long_send(
                system_id(self.uav), 1, 400, 0, int(armed), 21196, 0, 0, 0, 0, 0
            )
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                beat = self.link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if beat and beat.get_srcComponent() == 1:
                    if bool(beat.base_mode & 128) == armed:
                        return
        raise TimeoutError(f"arm={armed}")


@contextmanager
def process(args: list[str], cwd: Path, log: Path):
    with log.open("w") as output:
        child = subprocess.Popen(args, cwd=cwd, stdout=output, stderr=subprocess.STDOUT)
    try:
        yield child
    finally:
        child.terminate()
        try:
            child.wait(10)
        except subprocess.TimeoutExpired:
            child.kill()


def start_sitl(stack: ExitStack, work: Path, uav: str) -> Path:
    directory = work / f"sitl-{uav}"
    directory.mkdir(parents=True, exist_ok=True)
    args = [
        os.environ["ARDUPILOT_SITL"],
        "--model", "+",
        "--speedup", "1",
        f"-I{VEHICLES[uav]}",
        "--defaults", os.environ["SITL_DEFAULTS"],
        "--sysid", str(system_id(uav)),
        "--serial0", f"udpclient:127.0.0.1:{MAVLINK_PORT}",
    ]  # fmt: skip
    stack.enter_context(process(args, directory, directory / "sitl.out"))
    return directory


def make_logs(sides: list[Side], flights: int, armed_s: float) -> None:
    """Each flight ends a log; the last log stays open, logging disarmed.

    ArduPilot rotates on disarm only once its post-disarm logging persistence
    has expired with LOG_DISARMED off.
    """
    for side in sides:
        side.param("ARMING_CHECK", 0)
        side.param("LOG_FILE_DSRMROT", 1)
    for _ in range(flights):
        for side in sides:
            side.param("LOG_DISARMED", 1)
            side.arm(True)
        time.sleep(armed_s)
        for side in sides:
            side.arm(False)
            side.param("LOG_DISARMED", 0)
        time.sleep(20)
    for side in sides:
        side.param("LOG_DISARMED", 1)
    time.sleep(5)


def server_config(work: Path) -> Path:
    path = work / "server.jsonc"
    path.write_text(
        json.dumps(
            {
                "PORT": SERVER_PORT,
                "EXTENSIONS": {
                    "mavlink": {
                        "enabled": True,
                        "networks": {
                            "mav": {
                                "connections": [
                                    f"udp-listen://127.0.0.1:{MAVLINK_PORT}"
                                    f"?broadcast_port={MAVLINK_PORT + 5}"
                                ],
                            }
                        },
                    }
                },
            }
        )
    )
    return path


def cli(args: list[str], *, stop_when=None, stop_signal=signal.SIGINT) -> dict:
    """Runs the CLI; stop_when(status) may interrupt it. Returns events and code."""
    child = subprocess.Popen([*CLI, *args], stdout=subprocess.PIPE, text=True)
    events, interrupted = [], False
    assert child.stdout is not None
    for line in child.stdout:
        event = json.loads(line)
        events.append(event)
        if stop_when and not interrupted and event["event"] == "status":
            if stop_when(event):
                child.send_signal(stop_signal)
                interrupted = True
    return {"code": child.wait(), "events": events, "interrupted": interrupted}


def summary(run: dict) -> dict:
    return next(
        e for e in reversed(run["events"]) if e["event"] in ("summary", "interrupted")
    )


def receipt(path: str) -> dict:
    return json.loads(Path(path).read_text())


def matches_source(directory: Path, sitl: Path) -> list[str]:
    """Verified copies equal the vehicle's own log bytes (a prefix for the log
    that kept growing after LOG_DISARMED was restored)."""
    problems = []
    for item in json.loads((directory / "manifest.json").read_text())["files"]:
        local = (directory / item["file"]).read_bytes()
        if hashlib.sha256(local).hexdigest() != item["sha256"]:
            problems.append(f"{item['file']}: sha256 differs from receipt")
        if (sitl / "logs" / item["file"]).read_bytes()[: len(local)] != local:
            problems.append(f"{item['file']}: bytes differ from the SITL log")
    return problems


def largest_closed_log(directory: Path) -> str:
    """The open log keeps growing between runs, so resume tests use a closed one."""
    files = json.loads((directory / "manifest.json").read_text())["files"]
    newest = max(item["file"] for item in files)
    return max((f for f in files if f["file"] != newest), key=lambda f: f["bytes"])[
        "file"
    ]


def wait_for_receipt(directory: Path, count: int, timeout: float = 30) -> None:
    """The server finishes a cancelled run (restore, receipt) after the CLI exits."""
    deadline = time.monotonic() + timeout
    while len(list(directory.glob("run-*.json"))) < count:
        if time.monotonic() > deadline:
            raise TimeoutError(f"no run receipt in {directory}")
        time.sleep(0.5)


def halfway(event: dict) -> bool:
    return (
        event["phase"] == "transferring"
        and event["bytes_done"] > event["bytes_total"] / 3
    )


def scenarios(work: Path, sitl: dict[str, Path], side: dict[str, Side]) -> dict:
    results: dict[str, dict] = {}
    server = [
        "--port",
        str(SERVER_PORT + 1),
        "--log-dir",
        "logs",
        "--status-interval",
        "1",
    ]

    def record(name: str, ok: bool, **details) -> None:
        results[name] = {"ok": bool(ok), **details}
        print(name, "OK" if ok else "FAILED", json.dumps(details)[:400], flush=True)

    dest = work / "server-parallel"
    dest.mkdir()
    run = cli(
        [
            "collect",
            "--uav",
            "03",
            "--uav",
            "04",
            "--dest",
            str(dest),
            "--latest",
            "3",
            *server,
        ]
    )
    vehicles = summary(run)["vehicles"]
    problems = [p for uav in VEHICLES for p in matches_source(dest / uav, sitl[uav])]
    record(
        "server_parallel_multi_file",
        run["code"] == 0
        and not problems
        and all(len(v["files"]) == 3 for v in vehicles.values()),
        files={uav: v["files"] for uav, v in vehicles.items()},
        problems=problems,
        log_disarmed={uav: side[uav].param("LOG_DISARMED") for uav in VEHICLES},
        rates_Bps=[
            e["rate_Bps"]
            for e in run["events"]
            if e["event"] == "status" and e["rate_Bps"]
        ][-4:],
    )

    direct = [
        "--direct",
        f"3=tcp:127.0.0.1:{port('03', 1)}",
        "--log-dir",
        "logs",
        "--status-interval",
        "1",
    ]
    dest = work / "direct"
    run = cli(["collect", *direct, "--dest", str(dest), "--latest", "2"])
    problems = matches_source(dest / "sysid-3", sitl["03"])
    record(
        "direct_multi_file",
        run["code"] == 0 and not problems,
        summary=summary(run),
        problems=problems,
    )

    largest = largest_closed_log(work / "server-parallel" / "04")
    dest = work / "server-resume"
    dest.mkdir()
    args = ["collect", "--uav", "04", "--dest", str(dest), "--file", largest, *server]
    first = cli(args, stop_when=halfway)
    wait_for_receipt(dest / "04", 1)
    part = dest / "04" / f"{largest}.part"
    restored = side["04"].param("LOG_DISARMED")
    second = cli(args)
    resumed = receipt(summary(second)["vehicles"]["04"]["receipt_path"])["files"][0]
    record(
        "server_cancel_then_resume",
        first["interrupted"]
        and first["code"] != 0
        and restored == 1
        and second["code"] == 0
        and resumed["resumed_from_bytes"] > 0
        and not matches_source(dest / "04", sitl["04"]),
        part_existed=part.exists() or resumed["resumed_from_bytes"] > 0,
        log_disarmed_after_cancel=restored,
        resumed_from_bytes=resumed["resumed_from_bytes"],
        bytes=resumed["bytes"],
    )

    largest = largest_closed_log(work / "server-parallel" / "03")
    dest = work / "direct-crash"
    args = ["collect", *direct, "--dest", str(dest), "--file", largest]
    crashed = cli(args, stop_when=halfway, stop_signal=signal.SIGKILL)
    left_off = side["03"].param("LOG_DISARMED")
    state = json.loads((dest / "sysid-3" / "collect-state.json").read_text())
    part = dest / "sysid-3" / f"{largest}.part"
    with part.open("r+b") as stream:
        stream.seek(4096)
        stream.write(b"\x00\xff" * 64)
    rejected = cli(args)
    rejected_receipt = receipt(summary(rejected)["vehicles"]["sysid-3"]["receipt_path"])
    after_reject = side["03"].param("LOG_DISARMED")
    final = cli(args)
    record(
        "direct_crash_corrupt_partial_rejected_then_recovered",
        crashed["interrupted"]
        and left_off == 0
        and state["log_disarmed_original"] == 1
        and rejected["code"] != 0
        and "CRC32" in rejected_receipt["files"][0]["error"]
        and rejected_receipt["log_disarmed"]["recovered_from_interrupted_run"]
        and after_reject == 1
        and not part.exists()
        and final["code"] == 0
        and not matches_source(dest / "sysid-3", sitl["03"]),
        log_disarmed_after_crash=left_off,
        rejection=rejected_receipt["files"][0]["error"],
        log_disarmed_after_rejection=after_reject,
        final=summary(final),
    )

    side["04"].arm(True)
    dest = work / "armed"
    dest.mkdir()
    refused = cli(["collect", "--uav", "04", "--dest", str(dest), *server])
    still = side["04"].param("LOG_DISARMED")
    side["04"].arm(False)
    vehicle = summary(refused)["vehicles"]["04"]
    record(
        "armed_refusal",
        refused["code"] != 0
        and "armed" in vehicle["error"]
        and still == 1
        and not list((dest / "04").glob("*.BIN*")),
        error=vehicle["error"],
        log_disarmed=still,
    )

    dest = work / "armed-mid-transfer"
    dest.mkdir()
    armed_at = []

    def arm_halfway(event: dict) -> bool:
        if halfway(event) and not armed_at:
            side["04"].arm(True)
            armed_at.append(event["bytes_done"])
        return False

    largest = largest_closed_log(work / "server-parallel" / "04")
    aborted = cli(
        ["collect", "--uav", "04", "--dest", str(dest), "--file", largest, *server],
        stop_when=arm_halfway,
    )
    after_abort = side["04"].param("LOG_DISARMED")
    side["04"].arm(False)
    vehicle = summary(aborted)["vehicles"]["04"]
    record(
        "armed_mid_transfer_restores_log_disarmed",
        aborted["code"] != 0
        and bool(armed_at)
        and "armed" in (vehicle["error"] or "")
        and after_abort == 1
        and not (dest / "04" / largest).exists(),
        error=vehicle["error"],
        log_disarmed=after_abort,
    )
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument("--flights", type=int, default=3)
    parser.add_argument("--armed-seconds", type=float, default=6)
    args = parser.parse_args()
    work = args.work.resolve()
    work.mkdir(parents=True, exist_ok=False)
    with ExitStack() as stack:
        sitl = {uav: start_sitl(stack, work, uav) for uav in VEHICLES}
        side = {uav: Side(uav) for uav in VEHICLES}
        make_logs(list(side.values()), args.flights, args.armed_seconds)
        stack.enter_context(
            process(
                [
                    str(Path(sys.executable).with_name("skybrushd")),
                    "-c",
                    str(server_config(work)),
                ],
                work,
                work / "server.log",
            )
        )
        time.sleep(8)
        results = scenarios(work, sitl, side)
    (work / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    passed = all(result["ok"] for result in results.values())
    print("ALL PASSED" if passed else "FAILURES", flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
