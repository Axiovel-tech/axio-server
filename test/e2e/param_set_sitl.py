"""End-to-end check of PRM-SET against ArduPilot SITL through a live server.

Not collected by pytest (no ``test_`` prefix). Run from the server checkout,
with ``ARDUPILOT_SITL`` and ``SITL_DEFAULTS`` exported:

    heavy timeout 600 .venv/bin/python test/e2e/param_set_sitl.py --work /tmp/prm-set-e2e

The vehicle reaches the server over UDP (SERIAL0) through a relay that can
lose the server's PARAM_SET, alone or answered by a replay of the last
PARAM_VALUE (the late reply to the read before the write, as on the bench).
SERIAL1 is a side channel that reads the value the vehicle really holds.
Results go to ``<work>/results.json``.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
import uuid
from contextlib import ExitStack
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from fc_log_collection_sitl import process  # noqa: E402
from pymavlink import mavutil  # noqa: E402

SERVER_PORT = 25100
MAVLINK_PORT = 24650
RELAY_PORT = 24660
INSTANCE = 6
UAV = "03"
PARAM = "DISARM_DELAY"
PARAM_SET_ID = 23
PARAM_VALUE_ID = 22


class Relay:
    """UDP relay between the vehicle and the server with a loss mode."""

    def __init__(self) -> None:
        self.vehicle_side = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vehicle_side.bind(("127.0.0.1", RELAY_PORT))
        self.server_side = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_side.bind(("127.0.0.1", 0))
        self.vehicle: tuple[str, int] | None = None
        self.mode = "pass"
        self.dropped = 0
        self.last_param_value: dict[str, bytes] = {}
        self.parser = mavutil.mavlink.MAVLink(None)
        self.parser.robust_parsing = True

    def start(self) -> None:
        for target in (self._from_vehicle, self._from_server):
            threading.Thread(target=target, daemon=True).start()

    def _messages(self, data: bytes):
        return self.parser.parse_buffer(data) or []

    def _from_vehicle(self) -> None:
        while True:
            data, self.vehicle = self.vehicle_side.recvfrom(65536)
            for message in self._messages(data):
                if message.get_msgId() == PARAM_VALUE_ID:
                    self.last_param_value[message.param_id] = message.get_msgbuf()
            self.server_side.sendto(data, ("127.0.0.1", MAVLINK_PORT))

    def _from_server(self) -> None:
        while True:
            data, _ = self.server_side.recvfrom(65536)
            sets = [m for m in self._messages(data) if m.get_msgId() == PARAM_SET_ID]
            if sets and self.mode != "pass":
                self.dropped += 1
                stale = self.last_param_value.get(sets[0].param_id)
                if self.mode == "drop-set-replay-stale" and stale:
                    self.server_side.sendto(bytes(stale), ("127.0.0.1", MAVLINK_PORT))
                continue
            if self.vehicle is not None:
                self.vehicle_side.sendto(data, self.vehicle)


class Side:
    """Reads the vehicle's own parameter value over SERIAL1."""

    def __init__(self) -> None:
        self.link = mavutil.mavlink_connection(
            f"tcp:127.0.0.1:{5762 + 10 * INSTANCE}", source_system=240
        )
        self.link.wait_heartbeat(timeout=60)

    def read(self, name: str) -> float:
        while self.link.recv_match(blocking=False) is not None:
            pass
        for _ in range(10):
            self.link.mav.param_request_read_send(int(UAV), 1, name.encode(), -1)
            reply = self.link.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if reply and reply.param_id == name:
                return reply.param_value
        raise TimeoutError(name)


class Client:
    """The server's JSON API over TCP, one request at a time."""

    def __init__(self, timeout: float = 60) -> None:
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.sock = socket.create_connection(
                    ("127.0.0.1", SERVER_PORT + 1), timeout=60
                )
                break
            except ConnectionRefusedError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(1)
        self.buffer = b""

    def request(self, body: dict, timeout: float = 60) -> dict:
        message_id = str(uuid.uuid4())
        envelope = {"$fw.version": "1.0", "id": message_id, "body": body}
        self.sock.sendall((json.dumps(envelope) + "\n").encode())
        receipt, deadline = None, time.monotonic() + timeout
        while time.monotonic() < deadline:
            body = self._next()
            if body.get("type") == "ASYNC-RESP" and body.get("id") == receipt:
                return body
            if receipt is None and body.get("type") == body_type(envelope):
                receipts = body.get("receipt") or {}
                if UAV not in receipts:
                    return body
                receipt = receipts[UAV]
        raise TimeoutError(body_type(envelope))

    def _next(self) -> dict:
        while b"\n" not in self.buffer:
            self.buffer += self.sock.recv(65536)
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line).get("body", {})


def body_type(envelope: dict) -> str:
    return envelope["body"]["type"]


def start_sitl(stack: ExitStack, work: Path) -> None:
    directory = work / "sitl"
    directory.mkdir(parents=True, exist_ok=True)
    args = [
        os.environ["ARDUPILOT_SITL"],
        "--model", "+",
        "--speedup", "1",
        f"-I{INSTANCE}",
        "--defaults", os.environ["SITL_DEFAULTS"],
        "--sysid", str(int(UAV)),
        "--serial0", f"udpclient:127.0.0.1:{RELAY_PORT}",
    ]  # fmt: skip
    stack.enter_context(process(args, directory, directory / "sitl.out"))


def start_server(stack: ExitStack, work: Path) -> None:
    url = f"udp-listen://127.0.0.1:{MAVLINK_PORT}?broadcast_port={MAVLINK_PORT + 5}"
    network = {"connections": [url]}
    config = {
        "PORT": SERVER_PORT,
        "EXTENSIONS": {"mavlink": {"enabled": True, "networks": {"mav": network}}},
    }
    path = work / "server.jsonc"
    path.write_text(json.dumps(config))
    server = [str(Path(sys.executable).with_name("skybrushd")), "-c", str(path)]
    stack.enter_context(process(server, work, work / "server.log"))


def wait_for_uav(client: Client, timeout: float = 60) -> None:
    deadline = time.monotonic() + timeout
    while UAV not in (client.request({"type": "UAV-LIST"}).get("ids") or []):
        if time.monotonic() > deadline:
            raise TimeoutError(f"UAV {UAV} never appeared")
        time.sleep(1)


def wait_for_server_to_read(client: Client, side: Side, timeout: float = 30) -> None:
    """Until the server knows the autopilot it encodes parameters bytewise,
    so a write in that window lands mangled; wait until it reads right."""
    deadline = time.monotonic() + timeout
    expected = side.read(PARAM)
    while (
        client.request({"type": "PRM-GET", "ids": [UAV], "name": PARAM}).get("result")
        != expected
    ):
        if time.monotonic() > deadline:
            raise TimeoutError("the server never read the vehicle's value")
        time.sleep(1)


def set_through_server(client: Client, value: float) -> tuple[dict, float]:
    started = time.monotonic()
    reply = client.request(
        {"type": "PRM-SET", "ids": [UAV], "name": PARAM, "value": value}
    )
    return reply, time.monotonic() - started


def scenarios(client: Client, relay: Relay, side: Side) -> dict:
    results: dict[str, dict] = {}
    original = side.read(PARAM)

    def run(name: str, mode: str, value: float, expect_value: float | None) -> None:
        relay.mode, relay.dropped = mode, 0
        reply, seconds = set_through_server(client, value)
        relay.mode = "pass"
        held = side.read(PARAM)
        if expect_value is None:
            ok = "error" in reply and held != value
        else:
            ok = reply.get("result") == expect_value and held == expect_value
        results[name] = {
            "ok": ok,
            "reply": {k: reply[k] for k in ("result", "error") if k in reply},
            "vehicle_holds": held,
            "param_sets_dropped": relay.dropped,
            "seconds": round(seconds, 1),
        }
        print(name, "OK" if ok else "FAILED", json.dumps(results[name]), flush=True)

    written = original + 3
    run("confirmed write returns the value", "pass", written, written)
    run(
        "lost write answered by a stale PARAM_VALUE",
        "drop-set-replay-stale",
        original,
        None,
    )
    run("lost write with no answer", "drop-set", original, None)
    run("restore", "pass", original, original)
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", type=Path, required=True)
    args = parser.parse_args()
    work = args.work.resolve()
    work.mkdir(parents=True, exist_ok=False)
    relay = Relay()
    relay.start()
    with ExitStack() as stack:
        start_sitl(stack, work)
        side = Side()
        start_server(stack, work)
        client = Client()
        wait_for_uav(client)
        wait_for_server_to_read(client, side)
        results = scenarios(client, relay, side)
    (work / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    passed = all(result["ok"] for result in results.values())
    print("ALL PASSED" if passed else "FAILURES", flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
