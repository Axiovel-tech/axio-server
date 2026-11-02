"""axio-fc-logs: verified flight-controller log collection.

Through a running Axio Server (``--uav``) the server keeps its MAVLink link and
writes the files on its host. A directly attached flight controller
(``--direct SYSID=CONNECTION``) is collected by this process.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import cast

import trio
from flockwave.channels import ParserChannel
from flockwave.encoders.json import create_json_encoder
from flockwave.parsers.json import create_json_parser

from .collect import CollectRequest
from .runner import collect_with_status
from .transport import DirectTransport

__all__ = ("main",)

_COMMAND = "__fc_log_collect"
_REQUEST_ID = "fc-logs-1"


def emit(event: str, **data: object) -> None:
    print(json.dumps({"event": event, **data}, separators=(",", ":")), flush=True)


def summary(receipt: dict[str, object]) -> dict[str, object]:
    files = receipt.get("files")
    items = cast(list[dict[str, object]], files if isinstance(files, list) else [])
    return {
        "complete": bool(receipt.get("complete")),
        "error": receipt.get("error"),
        "files": {str(item["file"]): item["status"] for item in items},
        "log_disarmed": receipt.get("log_disarmed"),
        "receipt_path": receipt.get("receipt_path"),
    }


def direct_target(text: str) -> tuple[int, str]:
    system_id, separator, address = text.partition("=")
    if not separator or not system_id.isdigit() or not 1 <= int(system_id) <= 255:
        raise argparse.ArgumentTypeError("expected SYSID=CONNECTION")
    return int(system_id), address


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(prog="axio-fc-logs", description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    p = commands.add_parser(
        "collect",
        help="close the current log, then download and verify selected logs",
    )
    target = p.add_mutually_exclusive_group(required=True)
    target.add_argument("--uav", action="append", help="server UAV ID (repeatable)")
    target.add_argument(
        "--direct",
        action="append",
        type=direct_target,
        metavar="SYSID=CONNECTION",
        help="serial device or pymavlink address, e.g. 3=/dev/ttyACM0",
    )
    p.add_argument(
        "--dest",
        required=True,
        type=Path,
        help="existing directory; each vehicle gets a subdirectory",
    )
    pick = p.add_mutually_exclusive_group()
    pick.add_argument("--latest", type=int, default=1, help="newest N logs (default 1)")
    pick.add_argument(
        "--file", action="append", help="log file name, e.g. 00000042.BIN"
    )
    p.add_argument("--log-dir", default="/APM/LOGS", help="SITL uses 'logs'")
    p.add_argument(
        "--min-battery-voltage",
        type=float,
        help="refuse to start or stop below this voltage; unset for USB power",
    )
    p.add_argument("--status-interval", type=float, default=5.0)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5001)
    p.add_argument("--baud", type=int, default=115200)
    return root


def request_for(args: argparse.Namespace, directory: Path) -> CollectRequest:
    return CollectRequest(
        directory=directory,
        log_dir=args.log_dir,
        latest=args.latest,
        files=tuple(args.file or ()),
        min_battery_voltage=args.min_battery_voltage,
    )


async def collect_direct(
    args: argparse.Namespace, system_id: int, address: str
) -> dict[str, object]:
    uav = f"sysid-{system_id}"
    request = request_for(args, args.dest.resolve() / uav)
    receipt: dict[str, object] = {}
    try:
        async with DirectTransport.open(
            address, system_id, baud=args.baud
        ) as transport:
            async with collect_with_status(
                transport, request, uav, args.status_interval
            ) as items:
                async for kind, data in items:
                    if kind == "status":
                        emit("status", **data)
                    else:
                        receipt = data
    except (OSError, RuntimeError, trio.TooSlowError) as error:
        receipt = {"complete": False, "error": str(error) or type(error).__name__}
    return receipt


class ServerSession:
    """One OBJ-CMD for all UAVs and its asynchronous progress and results."""

    def __init__(self, stream: trio.SocketStream, uavs: list[str]):
        self._stream = stream
        self._messages = ParserChannel(stream.receive_some, create_json_parser())
        self._encode = create_json_encoder()
        self._uavs = uavs
        self.receipts: dict[str, str] = {}
        self.results: dict[str, dict[str, object]] = {}

    async def send(self, body: dict[str, object], identifier: str) -> None:
        message = {"$fw.version": "1.0", "id": identifier, "body": body}
        await self._stream.send_all(self._encode(message))

    async def run(self, kwds: dict[str, object]) -> None:
        await self.send(
            {"type": "OBJ-CMD", "ids": self._uavs, "command": _COMMAND, "kwds": kwds},
            _REQUEST_ID,
        )
        early: list[dict] = []
        answered = False
        while not answered or len(self.results) < len(self._uavs):
            message = await self._messages.receive()
            body = message["body"]
            if message.get("refs") == _REQUEST_ID:
                answered = True
                self._accept(body)
                for item in early:
                    self._handle(item)
            elif not answered:
                early.append(body)
            else:
                self._handle(body)

    async def cancel(self) -> None:
        pending = [r for r, uid in self.receipts.items() if uid not in self.results]
        if pending:
            await self.send({"type": "ASYNC-CANCEL", "ids": pending}, "fc-logs-cancel")

    def _accept(self, body: dict) -> None:
        if body.get("type") == "ACK-NAK":
            raise RuntimeError(body.get("reason", "server rejected the request"))
        for uid, error in body.get("error", {}).items():
            self.results[uid] = {"complete": False, "error": str(error)}
        for uid, result in body.get("result", {}).items():
            self.results[uid] = result
        self.receipts = {
            receipt: uid for uid, receipt in body.get("receipt", {}).items()
        }
        for uid in self._uavs:
            if uid not in self.results and uid not in self.receipts.values():
                self.results[uid] = {
                    "complete": False,
                    "error": "server omitted this UAV",
                }

    def _handle(self, body: dict) -> None:
        kind = body.get("type")
        if kind == "ASYNC-ST" and body.get("id") in self.receipts:
            status = body.get("progress", {}).get("object")
            if isinstance(status, dict):
                emit("status", **status)
        elif kind == "ASYNC-RESP" and body.get("id") in self.receipts:
            uid = self.receipts[body["id"]]
            result = body.get("result")
            self.results[uid] = (
                result
                if isinstance(result, dict)
                else {"complete": False, "error": str(body.get("error"))}
            )
        elif kind == "ASYNC-TIMEOUT":
            for receipt in body.get("ids", []):
                if receipt in self.receipts:
                    self.results[self.receipts[receipt]] = {
                        "complete": False,
                        "error": "server stopped reporting progress",
                    }


async def collect_through_server(args: argparse.Namespace) -> dict[str, dict]:
    if not args.dest.is_absolute():
        raise ValueError("--dest must be absolute; the server writes on its host")
    kwds = {
        "dest": str(args.dest),
        "latest": args.latest,
        "files": args.file,
        "log_dir": args.log_dir,
        "min_battery_voltage": args.min_battery_voltage,
        "status_interval": args.status_interval,
    }
    async with await trio.open_tcp_stream(args.host, args.port) as stream:
        session = ServerSession(stream, list(dict.fromkeys(args.uav)))
        try:
            await session.run(kwds)
        except KeyboardInterrupt:
            await session.cancel()
            raise
    return session.results


async def run(args: argparse.Namespace) -> int:
    request_for(args, args.dest)
    if args.uav:
        results = await collect_through_server(args)
    else:
        results = {}

        async def one(system_id: int, address: str) -> None:
            results[f"sysid-{system_id}"] = await collect_direct(
                args, system_id, address
            )

        async with trio.open_nursery() as nursery:
            for system_id, address in args.direct:
                nursery.start_soon(one, system_id, address)
    vehicles = {uid: summary(receipt) for uid, receipt in results.items()}
    complete = bool(vehicles) and all(v["complete"] for v in vehicles.values())
    emit("summary", complete=complete, vehicles=vehicles)
    return 0 if complete else 1


def main() -> int:
    args = parser().parse_args()
    try:
        return trio.run(run, args)
    except KeyboardInterrupt:
        emit("interrupted", note="receipts and partial files are on disk")
        return 130
    except (ValueError, RuntimeError, OSError) as error:
        emit("summary", complete=False, error=str(error) or type(error).__name__)
        return 1


if __name__ == "__main__":
    sys.exit(main())
