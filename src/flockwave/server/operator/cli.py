"""Agent-friendly inventory, FC parameter operations and telemetry."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path
from typing import cast

import trio

from . import tags
from .client import ServerClient

RC_SWITCH_PARAMETERS = [
    "FLTMODE_CH",
    *(f"FLTMODE{i}" for i in range(1, 7)),
    *(f"RC{i}_OPTION" for i in range(1, 17)),
    "RCMAP_ROLL",
    "RCMAP_PITCH",
    "RCMAP_THROTTLE",
    "RCMAP_YAW",
]


def numeric_values(value: object) -> dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError("expected an object of parameter names to finite numbers")
    result: dict[str, float] = {}
    for key, item in cast(dict[object, object], value).items():
        if (
            not isinstance(key, str)
            or not isinstance(item, (int, float))
            or isinstance(item, bool)
            or not math.isfinite(item)
        ):
            raise ValueError("parameter values must be finite numbers")
        result[key] = item
    return result


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        prog="axio-operator", description="Verified operations through Axio Server"
    )
    root.add_argument("--host", default="127.0.0.1")
    root.add_argument("--port", type=int, default=5001)
    root.add_argument("--timeout", type=float, default=30.0)
    sub = root.add_subparsers(dest="command", required=True)
    tags.add_parsers(sub)
    sub.add_parser(
        "devices", help="RTLS identities, remembered FC association and server UAV IDs"
    )
    p = sub.add_parser(
        "telemetry", help="fresh flight-controller temperatures and heartbeat"
    )
    p.add_argument("--uav", action="append", required=True)
    p.add_argument(
        "--sensor",
        action="append",
        choices=["MCU", "IMU", "IMU2", "IMU3", "barometer", "barometer2"],
    )
    p = sub.add_parser(
        "params", help="selected FC parameters; apply verifies fresh readback"
    )
    actions = p.add_subparsers(dest="operation", required=True)
    for action in ("read", "compare", "apply"):
        q = actions.add_parser(action)
        q.add_argument("--uav", action="append", required=True)
        q.add_argument("--name", action="append")
        q.add_argument("--profile", choices=["rc-switches"])
        if action != "read":
            source = q.add_mutually_exclusive_group(required=True)
            source.add_argument(
                "--values", help="JSON file of desired values in write order"
            )
            source.add_argument(
                "--reference", help="server UAV ID to copy selected values from"
            )
    return root


async def run(args: argparse.Namespace) -> int:
    async with ServerClient.connect(args.host, args.port) as client:
        if args.command in ("power", "geometry"):
            return await tags.run(client, args)
        if args.command == "devices":
            data = await client.request({"type": "X-RTLS-INF"}, timeout=args.timeout)
            data["uavs"] = await client.request(
                {"type": "UAV-LIST"}, timeout=args.timeout
            )
            print(json.dumps(data, indent=2, allow_nan=False))
            return 0
        names = args.sensor if args.command == "telemetry" else (args.name or [])
        if getattr(args, "profile", None) == "rc-switches":
            names = list(dict.fromkeys([*RC_SWITCH_PARAMETERS, *names]))
        desired = None
        if getattr(args, "values", None):
            desired = numeric_values(json.loads(Path(args.values).read_text()))
            if not isinstance(desired, dict):
                raise ValueError("values file must contain a JSON object")
        if getattr(args, "reference", None):
            if args.reference in args.uav:
                raise ValueError("reference must be distinct from selected targets")
            reference = await client.operate(
                [args.reference], "read", names=names, timeout=args.timeout
            )
            if not reference.complete:
                print(
                    json.dumps(
                        {"reference": asdict(reference), "complete": False}, indent=2
                    )
                )
                return 1
            entry = reference.results[args.reference]
            reference_values = numeric_values(entry.get("values"))
            desired = {name: reference_values[name] for name in names}
        operation = "telemetry" if args.command == "telemetry" else args.operation
        result = await client.operate(
            args.uav, operation, names=names, values=desired, timeout=args.timeout
        )
        complete = result.complete and all(
            row.get("consistent", True) for row in result.results.values()
        )
        print(
            json.dumps(
                {**asdict(result), "complete": complete}, indent=2, allow_nan=False
            )
        )
        return 0 if complete else 1


def main() -> int:
    args = parser().parse_args()
    try:
        return trio.run(run, args)
    except (
        ValueError,
        RuntimeError,
        OSError,
        trio.TooSlowError,
        trio.EndOfChannel,
    ) as error:
        print(
            json.dumps({"complete": False, "error": str(error) or type(error).__name__})
        )
        return 1
