"""Resolve server drone selections, then delegate tag work to the RTLS SDK."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict

from rtlslink import (
    RtlsClient,
    association_from_values,
    compare_geometry,
    read_geometry,
    read_parameters,
    set_power,
)

from .client import ServerClient


def add_parsers(sub) -> None:
    parser = sub.add_parser(
        "power", help="wake/sleep a mapped drone through the RTLS SDK"
    )
    targets = parser.add_mutually_exclusive_group(required=True)
    targets.add_argument("--uav", help="exact server UAV ID from devices")
    targets.add_argument(
        "--fc-id", type=int, help="onboard FC system ID, including sleeping drones"
    )
    state = parser.add_mutually_exclusive_group(required=True)
    state.add_argument("--sleep", action="store_true")
    state.add_argument("--wake", action="store_true")
    parser.add_argument(
        "--wait-for", choices=["tag", "flight-controller"], default="tag"
    )
    parser = sub.add_parser(
        "geometry", help="compare actual fitted anchor tables for mapped drones"
    )
    parser.add_argument(
        "--uav", action="append", required=True, help="exact server UAV ID from devices"
    )
    parser.add_argument("--tolerance", type=float, default=0.05)


def mapped_tag(
    inventory: dict, *, uav: str | None = None, fc_id: int | None = None
) -> dict:
    matches = []
    for row in inventory.get("status", {}).values():
        association = row.get("flightController", {})
        if (uav is not None and row.get("uav") == uav) or (
            fc_id is not None and association.get("system_id") == fc_id
        ):
            if association.get("state") not in ("live", "remembered"):
                raise ValueError("drone association is unknown or ambiguous")
            matches.append(row)
    if len(matches) != 1:
        raise ValueError("drone has no unique online tag association")
    return matches[0]


async def run(client: ServerClient, args: argparse.Namespace) -> int:
    inventory = await client.request({"type": "X-RTLS-INF"}, timeout=args.timeout)
    if args.command == "power":
        rows = [mapped_tag(inventory, uav=args.uav, fc_id=args.fc_id)]
    else:
        rows = [mapped_tag(inventory, uav=uid) for uid in args.uav]
    addresses = [tuple(row["address"]) for row in rows]
    async with RtlsClient(targets=addresses, broadcast=(), auto_list=False) as rtls:
        devices = [await rtls.resolve(address, args.timeout) for address in addresses]
        for row, device in zip(rows, devices):
            if row["id"] != device.system_id:
                raise ValueError("tag address changed since inventory")
            rtls.require_unique(device)
            snapshot = await read_parameters(rtls, device, ["FC_SYS_ID", "FC_STATE"])
            association = association_from_values(
                {name: item.value for name, item in snapshot.values.items()}
            )
            if (
                association.state not in ("live", "remembered")
                or association.system_id != row["flightController"]["system_id"]
            ):
                raise ValueError("onboard association changed since inventory")
        if args.command == "power":
            result = await set_power(
                rtls,
                devices[0],
                args.sleep,
                wait_for=args.wait_for,
                timeout=args.timeout,
            )
            output = asdict(result)
            success = result.complete
        else:
            snapshots = [
                await read_geometry(rtls, device, timeout=min(2.0, args.timeout))
                for device in devices
            ]
            agreement = compare_geometry(
                snapshots,
                expected_ids=[device.system_id for device in devices],
                tolerance_m=args.tolerance,
            )
            output = {
                "snapshots": [asdict(snapshot) for snapshot in snapshots],
                "agreement": asdict(agreement),
            }
            success = agreement.consistent
    print(json.dumps(output, indent=2, allow_nan=False))
    return 0 if success else 1
