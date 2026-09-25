"""Map firmware-owned FC associations without guessing a server UAV ID."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from typing import Mapping

from rtlslink.operations import association_from_values
from rtlslink.protocol import RtlsDevice, decode_param_value


def associations(
    devices: Mapping[int, RtlsDevice],
    sources: Mapping[str, tuple[str, int]],
    identities: Mapping[str, int],
    *,
    now: float,
    max_age: float,
) -> tuple[dict[int, str], dict[int, dict[str, object]]]:
    metadata: dict[int, dict[str, object]] = {}
    for tag_id, device in devices.items():
        names = ("FC_SYS_ID", "FC_STATE")
        if not any(name in device.params for name in names):
            continue
        values = {
            name: decode_param_value(device.params[name], device.param_types[name])
            for name in names
            if name in device.params
        }
        association = association_from_values(values)
        entry: dict[str, object] = asdict(association)
        stamp = min((device.param_received_at.get(name, 0) for name in names))
        entry["age_s"] = max(0.0, now - stamp)
        if association.state == "live" and (device.sleeping or now - stamp > max_age):
            entry["state"] = "remembered"
        if device.conflicting_addresses:
            entry.update(
                state="ambiguous", reason="tag ID appears at multiple addresses"
            )
        metadata[tag_id] = entry
    counts = Counter(
        entry["system_id"] for entry in metadata.values() if entry["system_id"]
    )
    for entry in metadata.values():
        if counts[entry["system_id"]] > 1:
            entry.update(
                state="ambiguous",
                reason="flight-controller ID appears on multiple tags",
            )

    mapping: dict[int, str] = {}
    for tag_id, device in devices.items():
        if device.conflicting_addresses:
            continue
        same_ip = [
            uid for uid, address in sources.items() if address[0] == device.address[0]
        ]
        selected_entry = metadata.get(tag_id)
        if selected_entry is None:
            # Older firmware retains the live source-IP association only.
            if len(same_ip) == 1:
                mapping[tag_id] = same_ip[0]
            continue
        entry = selected_entry
        if entry["state"] not in ("live", "remembered"):
            continue
        candidates = [
            uid for uid, sid in identities.items() if sid == entry["system_id"]
        ]
        if (
            len(candidates) > 1
            or len(same_ip) > 1
            or (same_ip and same_ip != candidates)
        ):
            entry.update(
                state="ambiguous",
                reason="server identity conflicts with onboard association",
            )
            continue
        if len(candidates) == 1:
            uid = candidates[0]
            if uid in sources and sources[uid][0] != device.address[0]:
                entry.update(
                    state="ambiguous",
                    reason="flight controller is live through another tag",
                )
            else:
                mapping[tag_id] = uid
    return mapping, metadata
