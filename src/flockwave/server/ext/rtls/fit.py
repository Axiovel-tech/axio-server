"""Rolling-summary calibration integration for ``X-RTLS-GEO fit``."""

from __future__ import annotations

import time
from dataclasses import dataclass
from statistics import median
from typing import TYPE_CHECKING, Any, Sequence

import trio

from .anchor_geometry import (
    FitResult,
    RangeObservation,
    fit_refined,
    fit_strict,
)
from .extension import _decoded_device_params
from .geometry import get_canonical

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "TwrSummary",
    "on_twr_summary",
    "run_fit",
)

SUMMARY_PROTOCOL_VERSION = 1
SUMMARY_FRESHNESS_S = 2.5
SUMMARY_WAIT_TIMEOUT_S = 3.0
MIN_SAMPLES_PER_PAIR = 20
#: UWB_ROLE wire value of the DL-TDoA / inter-anchor TWR initiator (A0);
#: mirrors ``rtls_link::uwb::role_anchor_initiator`` in the firmware.
ROLE_ANCHOR_INITIATOR = 2


@dataclass(frozen=True)
class TwrSummary:
    """One complete generation emitted by an A0 rolling window."""

    system_id: int
    version: int
    sequence: int
    valid_mask: int
    time_boot_ms: int
    ranges: tuple[RangeObservation, ...]
    received_at: float


@dataclass(frozen=True)
class _FitSession:
    summary: TwrSummary
    strict: FitResult
    reference: dict[str, Any]
    cell: str


def _parse_summary(
    system_id: int, data: dict[str, Any], now: float
) -> TwrSummary | None:
    """Validate the SDK's coherent ``twr_summary`` event."""
    try:
        version = int(data["version"])
        sequence = int(data["sequence"])
        valid_mask = int(data["validMask"])
        time_boot_ms = int(data["timeBootMs"])
        raw_ranges = data["ranges"]
        if (
            version <= 0
            or sequence < 0
            or valid_mask < 0
            or time_boot_ms < 0
            or not isinstance(raw_ranges, Sequence)
        ):
            return None
        ranges = tuple(
            RangeObservation(
                anchor_index=-1,  # resolved from the canonical MAC table
                peer_mac=int(item["peerMac"]),
                distance_m=float(item["distanceM"]),
                mad_m=float(item["madM"]),
                count=int(item["count"]),
            )
            for item in raw_ranges
        )
    except (KeyError, TypeError, ValueError):
        return None
    if (
        len({item.peer_mac for item in ranges}) != len(ranges)
        or any(
            item.distance_m <= 0 or item.mad_m < 0 or item.count <= 0
            for item in ranges
        )
    ):
        return None
    return TwrSummary(
        system_id=system_id,
        version=version,
        sequence=sequence,
        valid_mask=valid_mask,
        time_boot_ms=time_boot_ms,
        ranges=ranges,
        received_at=now,
    )


def on_twr_summary(
    ext: "RtlsExtension", system_id: int, data: dict[str, Any], now: float
) -> None:
    """Cache one SDK-validated coherent generation and wake fit waiters."""
    summary = _parse_summary(system_id, data, now)
    if summary is None:
        return
    previous = ext._twr_summaries.get(system_id)
    if (
        previous is not None
        and previous.sequence == summary.sequence
        and previous.time_boot_ms == summary.time_boot_ms
    ):
        return
    ext._twr_summaries[system_id] = summary
    changed = ext._twr_summary_changed
    ext._twr_summary_changed = trio.Event()
    changed.set()


def _configured_anchors(reference: dict[str, Any]) -> list[dict[str, Any]]:
    count = int(reference["UWB_AN_COUNT"])
    if count != 8:
        raise ValueError(
            f"four-tripod calibration requires exactly 8 anchors ({count} configured)"
        )
    return [
        {
            "index": index,
            "mac": int(reference[f"UWB_AN{index}_MAC"]),
        }
        for index in range(count)
    ]


def _find_a0(ext: "RtlsExtension", a0_mac: int) -> int:
    candidates: list[int] = []
    for system_id, device in ext._require_protocol().devices.items():
        params = _decoded_device_params(device)
        try:
            role = int(params.get("UWB_ROLE", -1))
            mac = int(params.get("UWB_MAC", -1))
        except (TypeError, ValueError):
            continue
        if role == ROLE_ANCHOR_INITIATOR and mac == a0_mac:
            candidates.append(system_id)
    if not candidates:
        raise ValueError(
            "A0 initiator is not online or its UWB role/MAC is not known"
        )
    if len(candidates) > 1:
        raise ValueError("multiple online initiators claim the configured A0 MAC")
    return candidates[0]


def _resolve_observations(
    summary: TwrSummary, anchors: Sequence[dict[str, Any]]
) -> tuple[RangeObservation, ...]:
    if summary.version != SUMMARY_PROTOCOL_VERSION:
        raise ValueError(
            f"A0 rolling-summary protocol version {summary.version} is unsupported"
        )
    expected_mask = sum(1 << index for index in range(1, len(anchors)))
    missing_slots = [
        index
        for index in range(1, len(anchors))
        if not summary.valid_mask & (1 << index)
    ]
    by_mac = {item.peer_mac: item for item in summary.ranges}
    missing_ranges = [
        anchor["index"]
        for anchor in anchors[1:]
        if anchor["mac"] not in by_mac
    ]
    if missing_slots or missing_ranges or summary.valid_mask & expected_mask != expected_mask:
        missing = sorted({*missing_slots, *missing_ranges})
        raise ValueError(
            "A0 summary is incomplete; missing " + ", ".join(f"A{i}" for i in missing)
        )

    observations = tuple(
        RangeObservation(
            anchor_index=anchor["index"],
            peer_mac=anchor["mac"],
            distance_m=by_mac[anchor["mac"]].distance_m,
            mad_m=by_mac[anchor["mac"]].mad_m,
            count=by_mac[anchor["mac"]].count,
        )
        for anchor in anchors[1:]
    )
    low_count = [
        item.anchor_index
        for item in observations
        if item.count < MIN_SAMPLES_PER_PAIR
    ]
    if low_count:
        raise ValueError(
            "A0 summary has insufficient samples for "
            + ", ".join(f"A{i}" for i in low_count)
            + f" (minimum {MIN_SAMPLES_PER_PAIR})"
        )
    return observations


async def _wait_for_summary(
    ext: "RtlsExtension",
    system_id: int,
    *,
    newer_than: float,
    timeout: float,
) -> TwrSummary:
    deadline = time.monotonic() + timeout
    while True:
        changed = ext._twr_summary_changed
        summary = ext._twr_summaries.get(system_id)
        now = time.monotonic()
        if (
            summary is not None
            and summary.received_at >= newer_than
            and now - summary.received_at <= SUMMARY_FRESHNESS_S
        ):
            return summary
        remaining = deadline - now
        if remaining <= 0:
            break
        with trio.move_on_after(remaining) as scope:
            await changed.wait()
        if scope.cancelled_caught:
            break
    if system_id not in ext._twr_summaries:
        raise ValueError(
            "no rolling TWR summary arrived from A0; check that A0 firmware "
            "supports the feature and its management telemetry reaches the server"
        )
    raise ValueError(
        "no fresh rolling TWR summary arrived from A0 within "
        f"{timeout:.1f} seconds"
    )


def _apply_geometry(
    reference: dict[str, Any],
    anchors: Sequence[dict[str, Any]],
    result: FitResult,
) -> dict[str, Any]:
    payload = {
        name: reference[name]
        for name in (
            "ORIGIN_LAT_E7",
            "ORIGIN_LON_E7",
            "ORIGIN_ALT_MM",
            "CELL_ID",
            "UWB_AN_COUNT",
        )
        if name in reference
    }
    payload["POS_YAW_DEG"] = 0.0
    positions = {int(item["index"]): item for item in result.anchors}
    for anchor in anchors:
        index = anchor["index"]
        position = positions[index]
        payload[f"UWB_AN{index}_X"] = position["xM"]
        payload[f"UWB_AN{index}_Y"] = position["yM"]
        payload[f"UWB_AN{index}_Z"] = position["zM"]
        payload[f"UWB_AN{index}_MAC"] = anchor["mac"]
        bias_name = f"UWB_AN{index}_BIAS_M"
        if bias_name in reference:
            payload[bias_name] = reference[bias_name]
    return payload


def _summary_json(summary: TwrSummary, observations: Sequence[RangeObservation]) -> dict:
    return {
        "systemId": summary.system_id,
        "version": summary.version,
        "sequence": summary.sequence,
        "timeBootMs": summary.time_boot_ms,
        "validMask": summary.valid_mask,
        "ageMs": round(max(0.0, time.monotonic() - summary.received_at) * 1000),
        "ranges": [
            {
                "anchorIndex": item.anchor_index,
                "peerMac": item.peer_mac,
                "distanceM": round(item.distance_m, 5),
                "madM": round(item.mad_m, 5),
                "count": item.count,
            }
            for item in observations
        ],
    }


async def run_fit(
    ext: "RtlsExtension",
    *,
    mode: str,
    summary_sequence: int | None = None,
    timeout: float = SUMMARY_WAIT_TIMEOUT_S,
) -> dict[str, Any]:
    """Run the strict fit on a new summary or refine the pinned summary."""
    if mode not in ("strict", "refined"):
        raise ValueError(f"Invalid fit mode: {mode!r} (expected strict or refined)")
    timeout = max(0.1, min(float(timeout), SUMMARY_WAIT_TIMEOUT_S))

    if mode == "strict":
        reference, cell = get_canonical(ext, None)
        anchors = _configured_anchors(reference)
        system_id = _find_a0(ext, anchors[0]["mac"])
        requested_at = time.monotonic()
        summary = await _wait_for_summary(
            ext, system_id, newer_than=requested_at, timeout=timeout
        )
        observations = _resolve_observations(summary, anchors)
        strict = fit_strict(observations)
        session = _FitSession(summary, strict, reference, cell)
        ext._geo_fit_session = session
        refined = None
    else:
        session = ext._geo_fit_session
        if session is None:
            raise ValueError("run a strict fit first to pin a calibration summary")
        if summary_sequence is None:
            raise ValueError("summarySequence is required for a refined fit")
        if int(summary_sequence) != session.summary.sequence:
            raise ValueError(
                "the requested summary is no longer the pinned calibration snapshot"
            )
        summary = session.summary
        strict = session.strict
        reference, cell = session.reference, session.cell
        anchors = _configured_anchors(reference)
        observations = _resolve_observations(summary, anchors)
        refined = fit_refined(observations, strict=strict)

    selected = strict if mode == "strict" else refined
    assert selected is not None
    selected_model = selected.model if selected.accepted else None
    payload = (
        _apply_geometry(reference, anchors, selected)
        if selected.accepted
        else None
    )
    body: dict[str, Any] = {
        "type": "X-RTLS-GEO",
        "op": "fit",
        "mode": mode,
        "cell": cell,
        "summary": _summary_json(summary, observations),
        "strict": strict.json(),
        "refined": refined.json() if refined is not None else None,
        "selectedModel": selected_model,
        "applyGeometry": payload,
    }
    if refined is not None:
        noise_floor = max(
            0.01, float(median(item.mad_m for item in observations))
        )
        improvement = strict.rms_m - refined.rms_m
        body["comparison"] = {
            "rmsImprovementM": round(improvement, 5),
            "noiseFloorM": round(noise_floor, 5),
            "meaningfulImprovement": improvement > noise_floor,
        }
    return body
