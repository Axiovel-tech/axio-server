"""Distributed rolling-summary calibration for ``X-RTLS-GEO fit``."""

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
    "DeviceTwrSummary",
    "on_twr_summary",
    "run_fit",
)

SUMMARY_PROTOCOL_VERSION = 1
SUMMARY_FRESHNESS_S = 2.5
SUMMARY_WAIT_TIMEOUT_S = 4.0
MAX_CAPTURE_SKEW_S = 1.5
MIN_SAMPLES_PER_PAIR = 20

# UWB_ROLE wire values; mirrored from rtls_link::uwb::params.
ROLE_ANCHOR_INITIATOR = 2
ROLE_ANCHOR_RESPONDER = 3


@dataclass(frozen=True)
class _Anchor:
    index: int
    mac: int


@dataclass(frozen=True)
class _TwrRange:
    peer_mac: int
    distance_m: float
    mad_m: float
    count: int


@dataclass(frozen=True)
class DeviceTwrSummary:
    """One complete rolling generation emitted by one responder."""

    system_id: int
    version: int
    sequence: int
    valid_mask: int
    time_boot_ms: int
    ranges: tuple[_TwrRange, ...]
    received_at: float


@dataclass(frozen=True)
class _CaptureSource:
    anchor: _Anchor
    summary: DeviceTwrSummary


@dataclass(frozen=True)
class _CalibrationCapture:
    capture_id: int
    sources: tuple[_CaptureSource, ...]
    observations: tuple[RangeObservation, ...]
    participant_system_ids: frozenset[int]


@dataclass(frozen=True)
class _FitSession:
    capture: _CalibrationCapture
    strict: FitResult
    reference: dict[str, Any]
    cell: str


def _parse_summary(
    system_id: int, data: dict[str, Any], now: float
) -> DeviceTwrSummary | None:
    """Validate the SDK's coherent per-device ``twr_summary`` event."""
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
            _TwrRange(
                peer_mac=int(item["peerMac"]),
                distance_m=float(item["distanceM"]),
                mad_m=float(item["madM"]),
                count=int(item["count"]),
            )
            for item in raw_ranges
        )
    except (KeyError, TypeError, ValueError):
        return None
    if len({item.peer_mac for item in ranges}) != len(ranges) or any(
        item.distance_m <= 0 or item.mad_m < 0 or item.count <= 0 for item in ranges
    ):
        return None
    return DeviceTwrSummary(
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
    """Cache one SDK-validated generation and wake capture waiters."""
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


def _configured_anchors(reference: dict[str, Any]) -> tuple[_Anchor, ...]:
    count = int(reference["UWB_AN_COUNT"])
    if count != 8:
        raise ValueError(
            f"four-tripod calibration requires exactly 8 anchors ({count} configured)"
        )
    anchors = tuple(
        _Anchor(index=index, mac=int(reference[f"UWB_AN{index}_MAC"]))
        for index in range(count)
    )
    macs = [anchor.mac for anchor in anchors]
    if any(mac in (0, 0xFFFF) for mac in macs) or len(set(macs)) != len(macs):
        raise ValueError(
            "the canonical anchor table contains invalid or duplicate MACs"
        )
    return anchors


def _resolve_anchor_devices(
    ext: "RtlsExtension", anchors: Sequence[_Anchor]
) -> dict[int, int]:
    """Map each configured anchor slot to one unique online device."""
    claims: dict[int, list[tuple[int, int]]] = {}
    for system_id, device in ext._require_protocol().devices.items():
        params = _decoded_device_params(device)
        try:
            role = int(params.get("UWB_ROLE", -1))
            mac = int(params.get("UWB_MAC", -1))
        except (TypeError, ValueError):
            continue
        claims.setdefault(mac, []).append((system_id, role))

    resolved: dict[int, int] = {}
    for anchor in anchors:
        candidates = claims.get(anchor.mac, ())
        if not candidates:
            raise ValueError(
                f"A{anchor.index} is not online or its UWB MAC/role is not known"
            )
        if len(candidates) > 1:
            raise ValueError(
                f"multiple online devices claim the configured A{anchor.index} MAC"
            )
        system_id, role = candidates[0]
        expected_role = (
            ROLE_ANCHOR_INITIATOR if anchor.index == 0 else ROLE_ANCHOR_RESPONDER
        )
        if role != expected_role:
            expected_name = "initiator" if anchor.index == 0 else "responder"
            raise ValueError(
                f"A{anchor.index} has UWB role {role}; expected {expected_name}"
            )
        resolved[anchor.index] = system_id
    return resolved


async def _wait_for_responder_summaries(
    ext: "RtlsExtension",
    responder_system_ids: dict[int, int],
    *,
    newer_than: float,
    timeout: float,
) -> tuple[DeviceTwrSummary, ...]:
    """Collect one post-request generation per responder with bounded skew."""
    deadline = time.monotonic() + timeout
    latest: dict[int, DeviceTwrSummary] = {}
    skew_s: float | None = None

    while True:
        changed = ext._twr_summary_changed
        now = time.monotonic()
        latest = {
            index: summary
            for index, system_id in responder_system_ids.items()
            if (summary := ext._twr_summaries.get(system_id)) is not None
            and summary.received_at >= newer_than
            and now - summary.received_at <= SUMMARY_FRESHNESS_S
        }
        if len(latest) == len(responder_system_ids):
            stamps = [summary.received_at for summary in latest.values()]
            skew_s = max(stamps) - min(stamps)
            if skew_s <= MAX_CAPTURE_SKEW_S:
                return tuple(latest[index] for index in sorted(latest))

        remaining = deadline - now
        if remaining <= 0:
            break
        with trio.move_on_after(remaining) as scope:
            await changed.wait()
        if scope.cancelled_caught:
            break

    missing = sorted(set(responder_system_ids) - set(latest))
    if missing:
        raise ValueError(
            "no fresh rolling TWR summary from "
            + ", ".join(f"A{index}" for index in missing)
            + f" within {timeout:.1f} seconds"
        )
    assert skew_s is not None
    raise ValueError(
        f"responder summary skew is {skew_s:.2f} seconds; "
        f"maximum is {MAX_CAPTURE_SKEW_S:.2f} seconds"
    )


def _resolve_observations(
    summaries: Sequence[DeviceTwrSummary],
    anchors: Sequence[_Anchor],
) -> tuple[tuple[_CaptureSource, ...], tuple[RangeObservation, ...]]:
    """Validate seven responder-owned A0 spokes and normalize for the solver."""
    a0 = anchors[0]
    sources: list[_CaptureSource] = []
    observations: list[RangeObservation] = []

    for anchor, summary in zip(anchors[1:], summaries, strict=True):
        if summary.version != SUMMARY_PROTOCOL_VERSION:
            raise ValueError(
                f"A{anchor.index} rolling-summary protocol version "
                f"{summary.version} is unsupported"
            )
        if summary.valid_mask != 0x01:
            raise ValueError(
                f"A{anchor.index} summary has peer mask "
                f"0x{summary.valid_mask:02x}; expected only A0"
            )
        if len(summary.ranges) != 1 or summary.ranges[0].peer_mac != a0.mac:
            raise ValueError(
                f"A{anchor.index} summary does not contain exactly its A0 range"
            )
        measured = summary.ranges[0]
        if measured.count < MIN_SAMPLES_PER_PAIR:
            raise ValueError(
                f"A{anchor.index} summary has {measured.count} samples; "
                f"minimum is {MIN_SAMPLES_PER_PAIR}"
            )
        sources.append(_CaptureSource(anchor=anchor, summary=summary))
        observations.append(
            RangeObservation(
                anchor_index=anchor.index,
                peer_mac=anchor.mac,
                distance_m=measured.distance_m,
                mad_m=measured.mad_m,
                count=measured.count,
            )
        )

    return tuple(sources), tuple(observations)


def _new_capture(
    ext: "RtlsExtension",
    sources: tuple[_CaptureSource, ...],
    observations: tuple[RangeObservation, ...],
    participant_system_ids: frozenset[int],
) -> _CalibrationCapture:
    capture = _CalibrationCapture(
        capture_id=ext._next_geo_capture_id,
        sources=sources,
        observations=observations,
        participant_system_ids=participant_system_ids,
    )
    ext._next_geo_capture_id += 1
    return capture


def _apply_geometry(
    reference: dict[str, Any],
    anchors: Sequence[_Anchor],
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
        position = positions[anchor.index]
        payload[f"UWB_AN{anchor.index}_X"] = position["xM"]
        payload[f"UWB_AN{anchor.index}_Y"] = position["yM"]
        payload[f"UWB_AN{anchor.index}_Z"] = position["zM"]
        payload[f"UWB_AN{anchor.index}_MAC"] = anchor.mac
        bias_name = f"UWB_AN{anchor.index}_BIAS_M"
        if bias_name in reference:
            payload[bias_name] = reference[bias_name]
    return payload


def _capture_json(capture: _CalibrationCapture) -> dict[str, Any]:
    now = time.monotonic()
    stamps = [source.summary.received_at for source in capture.sources]
    return {
        "captureId": capture.capture_id,
        "version": SUMMARY_PROTOCOL_VERSION,
        "validMask": 0xFE,
        "ageMs": round(max(now - stamp for stamp in stamps) * 1000),
        "maxSkewMs": round((max(stamps) - min(stamps)) * 1000),
        "sources": [
            {
                "anchorIndex": source.anchor.index,
                "anchorMac": source.anchor.mac,
                "systemId": source.summary.system_id,
                "sequence": source.summary.sequence,
                "timeBootMs": source.summary.time_boot_ms,
                "ageMs": round(max(0.0, now - source.summary.received_at) * 1000),
            }
            for source in capture.sources
        ],
        "ranges": [
            {
                "anchorIndex": item.anchor_index,
                "peerMac": item.peer_mac,
                "distanceM": round(item.distance_m, 5),
                "madM": round(item.mad_m, 5),
                "count": item.count,
            }
            for item in capture.observations
        ],
    }


async def run_fit(
    ext: "RtlsExtension",
    *,
    mode: str,
    cell: str | None = None,
    capture_id: int | None = None,
    timeout: float = SUMMARY_WAIT_TIMEOUT_S,
) -> dict[str, Any]:
    """Run a strict fit on a distributed capture or refine the pinned one."""
    if mode not in ("strict", "refined"):
        raise ValueError(f"Invalid fit mode: {mode!r} (expected strict or refined)")
    timeout = max(0.1, min(float(timeout), SUMMARY_WAIT_TIMEOUT_S))

    if mode == "strict":
        reference, cell = get_canonical(ext, cell)
        anchors = _configured_anchors(reference)
        devices = _resolve_anchor_devices(ext, anchors)
        requested_at = time.monotonic()
        summaries = await _wait_for_responder_summaries(
            ext,
            {index: devices[index] for index in range(1, len(anchors))},
            newer_than=requested_at,
            timeout=timeout,
        )
        sources, observations = _resolve_observations(summaries, anchors)
        capture = _new_capture(
            ext,
            sources,
            observations,
            participant_system_ids=frozenset(devices.values()),
        )
        strict = fit_strict(observations)
        session = _FitSession(capture, strict, reference, cell)
        ext._geo_fit_session = session
        refined = None
    else:
        session = ext._geo_fit_session
        if session is None:
            raise ValueError("run a strict fit first to pin a calibration capture")
        if capture_id is None:
            raise ValueError("captureId is required for a refined fit")
        if int(capture_id) != session.capture.capture_id:
            raise ValueError(
                "the requested capture is no longer the pinned calibration snapshot"
            )
        capture = session.capture
        observations = capture.observations
        strict = session.strict
        reference, cell = session.reference, session.cell
        anchors = _configured_anchors(reference)
        refined = fit_refined(observations, strict=strict)

    selected = strict if mode == "strict" else refined
    assert selected is not None
    selected_model = selected.model if selected.accepted else None
    payload = (
        _apply_geometry(reference, anchors, selected) if selected.accepted else None
    )
    body: dict[str, Any] = {
        "type": "X-RTLS-GEO",
        "op": "fit",
        "mode": mode,
        "cell": cell,
        "summary": _capture_json(capture),
        "strict": strict.json(),
        "refined": refined.json() if refined is not None else None,
        "selectedModel": selected_model,
        "applyGeometry": payload,
    }
    if refined is not None:
        noise_floor = max(0.01, float(median(item.mad_m for item in observations)))
        improvement = strict.rms_m - refined.rms_m
        body["comparison"] = {
            "rmsImprovementM": round(improvement, 5),
            "noiseFloorM": round(noise_floor, 5),
            "meaningfulImprovement": improvement > noise_floor,
        }
    return body
