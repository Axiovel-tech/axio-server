"""TWR capture + rectangular geometry fit (X-RTLS-GEO capture/fit).

The daily anchor-setup problem: tripods go up in roughly the surveyed
spots, but "roughly" is centimeters of error the UWB solver then bakes
into every position. The anchors continuously range each other (TWR),
so the geometry that the anchors actually stand in is measurable:

- ``capture`` collects a window of inter-anchor TWR samples (the
  steady-state harvest keeps only the latest value per pair; a fit
  needs a robust aggregate over many);
- ``fit`` aggregates the window (median + MAD outlier rejection per
  pair), then solves two least-squares problems against the cell's
  RECTANGULAR shape prior — the site is two stacked/offset rectangles
  (or one rectangle with per-anchor heights):

  1. the RIGID fit optimizes only the shape parameters (width, length,
     layer heights, upper-layer offset), yielding the best geometry
     that keeps the assumed shape;
  2. the RELAXED fit frees every anchor coordinate inside a small box
     around the rigid solution, regularized toward it.

  One solve, two readings: ``relaxed - rigid`` per anchor is the "move
  this tripod" suggestion; the relaxed positions are the best geometry
  achievable WITHOUT moving anything (write it with the sync op's
  explicit ``geometry`` payload).

Distances constrain only the SHAPE, never the frame, so the fitted
shape is aligned back onto the configured layout (2-D Kabsch over the
plan view plus a mean height shift): the operator's cell frame — and
everything referencing it — stays put.

Solvers are plain numpy Levenberg–Marquardt with numerical Jacobians:
six shape parameters (or 3N ≤ 24 relaxed coordinates) against ≤ 28
pair measurements need no scipy dependency on a show machine.
"""

from __future__ import annotations

import statistics
from typing import TYPE_CHECKING, Any, Mapping, Optional

import numpy as np

from .extension import _decoded_device_params
from .geometry import _resolve_reference

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("run_capture", "run_capture_status", "run_fit", "on_twr_sample")

#: capture window bounds, seconds
CAPTURE_DEFAULT_S = 20.0
CAPTURE_MAX_S = 120.0

#: per-pair sample cap: at the firmware's ranging cadence a runaway
#: capture must not grow without bound
MAX_SAMPLES_PER_PAIR = 4096

#: minimum aggregated samples for a pair to count as measured
MIN_SAMPLES_PER_PAIR = 3

#: samples farther than this many (scaled) MADs from the median are dropped
MAD_K = 3.0

#: default / maximum half-width of the relaxed fit's box around the rigid
#: solution, meters
RELAX_MARGIN_DEFAULT_M = 0.10
RELAX_MARGIN_MAX_M = 0.50

#: regularization weight pulling relaxed coordinates toward the rigid
#: shape (in residual units per meter of deviation): small enough that a
#: real measurement wins, large enough that unconstrained directions
#: (e.g. a poorly-covered anchor) stay put
RELAX_REGULARIZATION = 0.05

#: two anchors are on the same layer when their configured heights are
#: within this many meters
LAYER_SPLIT_M = 0.30

#: Levenberg–Marquardt iteration cap and convergence threshold
LM_MAX_ITERATIONS = 60
LM_TOLERANCE = 1e-10


# ---- capture -------------------------------------------------------------


def on_twr_sample(
    ext: "RtlsExtension",
    system_id: int,
    peer_mac: int,
    distance_m: float,
    now: float,
) -> None:
    """Appends one TWR sample to the running capture window (no-op when
    no capture is active). Called from the extension's TWR event hook."""
    capture = ext._geo_capture
    if capture is None or now > capture["until"]:
        return
    samples = capture["samples"].setdefault((system_id, peer_mac), [])
    if len(samples) < MAX_SAMPLES_PER_PAIR:
        samples.append(float(distance_m))


def run_capture(ext: "RtlsExtension", duration: float, now: float) -> dict:
    """(Re)starts a TWR capture window; returns the capture status body.
    Restarting is deliberate: a stale half-window from an earlier session
    must not leak into a new fit."""
    duration = max(1.0, min(float(duration), CAPTURE_MAX_S))
    ext._geo_capture = {
        "started": now,
        "until": now + duration,
        "samples": {},
    }
    return run_capture_status(ext, now)


def run_capture_status(ext: "RtlsExtension", now: float) -> dict:
    capture = ext._geo_capture
    if capture is None:
        return {
            "type": "X-RTLS-GEO",
            "op": "capture-status",
            "running": False,
        }
    samples = capture["samples"]
    counts = [len(v) for v in samples.values()]
    return {
        "type": "X-RTLS-GEO",
        "op": "capture-status",
        "running": now < capture["until"],
        "elapsed": round(max(0.0, now - capture["started"]), 1),
        "duration": round(capture["until"] - capture["started"], 1),
        "pairs": len(samples),
        "samplesTotal": int(sum(counts)),
        "samplesPerPairMin": int(min(counts)) if counts else 0,
    }


def _robust_aggregate(samples: list[float]) -> Optional[dict[str, Any]]:
    """Median + MAD outlier rejection over one pair's samples; ``None``
    when too few samples survive."""
    if len(samples) < MIN_SAMPLES_PER_PAIR:
        return None
    median = statistics.median(samples)
    mad = statistics.median([abs(s - median) for s in samples])
    scale = max(1.4826 * mad, 0.005)  # never let MAD=0 nuke everything
    kept = [s for s in samples if abs(s - median) <= MAD_K * scale]
    if len(kept) < MIN_SAMPLES_PER_PAIR:
        return None
    return {
        "distanceM": statistics.median(kept),
        "madM": mad,
        "count": len(kept),
        "dropped": len(samples) - len(kept),
    }


def _mac_of(ext: "RtlsExtension", system_id: int) -> Optional[int]:
    device = ext._require_protocol().devices.get(system_id)
    if device is None:
        return None
    mac = _decoded_device_params(device).get("UWB_MAC")
    try:
        return int(mac) if mac is not None else None
    except (TypeError, ValueError):
        return None


def aggregate_capture(
    ext: "RtlsExtension",
) -> dict[tuple[int, int], dict[str, Any]]:
    """Aggregates the capture window into canonical per-MAC-pair
    measurements: (min_mac, max_mac) -> {distanceM, madM, count, dropped}.
    Both directions of a pair merge into one measurement."""
    capture = ext._geo_capture
    if capture is None:
        return {}
    by_pair: dict[tuple[int, int], list[float]] = {}
    for (system_id, peer_mac), samples in capture["samples"].items():
        own_mac = _mac_of(ext, system_id)
        if own_mac is None or own_mac == peer_mac:
            continue
        key = (min(own_mac, peer_mac), max(own_mac, peer_mac))
        by_pair.setdefault(key, []).extend(samples)
    out: dict[tuple[int, int], dict[str, Any]] = {}
    for key, samples in by_pair.items():
        aggregated = _robust_aggregate(samples)
        if aggregated is not None:
            out[key] = aggregated
    return out


# ---- shape model ---------------------------------------------------------


class _ShapeModel:
    """The rectangular shape prior, derived from the CONFIGURED layout:
    every anchor is assigned a plan-view corner (u, v ∈ {0, 1}) and a
    layer (0 = low, 1 = high). Parameters: [W, L, z_low, (z_high),
    (ox, oy)] — the layer-height and upper-layer-offset parameters exist
    only when two layers exist."""

    def __init__(self, anchors: list[dict[str, Any]]):
        xs = [a["x"] for a in anchors]
        ys = [a["y"] for a in anchors]
        zs = [a["z"] for a in anchors]
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        if x1 - x0 < 0.5 or y1 - y0 < 0.5:
            raise ValueError(
                "configured anchor layout is not a rectangle (plan-view "
                "extent below 0.5 m)"
            )

        z_sorted = sorted(zs)
        self.two_layers = (z_sorted[-1] - z_sorted[0]) > LAYER_SPLIT_M
        z_mid = (z_sorted[0] + z_sorted[-1]) / 2.0

        self.anchors = anchors
        self.assignment: list[tuple[int, int, int]] = []
        corners_used: set[tuple[int, int, int]] = set()
        for anchor in anchors:
            u = 0 if abs(anchor["x"] - x0) < abs(anchor["x"] - x1) else 1
            v = 0 if abs(anchor["y"] - y0) < abs(anchor["y"] - y1) else 1
            layer = (
                1 if self.two_layers and anchor["z"] > z_mid else 0
            )
            key = (u, v, layer)
            if key in corners_used:
                raise ValueError(
                    "configured anchor layout does not resolve to distinct "
                    "rectangle corners (two anchors share corner "
                    f"u={u}, v={v}, layer={layer}) — the rectangular shape "
                    "prior cannot describe this site"
                )
            corners_used.add(key)
            self.assignment.append(key)

        low_zs = [
            a["z"]
            for a, key in zip(anchors, self.assignment)
            if key[2] == 0
        ]
        high_zs = [
            a["z"]
            for a, key in zip(anchors, self.assignment)
            if key[2] == 1
        ]
        theta = [x1 - x0, y1 - y0, statistics.mean(low_zs)]
        if self.two_layers:
            theta.append(statistics.mean(high_zs) if high_zs else theta[2])
            theta.extend([0.0, 0.0])  # upper-layer plan offset ox, oy
        self.theta0 = np.array(theta, dtype=float)

    def positions(self, theta: np.ndarray) -> np.ndarray:
        width, length, z_low = theta[0], theta[1], theta[2]
        if self.two_layers:
            z_high, ox, oy = theta[3], theta[4], theta[5]
        else:
            z_high, ox, oy = z_low, 0.0, 0.0
        out = np.empty((len(self.assignment), 3), dtype=float)
        for i, (u, v, layer) in enumerate(self.assignment):
            out[i, 0] = u * width + (ox if layer else 0.0)
            out[i, 1] = v * length + (oy if layer else 0.0)
            out[i, 2] = z_high if layer else z_low
        return out


def _kabsch_align(
    fitted: np.ndarray, reference: np.ndarray
) -> np.ndarray:
    """Aligns the fitted positions onto the reference layout with a
    plan-view rotation + translation and a mean height shift (distances
    carry no frame, so the operator's frame must be restored)."""
    f_xy = fitted[:, :2]
    r_xy = reference[:, :2]
    f_c = f_xy - f_xy.mean(axis=0)
    r_c = r_xy - r_xy.mean(axis=0)
    h = f_c.T @ r_c
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, d]) @ u.T
    aligned = fitted.copy()
    aligned[:, :2] = (rot @ f_c.T).T + r_xy.mean(axis=0)
    aligned[:, 2] += reference[:, 2].mean() - fitted[:, 2].mean()
    return aligned


def _levenberg_marquardt(
    residual_fn, theta0: np.ndarray, *, bounds=None
) -> np.ndarray:
    """Tiny LM with numerical Jacobians and optional box clamping —
    six-ish parameters against ≤ 28 residuals need nothing bigger."""
    theta = theta0.astype(float).copy()
    lam = 1e-3
    cost = float(np.sum(residual_fn(theta) ** 2))
    for _ in range(LM_MAX_ITERATIONS):
        r = residual_fn(theta)
        jac = np.empty((r.size, theta.size))
        for j in range(theta.size):
            step = max(1e-6, 1e-6 * abs(theta[j]))
            probe = theta.copy()
            probe[j] += step
            jac[:, j] = (residual_fn(probe) - r) / step
        jtj = jac.T @ jac
        jtr = jac.T @ r
        improved = False
        for _ in range(8):
            try:
                delta = np.linalg.solve(
                    jtj + lam * np.diag(np.maximum(np.diag(jtj), 1e-12)),
                    -jtr,
                )
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            candidate = theta + delta
            if bounds is not None:
                candidate = np.clip(candidate, bounds[0], bounds[1])
            c = float(np.sum(residual_fn(candidate) ** 2))
            if c < cost:
                theta, cost = candidate, c
                lam = max(lam / 10, 1e-9)
                improved = True
                break
            lam *= 10
        if not improved or float(np.linalg.norm(delta)) < LM_TOLERANCE:
            break
    return theta


# ---- fit -----------------------------------------------------------------


def _pair_residuals(
    positions: np.ndarray,
    index_of_mac: Mapping[int, int],
    measurements: Mapping[tuple[int, int], dict[str, Any]],
) -> np.ndarray:
    out = []
    for (mac_a, mac_b), m in sorted(measurements.items()):
        ia = index_of_mac.get(mac_a)
        ib = index_of_mac.get(mac_b)
        if ia is None or ib is None:
            continue
        predicted = float(np.linalg.norm(positions[ia] - positions[ib]))
        out.append(predicted - m["distanceM"])
    return np.array(out, dtype=float)


def run_fit(
    ext: "RtlsExtension",
    *,
    margin: float = RELAX_MARGIN_DEFAULT_M,
) -> dict[str, Any]:
    """Aggregates the capture window and runs the rigid + relaxed fits
    against the current (majority-reference) cell geometry. Raises
    ValueError with a client-presentable reason when the capture or the
    layout cannot support a fit."""
    margin = max(0.01, min(float(margin), RELAX_MARGIN_MAX_M))

    measurements = aggregate_capture(ext)
    if not measurements:
        raise ValueError(
            "no usable TWR measurements captured — run a capture first "
            "(and check that the anchors hear each other)"
        )

    _, ref_subset, cell_id = _resolve_reference(ext, None, None)
    count = int(ref_subset["UWB_AN_COUNT"])
    anchors = []
    for index in range(count):
        anchors.append(
            {
                "index": index,
                "mac": int(ref_subset[f"UWB_AN{index}_MAC"]),
                "x": float(ref_subset[f"UWB_AN{index}_X"]),
                "y": float(ref_subset[f"UWB_AN{index}_Y"]),
                "z": float(ref_subset[f"UWB_AN{index}_Z"]),
            }
        )
    if len(anchors) < 4:
        raise ValueError(
            f"the rectangular fit needs at least 4 anchors ({len(anchors)} "
            "configured)"
        )
    index_of_mac = {a["mac"]: i for i, a in enumerate(anchors)}

    macs = set(index_of_mac)
    usable = {
        pair: m
        for pair, m in measurements.items()
        if pair[0] in macs and pair[1] in macs
    }
    expected_pairs = len(anchors) * (len(anchors) - 1) // 2
    missing_pairs = [
        [a["mac"], b["mac"]]
        for i, a in enumerate(anchors)
        for b in anchors[i + 1 :]
        if (min(a["mac"], b["mac"]), max(a["mac"], b["mac"])) not in usable
    ]
    if len(usable) < len(anchors):
        raise ValueError(
            f"only {len(usable)} of {expected_pairs} anchor pairs were "
            "measured — not enough ranging coverage for a fit"
        )

    configured = np.array(
        [[a["x"], a["y"], a["z"]] for a in anchors], dtype=float
    )
    model = _ShapeModel(anchors)

    def rigid_residuals(theta: np.ndarray) -> np.ndarray:
        return _pair_residuals(model.positions(theta), index_of_mac, usable)

    theta = _levenberg_marquardt(rigid_residuals, model.theta0)
    rigid_aligned = _kabsch_align(model.positions(theta), configured)
    rigid_r = _pair_residuals(rigid_aligned, index_of_mac, usable)
    rigid_rms = float(np.sqrt(np.mean(rigid_r**2))) if rigid_r.size else 0.0

    flat0 = rigid_aligned.reshape(-1)
    lower = flat0 - margin
    upper = flat0 + margin

    def relaxed_residuals(flat: np.ndarray) -> np.ndarray:
        positions = flat.reshape(-1, 3)
        pair_r = _pair_residuals(positions, index_of_mac, usable)
        reg = RELAX_REGULARIZATION * (flat - flat0)
        return np.concatenate([pair_r, reg])

    relaxed_flat = _levenberg_marquardt(
        relaxed_residuals, flat0, bounds=(lower, upper)
    )
    relaxed = relaxed_flat.reshape(-1, 3)
    relaxed_r = _pair_residuals(relaxed, index_of_mac, usable)
    relaxed_rms = (
        float(np.sqrt(np.mean(relaxed_r**2))) if relaxed_r.size else 0.0
    )

    def anchor_list(positions: np.ndarray) -> list[dict[str, Any]]:
        return [
            {
                "index": a["index"],
                "mac": a["mac"],
                "x": round(float(positions[i, 0]), 4),
                "y": round(float(positions[i, 1]), 4),
                "z": round(float(positions[i, 2]), 4),
            }
            for i, a in enumerate(anchors)
        ]

    residual_rows = []
    for (mac_a, mac_b), m in sorted(usable.items()):
        ia, ib = index_of_mac[mac_a], index_of_mac[mac_b]
        predicted = float(np.linalg.norm(relaxed[ia] - relaxed[ib]))
        residual_rows.append(
            {
                "macA": mac_a,
                "macB": mac_b,
                "measuredM": round(m["distanceM"], 4),
                "predictedM": round(predicted, 4),
                "residualM": round(predicted - m["distanceM"], 4),
                "samples": m["count"],
                "dropped": m["dropped"],
            }
        )

    moves = []
    for i, a in enumerate(anchors):
        delta = relaxed[i] - configured[i]
        dist = float(np.linalg.norm(delta))
        moves.append(
            {
                "index": a["index"],
                "mac": a["mac"],
                "dxM": round(float(delta[0]), 4),
                "dyM": round(float(delta[1]), 4),
                "dzM": round(float(delta[2]), 4),
                "distM": round(dist, 4),
            }
        )

    return {
        "type": "X-RTLS-GEO",
        "op": "fit",
        "cell": cell_id,
        "coverage": {
            "pairsMeasured": len(usable),
            "pairsExpected": expected_pairs,
            "missingPairs": missing_pairs,
        },
        "rigid": {
            "anchors": anchor_list(rigid_aligned),
            "rmsM": round(rigid_rms, 4),
        },
        "relaxed": {
            "anchors": anchor_list(relaxed),
            "rmsM": round(relaxed_rms, 4),
            "marginM": margin,
        },
        # relaxed - CONFIGURED: how far each tripod stands from where the
        # measurements say it is; also exactly the "move this tripod"
        # suggestion (or, applied as-is, the no-move geometry)
        "moves": moves,
        "residuals": residual_rows,
    }
