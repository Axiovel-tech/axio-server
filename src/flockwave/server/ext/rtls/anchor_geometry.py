"""Pure four-tripod anchor-geometry fitting.

The fitters in this module know nothing about MAVLink, devices, server state,
or the control protocol.  They consume the seven A0-spoke summaries and
produce an absolute A0-origin NED geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, degrees, pi, radians, sin, sqrt
from statistics import median
from typing import Any, Callable, Sequence

import numpy as np

__all__ = (
    "FitResult",
    "RangeObservation",
    "fit_refined",
    "fit_strict",
)

ANCHOR_COUNT = 8
ABSOLUTE_RESIDUAL_LIMIT_M = 0.15
DIMENSION_MAX_M = 500.0
DIMENSION_MIN_M = 0.1
HEIGHT_MAX_M = 100.0
REFINED_ANGLE_LIMIT_DEG = 5.0
REFINED_DIMENSION_DELTA_M = 0.25
REFINED_DIMENSION_DELTA_RATIO = 0.02
WEIGHT_MAD_FLOOR_M = 0.005
HUBER_DELTA_M = 0.15
LM_MAX_ITERATIONS = 80
LM_TOLERANCE = 1e-10


@dataclass(frozen=True)
class RangeObservation:
    """One robust A0-to-anchor range summary."""

    anchor_index: int
    peer_mac: int
    distance_m: float
    mad_m: float
    count: int


@dataclass(frozen=True)
class FitResult:
    """Transport-independent result of one geometry model."""

    model: str
    accepted: bool
    parameters: dict[str, float]
    anchors: tuple[dict[str, Any], ...]
    rms_m: float
    weighted_objective: float
    residuals: tuple[dict[str, Any], ...]
    reasons: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def json(self) -> dict[str, Any]:
        """Return the control-protocol representation."""
        return {
            "model": self.model,
            "accepted": self.accepted,
            "parameters": {
                key: round(float(value), 5)
                for key, value in self.parameters.items()
            },
            "anchors": list(self.anchors),
            "rmsM": round(self.rms_m, 5),
            "weightedObjective": round(self.weighted_objective, 6),
            "residuals": list(self.residuals),
            "reasons": list(self.reasons),
            "warnings": list(self.warnings),
        }


def _validate(observations: Sequence[RangeObservation]) -> tuple[RangeObservation, ...]:
    by_index = {item.anchor_index: item for item in observations}
    if set(by_index) != set(range(1, ANCHOR_COUNT)):
        raise ValueError("the fit requires exactly the seven A0 spokes (A1–A7)")
    ordered = tuple(by_index[index] for index in range(1, ANCHOR_COUNT))
    if any(
        item.distance_m <= 0
        or item.mad_m < 0
        or item.count <= 0
        for item in ordered
    ):
        raise ValueError("range summaries contain invalid distance or quality values")
    return ordered


def _weights(observations: Sequence[RangeObservation]) -> np.ndarray:
    # Approximate standard error of the robust aggregate.  Normalize the
    # weights so the objective remains readable across sample cadences.
    raw = np.array(
        [
            sqrt(item.count) / max(item.mad_m, WEIGHT_MAD_FLOOR_M)
            for item in observations
        ],
        dtype=float,
    )
    return raw / max(float(median(raw)), 1e-9)


def _huber_pseudo_residual(values: np.ndarray) -> np.ndarray:
    absolute = np.abs(values)
    loss = np.where(
        absolute <= HUBER_DELTA_M,
        0.5 * values**2,
        HUBER_DELTA_M * (absolute - 0.5 * HUBER_DELTA_M),
    )
    return np.sign(values) * np.sqrt(2.0 * loss)


def _solve(
    residual_fn: Callable[[np.ndarray], np.ndarray],
    initial: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    project: Callable[[np.ndarray], np.ndarray] | None = None,
) -> np.ndarray:
    """Small bounded Levenberg–Marquardt solver for the 3/6 parameter fits."""
    theta = np.clip(initial.astype(float), lower, upper)
    if project is not None:
        theta = project(theta)
    damping = 1e-3
    cost = float(np.sum(residual_fn(theta) ** 2))
    for _ in range(LM_MAX_ITERATIONS):
        residual = residual_fn(theta)
        jacobian = np.empty((residual.size, theta.size))
        for index in range(theta.size):
            step = max(1e-6, 1e-6 * abs(theta[index]))
            probe = theta.copy()
            probe[index] += step
            if project is not None:
                probe = project(probe)
            jacobian[:, index] = (residual_fn(probe) - residual) / step
        jtj = jacobian.T @ jacobian
        jtr = jacobian.T @ residual
        improved = False
        delta = np.zeros_like(theta)
        for _ in range(8):
            try:
                delta = np.linalg.solve(
                    jtj
                    + damping
                    * np.diag(np.maximum(np.diag(jtj), 1e-12)),
                    -jtr,
                )
            except np.linalg.LinAlgError:
                damping *= 10
                continue
            candidate = np.clip(theta + delta, lower, upper)
            if project is not None:
                candidate = project(candidate)
            candidate_cost = float(np.sum(residual_fn(candidate) ** 2))
            if candidate_cost < cost:
                theta, cost = candidate, candidate_cost
                damping = max(damping / 10, 1e-9)
                improved = True
                break
            damping *= 10
        if not improved or float(np.linalg.norm(delta)) < LM_TOLERANCE:
            break
    return theta


def _strict_positions(theta: np.ndarray) -> np.ndarray:
    length, width, height = theta
    return np.array(
        [
            (0.0, 0.0, 0.0),
            (length, 0.0, 0.0),
            (0.0, width, 0.0),
            (length, width, 0.0),
            (0.0, 0.0, -height),
            (length, 0.0, -height),
            (0.0, width, -height),
            (length, width, -height),
        ],
        dtype=float,
    )


def _refined_positions(theta: np.ndarray) -> np.ndarray:
    bottom_length, bottom_width, top_length, top_width, height, angle = theta
    angle_cos, angle_sin = cos(angle), sin(angle)
    return np.array(
        [
            (0.0, 0.0, 0.0),
            (bottom_length, 0.0, 0.0),
            (bottom_width * angle_cos, bottom_width * angle_sin, 0.0),
            (
                bottom_length + bottom_width * angle_cos,
                bottom_width * angle_sin,
                0.0,
            ),
            (0.0, 0.0, -height),
            (top_length, 0.0, -height),
            (top_width * angle_cos, top_width * angle_sin, -height),
            (
                top_length + top_width * angle_cos,
                top_width * angle_sin,
                -height,
            ),
        ],
        dtype=float,
    )


def _data_residuals(
    positions: np.ndarray, observations: Sequence[RangeObservation]
) -> np.ndarray:
    return np.array(
        [
            float(np.linalg.norm(positions[item.anchor_index]))
            - item.distance_m
            for item in observations
        ],
        dtype=float,
    )


def _finish(
    model: str,
    parameters: dict[str, float],
    positions: np.ndarray,
    observations: Sequence[RangeObservation],
    weights: np.ndarray,
    objective_residuals: np.ndarray,
    *,
    extra_reasons: Sequence[str] = (),
    warnings: Sequence[str] = (),
) -> FitResult:
    residual_values = _data_residuals(positions, observations)
    rms = float(np.sqrt(np.mean(residual_values**2)))
    reasons = list(extra_reasons)
    worst = float(np.max(np.abs(residual_values)))
    if worst > ABSOLUTE_RESIDUAL_LIMIT_M:
        reasons.append(
            f"worst spoke residual {worst:.3f} m exceeds "
            f"{ABSOLUTE_RESIDUAL_LIMIT_M:.3f} m"
        )

    anchors = tuple(
        {
            "index": index,
            "x": round(float(position[0]), 5),
            "y": round(float(position[1]), 5),
            "z": round(float(position[2]), 5),
        }
        for index, position in enumerate(positions)
    )
    residuals = tuple(
        {
            "anchorIndex": item.anchor_index,
            "peerMac": item.peer_mac,
            "measuredM": round(item.distance_m, 5),
            "predictedM": round(float(item.distance_m + residual), 5),
            "residualM": round(float(residual), 5),
            "madM": round(item.mad_m, 5),
            "count": item.count,
            "weight": round(float(weight), 5),
        }
        for item, residual, weight in zip(
            observations, residual_values, weights, strict=True
        )
    )
    return FitResult(
        model=model,
        accepted=not reasons,
        parameters=parameters,
        anchors=anchors,
        rms_m=rms,
        weighted_objective=float(np.mean(objective_residuals**2)),
        residuals=residuals,
        reasons=tuple(reasons),
        warnings=tuple(warnings),
    )


def fit_strict(observations: Sequence[RangeObservation]) -> FitResult:
    """Fit congruent, perpendicular lower/upper rectangles."""
    ordered = _validate(observations)
    weights = _weights(ordered)
    distances = np.array([item.distance_m for item in ordered])
    initial = np.array([distances[0], distances[1], distances[3]])
    lower = np.array([DIMENSION_MIN_M, DIMENSION_MIN_M, DIMENSION_MIN_M])
    upper = np.array([DIMENSION_MAX_M, DIMENSION_MAX_M, HEIGHT_MAX_M])

    def objective(theta: np.ndarray) -> np.ndarray:
        return _huber_pseudo_residual(
            _data_residuals(_strict_positions(theta), ordered) * weights
        )

    theta = _solve(objective, initial, lower, upper)
    positions = _strict_positions(theta)
    return _finish(
        "strict",
        {
            "lengthM": float(theta[0]),
            "widthM": float(theta[1]),
            "heightM": float(theta[2]),
        },
        positions,
        ordered,
        weights,
        objective(theta),
    )


def fit_refined(
    observations: Sequence[RangeObservation],
    *,
    strict: FitResult | None = None,
) -> FitResult:
    """Fit aligned upper/lower parallelograms with one shared skew angle."""
    ordered = _validate(observations)
    strict = strict or fit_strict(ordered)
    weights = _weights(ordered)
    distance = {item.anchor_index: item.distance_m for item in ordered}
    height = strict.parameters["heightM"]
    initial = np.array(
        [
            strict.parameters["lengthM"],
            strict.parameters["widthM"],
            sqrt(max(distance[5] ** 2 - height**2, DIMENSION_MIN_M**2)),
            sqrt(max(distance[6] ** 2 - height**2, DIMENSION_MIN_M**2)),
            height,
            pi / 2,
        ],
        dtype=float,
    )
    angle_delta = radians(REFINED_ANGLE_LIMIT_DEG)
    lower = np.array(
        [
            DIMENSION_MIN_M,
            DIMENSION_MIN_M,
            DIMENSION_MIN_M,
            DIMENSION_MIN_M,
            DIMENSION_MIN_M,
            pi / 2 - angle_delta,
        ]
    )
    upper = np.array(
        [
            DIMENSION_MAX_M,
            DIMENSION_MAX_M,
            DIMENSION_MAX_M,
            DIMENSION_MAX_M,
            HEIGHT_MAX_M,
            pi / 2 + angle_delta,
        ]
    )

    def project(theta: np.ndarray) -> np.ndarray:
        value = np.clip(theta, lower, upper)
        for bottom_index, top_index in ((0, 2), (1, 3)):
            bottom = value[bottom_index]
            delta = max(
                REFINED_DIMENSION_DELTA_M,
                REFINED_DIMENSION_DELTA_RATIO * bottom,
            )
            value[top_index] = np.clip(
                value[top_index], bottom - delta, bottom + delta
            )
        return value

    def objective(theta: np.ndarray) -> np.ndarray:
        data = _huber_pseudo_residual(
            _data_residuals(_refined_positions(theta), ordered) * weights
        )
        # Soft construction priors. They choose the least-deformed geometry
        # supported by the ranges without pretending to be measurements.
        priors = np.array(
            [
                0.01 * (theta[2] - theta[0]) / REFINED_DIMENSION_DELTA_M,
                0.01 * (theta[3] - theta[1]) / REFINED_DIMENSION_DELTA_M,
                0.01 * (theta[5] - pi / 2) / angle_delta,
            ]
        )
        return np.concatenate((data, priors))

    theta = _solve(objective, initial, lower, upper, project=project)
    positions = _refined_positions(theta)
    refined_rms = float(
        np.sqrt(np.mean(_data_residuals(positions, ordered) ** 2))
    )
    improvement = strict.rms_m - refined_rms
    noise_floor = max(float(median(item.mad_m for item in ordered)), 0.01)
    reasons: list[str] = []
    warnings: list[str] = []
    if improvement <= noise_floor:
        reasons.append(
            f"RMS improvement {improvement:.3f} m does not exceed "
            f"the {noise_floor:.3f} m measurement noise floor"
        )

    angle_deg = degrees(theta[5])
    if abs(abs(angle_deg - 90.0) - REFINED_ANGLE_LIMIT_DEG) < 0.25:
        warnings.append("corner angle is close to its safety bound")
    for label, bottom, top in (
        ("length", theta[0], theta[2]),
        ("width", theta[1], theta[3]),
    ):
        limit = max(
            REFINED_DIMENSION_DELTA_M,
            REFINED_DIMENSION_DELTA_RATIO * bottom,
        )
        if abs(top - bottom) >= 0.9 * limit:
            warnings.append(f"upper/lower {label} difference is close to its bound")

    return _finish(
        "refined",
        {
            "bottomLengthM": float(theta[0]),
            "bottomWidthM": float(theta[1]),
            "topLengthM": float(theta[2]),
            "topWidthM": float(theta[3]),
            "heightM": float(theta[4]),
            "angleDeg": angle_deg,
        },
        positions,
        ordered,
        weights,
        objective(theta),
        extra_reasons=reasons,
        warnings=warnings,
    )
