"""Anchor positions from surveyed pair ranges, and the commit gate.

A survey measures the range between every two anchors of a group, once in
each direction. ``fit_survey`` turns those ranges and the operator's frame
declaration into positions in the cell frame (north, east, down: up is
negative z), a report per anchor and per pair, and the blocks and warnings
that decide whether the result may be committed.

Ranges fix a layout only up to its mirror images, and a least-squares
descent from a wrong sketch can stop in a layout that is not the best one.
The fit therefore descends from several variants of its starting point and
lets the ranges choose; the placement and the frame only separate layouts
the ranges cannot tell apart.

Pure: numpy only, no device I/O and nothing of the server's runtime.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass
from itertools import combinations, product
from typing import Any

import numpy as np

from flockwave.server.errors import FlockwaveError

__all__ = (
    "AnchorFit",
    "DirectedRange",
    "Frame",
    "GateEntry",
    "PairFit",
    "SurveyFit",
    "SurveyFitError",
    "fit_survey",
    "frame_refusal",
)

#: a direction with fewer inliers or a smaller valid share is not used
MIN_INLIERS = 20
MIN_SHARE = 0.5

#: commit warnings, metres and share
RMS_WARNING_M = 0.05
PAIR_WARNING_M = 0.10
ASYMMETRY_WARNING_M = 0.03
SHARE_WARNING = 0.9

#: an anchor with a larger predicted uncertainty is not located, metres
SIGMA_BLOCK_M = 0.20

#: ranges are quantised to 1 cm, so no fit is trusted below this, metres
NOISE_FLOOR_M = 0.01
#: range noise assumed when the fit has no spare measurement to estimate it
NOISE_PRIOR_M = 0.03

#: two layouts closer than this at every anchor are one solution, metres
SAME_SOLUTION_M = 0.01

#: where a height nobody gave starts, above the origin's group, metres. A
#: start exactly level with the group has no height gradient to descend along.
START_HEIGHT_M = 1.0

MAX_ITERATIONS = 200
STEP_TOLERANCE_M = 1e-10
RANK_TOLERANCE = 1e-9

Point = tuple[float, float, float]


def _metres(value: float) -> float:
    """A length for a message: 0.1 mm, and never the minus zero a mirrored
    or rounded coordinate can be."""
    return round(value, 4) + 0.0


class SurveyFitError(FlockwaveError):
    """The fit cannot run; the message is the reason for the operator."""


@dataclass(frozen=True)
class DirectedRange:
    """What a responder measured to the initiator of one survey step."""

    range_m: float
    mad_m: float
    inliers: int
    share: float

    @property
    def usable(self) -> bool:
        return self.inliers >= MIN_INLIERS and self.share >= MIN_SHARE

    @property
    def json(self) -> dict[str, Any]:
        return {
            "rangeM": _metres(self.range_m),
            "madM": _metres(self.mad_m),
            "n": self.inliers,
            "share": round(self.share, 3),
        }


@dataclass(frozen=True)
class Frame:
    """The operator's declaration of the cell frame.

    ``origin`` sits at x = y = 0 and ``x_axis`` on +x. The anchors of one
    height group share their z, the group of ``origin`` is z = 0, and an
    anchor in no group has a z of its own. ``upper`` names anchors above the
    origin's group and ``y_positive`` one anchor at y > 0; they settle the
    mirror images.
    """

    origin: int
    x_axis: int
    height_groups: tuple[tuple[int, ...], ...] = ()
    upper: tuple[int, ...] = ()
    y_positive: int | None = None

    @classmethod
    def two_tiers(cls, anchors: Sequence[int]) -> Frame:
        """The standard cell from its anchors in slot order: the first four
        level with the origin, the rest above them, the third at y > 0."""
        if len(anchors) < 3:
            raise SurveyFitError("The standard cell needs at least three anchors")
        lower, upper = tuple(anchors[:4]), tuple(anchors[4:])
        return cls(
            origin=anchors[0],
            x_axis=anchors[1],
            height_groups=(lower, upper) if upper else (lower,),
            upper=upper,
            y_positive=anchors[2],
        )


@dataclass(frozen=True)
class AnchorFit:
    position: Point
    links: int
    residual_rms_m: float
    worst_pair_m: float
    #: ``None`` when the ranges leave the position undetermined
    sigma_m: float | None
    #: ``None`` when the anchor stores no position
    moved_m: float | None

    @property
    def json(self) -> dict[str, Any]:
        x, y, z = (_metres(value) for value in self.position)
        result: dict[str, Any] = {
            "x": x,
            "y": y,
            "z": z,
            "links": self.links,
            "residualRmsM": _metres(self.residual_rms_m),
            "worstPairM": _metres(self.worst_pair_m),
            "sigmaM": None if self.sigma_m is None else _metres(self.sigma_m),
        }
        if self.moved_m is not None:
            result["movedM"] = _metres(self.moved_m)
        return result


@dataclass(frozen=True)
class PairFit:
    """One pair the fit used. ``ab`` is the range ``b`` measured while ``a``
    initiated; the residual is the fitted distance minus ``range_m``."""

    a: int
    b: int
    range_m: float
    ab: DirectedRange | None
    ba: DirectedRange | None
    fitted_m: float
    residual_m: float

    @property
    def json(self) -> dict[str, Any]:
        return {
            "a": self.a,
            "b": self.b,
            "rangeM": _metres(self.range_m),
            "ab": None if self.ab is None else self.ab.json,
            "ba": None if self.ba is None else self.ba.json,
            "fittedM": _metres(self.fitted_m),
            "residualM": _metres(self.residual_m),
        }


@dataclass(frozen=True)
class GateEntry:
    code: str
    device: int | None
    peer: int | None
    detail: str

    @property
    def json(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "device": self.device,
            "peer": self.peer,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class SurveyFit:
    """The fit and its gate. A result with ``blocked`` entries cannot be
    committed; ``warnings`` need the operator's acknowledgement."""

    anchors: dict[int, AnchorFit]
    pairs: tuple[PairFit, ...]
    rms_m: float
    worst_m: float
    planar: bool
    notes: tuple[str, ...]
    blocked: tuple[GateEntry, ...]
    warnings: tuple[GateEntry, ...]

    @property
    def json(self) -> dict[str, Any]:
        """The ``pairs``, ``fit`` and ``gate`` members of the survey state."""
        return {
            "pairs": [pair.json for pair in self.pairs],
            "fit": {
                "rmsM": _metres(self.rms_m),
                "worstM": _metres(self.worst_m),
                "planar": self.planar,
                "anchors": {str(k): v.json for k, v in self.anchors.items()},
                "notes": list(self.notes),
            },
            "gate": {
                "blocked": [entry.json for entry in self.blocked],
                "warnings": [entry.json for entry in self.warnings],
            },
        }


def frame_refusal(frame: Frame, anchors: Sequence[int]) -> str | None:
    """Why the frame cannot be used for this group, or ``None``."""
    grouped = [anchor for group in frame.height_groups for anchor in group]
    named = [frame.origin, frame.x_axis, *grouped, *frame.upper]
    if frame.y_positive is not None:
        named.append(frame.y_positive)
    outside = sorted(set(named) - set(anchors))
    if outside:
        return f"The frame names anchor {outside[0]}, which is not in the survey"
    if len(set(anchors)) != len(anchors):
        return "An anchor is listed twice"
    if frame.origin == frame.x_axis:
        return "The origin and the +x anchor must be two anchors"
    if len(set(grouped)) != len(grouped):
        return "An anchor is in two height groups"
    level = next((g for g in frame.height_groups if frame.origin in g), ())
    if set(frame.upper) & {frame.origin, *level}:
        return "An upper anchor cannot be level with the origin"
    if frame.y_positive in (frame.origin, frame.x_axis):
        return "The anchor at y > 0 cannot be on the x axis"
    return None


@dataclass(frozen=True)
class _Solution:
    positions: np.ndarray
    rms: float


class _Problem:
    """The unknowns of one group and the residuals of its measured pairs.

    Every coordinate is either fixed at zero or equal to one unknown, so the
    flat positions are ``expand @ params``.
    """

    def __init__(
        self,
        anchors: tuple[int, ...],
        frame: Frame,
        ranges: dict[tuple[int, int], float],
    ):
        index = {anchor: i for i, anchor in enumerate(anchors)}
        self.count = len(anchors)
        self.origin = index[frame.origin]
        self.x_axis = index[frame.x_axis]
        self.upper = {index[anchor] for anchor in frame.upper}
        self.y_positive = None if frame.y_positive is None else index[frame.y_positive]

        groups = [[index[anchor] for anchor in group] for group in frame.height_groups]
        self.level = next((g for g in groups if self.origin in g), [self.origin])
        grouped = {i for group in groups for i in group} | {self.origin}
        self.free_heights = [g for g in groups if self.origin not in g and g]
        self.free_heights += [[i] for i in range(self.count) if i not in grouped]
        self.planar = len(self.level) == self.count

        self.expand = self._expansion()

        self.pairs = list(ranges)
        self.first = np.array([index[a] for a, _ in self.pairs], dtype=int)
        self.second = np.array([index[b] for _, b in self.pairs], dtype=int)
        self.ranges = np.array([ranges[pair] for pair in self.pairs], dtype=float)

    def _expansion(self) -> np.ndarray:
        """One column per unknown: x of every anchor but the origin, y of
        every anchor off the x axis, one z per free height."""
        columns = [[3 * i] for i in range(self.count) if i != self.origin]
        columns += [
            [3 * i + 1]
            for i in range(self.count)
            if i not in (self.origin, self.x_axis)
        ]
        columns += [[3 * i + 2 for i in members] for members in self.free_heights]
        expand = np.zeros((3 * self.count, len(columns)))
        for column, rows in enumerate(columns):
            expand[rows, column] = 1.0
        return expand

    def pack(self, positions: np.ndarray) -> np.ndarray:
        return (self.expand.T @ positions.ravel()) / self.expand.sum(axis=0)

    def positions(self, params: np.ndarray) -> np.ndarray:
        return (self.expand @ params).reshape(-1, 3)

    def linearised(self, params: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The residuals (fitted distance minus range) and their Jacobian."""
        positions = self.positions(params)
        difference = positions[self.first] - positions[self.second]
        distance = np.linalg.norm(difference, axis=1)
        unit = difference / np.maximum(distance, 1e-12)[:, None]
        by_coordinate = np.zeros((len(self.pairs), 3 * self.count))
        rows = np.arange(len(self.pairs))
        for axis in range(3):
            by_coordinate[rows, 3 * self.first + axis] = unit[:, axis]
            by_coordinate[rows, 3 * self.second + axis] = -unit[:, axis]
        return distance - self.ranges, by_coordinate @ self.expand

    def aligned(self, positions: np.ndarray) -> np.ndarray:
        """The layout moved and turned about z so that the origin is at zero
        and the +x anchor on the x axis. Unknown heights stay unknown."""
        moved = positions - np.nan_to_num(positions[self.origin])
        x, y = moved[self.x_axis, :2]
        length = float(np.hypot(x, y))
        if length < 1e-9:
            return moved
        turn = np.array([[x, -y], [y, x]]) / length
        moved[:, :2] = moved[:, :2] @ turn
        return moved

    def descend(self, start: np.ndarray) -> _Solution:
        """Levenberg-Marquardt from a layout to the nearest minimum."""
        params = self.pack(self.aligned(start))
        residual, jacobian = self.linearised(params)
        cost = float(residual @ residual)
        damping = 1e-3
        for _ in range(MAX_ITERATIONS):
            normal = jacobian.T @ jacobian
            damped = normal + damping * np.diag(np.diag(normal) + 1e-9)
            step = np.linalg.lstsq(damped, -jacobian.T @ residual, rcond=None)[0]
            trial_residual, trial_jacobian = self.linearised(params + step)
            trial_cost = float(trial_residual @ trial_residual)
            if trial_cost > cost:
                damping *= 4.0
                if damping > 1e9:
                    break
                continue
            params, residual, jacobian, cost = (
                params + step,
                trial_residual,
                trial_jacobian,
                trial_cost,
            )
            damping = max(damping / 3.0, 1e-9)
            if np.abs(step).max() < STEP_TOLERANCE_M:
                break
        positions = self.positions(params)
        if positions[self.x_axis, 0] < 0:
            positions[:, :2] *= -1.0
        return _Solution(positions, float(np.sqrt(cost / max(len(self.pairs), 1))))

    def not_above(self, positions: np.ndarray) -> list[int]:
        """Indexes of the ``upper`` anchors that are not above the origin's
        group."""
        return [i for i in sorted(self.upper) if positions[i, 2] >= 0]

    def frame_violations(self, positions: np.ndarray) -> int:
        """How many anchors contradict ``upper`` or ``y_positive``."""
        wrong_side = self.y_positive is not None and positions[self.y_positive, 1] <= 0
        return len(self.not_above(positions)) + int(wrong_side)

    def scaled_start(self) -> np.ndarray | None:
        """Classical multidimensional scaling of the ranges, turned so that
        the origin's height group is level. A missing range is replaced by
        the shortest path through measured ones; ``None`` when the group is
        not connected."""
        distance = np.full((self.count, self.count), np.inf)
        np.fill_diagonal(distance, 0.0)
        distance[self.first, self.second] = distance[self.second, self.first] = (
            self.ranges
        )
        for via in range(self.count):
            distance = np.minimum(distance, distance[:, [via]] + distance[[via], :])
        if not np.isfinite(distance).all():
            return None
        centring = np.eye(self.count) - 1.0 / self.count
        values, vectors = np.linalg.eigh(-0.5 * centring @ distance**2 @ centring)
        shape = np.zeros((self.count, 3))
        kept = min(3, self.count)
        shape[:, :kept] = (vectors * np.sqrt(np.clip(values, 0.0, None)))[:, -kept:]
        level = shape[self.level]
        up = np.linalg.svd(level - level.mean(axis=0))[2][-1]
        across = np.cross(up, np.eye(3)[np.argmin(np.abs(up))])
        across /= np.linalg.norm(across)
        return shape @ np.column_stack([across, np.cross(up, across), up])

    def with_heights(self, rough: np.ndarray) -> np.ndarray:
        """A placement with the heights it does not give filled in."""
        start = rough.copy()
        start[self.level, 2] = 0.0
        for members in self.free_heights:
            given = start[members, 2]
            start[members, 2] = (
                -START_HEIGHT_M if np.isnan(given).all() else np.nanmean(given)
            )
        return start

    def variants(self, start: np.ndarray) -> Iterator[np.ndarray]:
        """The start, every exchange of two anchors, and each anchor
        reflected through the plane of the three nearest anchors it ranged."""
        yield start
        for i, j in combinations(range(self.count), 2):
            exchanged = start.copy()
            exchanged[[i, j]] = start[[j, i]]
            yield exchanged
        for i in range(self.count):
            ranged = sorted(
                (self.ranges[k], int(a + b - i))
                for k, (a, b) in enumerate(zip(self.first, self.second))
                if i in (a, b)
            )
            if len(ranged) < 3:
                continue
            base, second, third = (start[other] for _, other in ranged[:3])
            normal = np.cross(second - base, third - base)
            length = np.linalg.norm(normal)
            if length < 1e-9:
                continue
            normal /= length
            reflected = start.copy()
            reflected[i] -= 2.0 * ((start[i] - base) @ normal) * normal
            yield reflected

    def uncertainty(self, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Per anchor: the predicted standard deviation of its position, and
        whether the ranges leave it undetermined."""
        residual, jacobian = self.linearised(self.pack(positions))
        spare = len(self.pairs) - jacobian.shape[1]
        variance = NOISE_PRIOR_M**2
        if spare > 0:
            variance = max(float(residual @ residual) / spare, NOISE_FLOOR_M**2)
        _, singular, basis = np.linalg.svd(jacobian, full_matrices=True)
        strength = np.zeros(jacobian.shape[1])
        strength[: len(singular)] = singular
        solid = strength > RANK_TOLERANCE * max(float(strength.max(initial=0.0)), 1e-12)
        covariance = variance * (basis[solid].T / strength[solid] ** 2) @ basis[solid]
        spread = np.einsum("ij,jk,ik->i", self.expand, covariance, self.expand)
        loose = (np.abs(self.expand @ basis[~solid].T) > 1e-6).any(axis=1)
        sigma = np.sqrt(spread.reshape(-1, 3).sum(axis=1))
        return sigma, loose.reshape(-1, 3).any(axis=1)


def _pair_ranges(
    anchors: tuple[int, ...], measurements: Mapping[tuple[int, int], DirectedRange]
) -> dict[tuple[int, int], float]:
    """The range of every pair with a usable direction: the mean of the
    usable ones."""
    ranges = {}
    for a, b in combinations(anchors, 2):
        usable = [
            m.range_m
            for m in (measurements.get((a, b)), measurements.get((b, a)))
            if m is not None and m.usable
        ]
        if usable:
            ranges[a, b] = sum(usable) / len(usable)
    return ranges


def _layout(
    anchors: tuple[int, ...], points: Mapping[int, Sequence[float]] | None
) -> np.ndarray | None:
    """The points of every anchor as rows, NaN for a height not given;
    ``None`` unless every anchor has at least x and y."""
    if points is None or any(len(points.get(anchor, ())) < 2 for anchor in anchors):
        return None
    return np.array([[*points[anchor], np.nan][:3] for anchor in anchors], dtype=float)


def _distinct(solutions: list[_Solution]) -> list[_Solution]:
    """The different layouts among the solutions and their mirror images in
    y and in z, which fit the ranges exactly as well, best first."""
    mirrored = [
        _Solution(solution.positions * mirror, solution.rms)
        for solution in sorted(solutions, key=lambda s: s.rms)
        for mirror in product((1.0,), (1.0, -1.0), (1.0, -1.0))
    ]
    kept: list[_Solution] = []
    for solution in mirrored:
        if all(_differing(solution, other).any() for other in kept):
            kept.append(solution)
    return kept


def _differing(a: _Solution, b: _Solution) -> np.ndarray:
    return np.linalg.norm(a.positions - b.positions, axis=1) > SAME_SOLUTION_M


def _closest(
    items: list[_Solution], score: Callable[[_Solution], float], margin: float
) -> list[_Solution]:
    scores = [score(item) for item in items]
    return [item for item, value in zip(items, scores) if value <= min(scores) + margin]


def _onto(reference: np.ndarray, positions: np.ndarray) -> np.ndarray:
    """The reference turned about z and moved in x and y to lie as close to
    the positions as it can, never mirrored. A sketch is compared this way
    because its origin and +x anchors are as rough as the others, and may be
    the ones in the wrong place."""
    centre = positions[:, :2].mean(axis=0)
    ours = positions[:, :2] - centre
    theirs = reference[:, :2] - reference[:, :2].mean(axis=0)
    angle = np.arctan2(
        np.sum(theirs[:, 0] * ours[:, 1] - theirs[:, 1] * ours[:, 0]),
        np.sum(theirs * ours),
    )
    cos, sin = np.cos(angle), np.sin(angle)
    moved = reference.copy()
    moved[:, :2] = theirs @ np.array([[cos, sin], [-sin, cos]]) + centre
    return moved


def _match(
    positions: np.ndarray, reference: np.ndarray
) -> tuple[float, tuple[int, int] | None]:
    """How far a layout is from the reference, and the two anchors whose
    exchange in the reference explains it better, if any. An exchange
    counts double, so it has to at least halve the distance."""

    def apart(rows: np.ndarray) -> float:
        return float(np.sqrt(np.nansum((positions - _onto(rows, positions)) ** 2)))

    nearest, exchange = apart(reference), None
    for i, j in combinations(range(len(positions)), 2):
        exchanged = reference.copy()
        exchanged[[i, j]] = reference[[j, i]]
        distance = 2.0 * apart(exchanged)
        if distance < nearest:
            nearest, exchange = distance, (i, j)
    return nearest, exchange


def _choose(
    problem: _Problem, solutions: list[_Solution], reference: np.ndarray | None
) -> tuple[_Solution, np.ndarray]:
    """The winning layout, and per anchor whether another layout fits the
    ranges as well and neither the frame nor the reference tells them
    apart. A layout is a rival unless its rms is twice the best one's.
    Among rivals the frame decides first, then the nearest to the
    reference, then the one that needs no exchange to be that near."""
    limit = 2.0 * max(solutions[0].rms, NOISE_FLOOR_M)
    rivals = [s for s in solutions if max(s.rms, NOISE_FLOOR_M) < limit]
    rivals = _closest(rivals, lambda s: problem.frame_violations(s.positions), 0.0)
    if reference is not None:
        rivals = _closest(
            rivals, lambda s: _match(s.positions, reference)[0], SAME_SOLUTION_M
        )
        rivals = _closest(
            rivals, lambda s: _match(s.positions, reference)[1] is not None, 0.0
        )
    undecided = np.zeros(problem.count, dtype=bool)
    for other in rivals[1:]:
        undecided |= _differing(rivals[0], other)
    return rivals[0], undecided


def _pair_gate(
    anchors: tuple[int, ...],
    measurements: Mapping[tuple[int, int], DirectedRange],
    pairs: Sequence[PairFit],
) -> tuple[list[GateEntry], list[GateEntry]]:
    fitted = {(pair.a, pair.b): pair for pair in pairs}
    blocked, warnings = [], []
    for a, b in combinations(anchors, 2):
        directions = [
            (a, b, measurements.get((a, b))),
            (b, a, measurements.get((b, a))),
        ]
        if (a, b) not in fitted:
            seen = any(m is not None for _, _, m in directions)
            detail = "too few valid ranges" if seen else "not measured"
            blocked.append(GateEntry("pair-missing", a, b, detail))
            continue
        residual = fitted[a, b].residual_m
        if abs(residual) > PAIR_WARNING_M:
            warnings.append(GateEntry("pair", a, b, f"residual {residual:+.3f} m"))
        usable = [m.range_m for _, _, m in directions if m is not None and m.usable]
        if len(usable) == 2 and abs(usable[0] - usable[1]) > ASYMMETRY_WARNING_M:
            detail = f"the two directions differ by {abs(usable[0] - usable[1]):.3f} m"
            warnings.append(GateEntry("asymmetry", a, b, detail))
        for initiator, responder, m in directions:
            if m is not None and m.share < SHARE_WARNING:
                detail = f"valid share {m.share:.2f} of its ranges to {initiator}"
                warnings.append(GateEntry("low-share", responder, initiator, detail))
    return blocked, warnings


def _gate(
    anchors: tuple[int, ...],
    measurements: Mapping[tuple[int, int], DirectedRange],
    pairs: Sequence[PairFit],
    reports: Mapping[int, AnchorFit],
    reasons: Mapping[int, str],
    rms: float,
) -> tuple[tuple[GateEntry, ...], tuple[GateEntry, ...]]:
    blocked, warnings = _pair_gate(anchors, measurements, pairs)
    for anchor in anchors:
        report = reports[anchor]
        if report.links < min(3, len(anchors) - 1):
            detail = f"{report.links} links, a position needs three"
        elif anchor in reasons:
            detail = reasons[anchor]
        elif report.sigma_m is not None and report.sigma_m > SIGMA_BLOCK_M:
            detail = f"predicted uncertainty {report.sigma_m:.2f} m"
        else:
            continue
        blocked.append(GateEntry("not-located", anchor, None, detail))
    if rms > RMS_WARNING_M:
        warnings.append(GateEntry("rms", None, None, f"fit rms {rms:.3f} m"))
    return tuple(blocked), tuple(warnings)


def _start(
    problem: _Problem,
    anchors: tuple[int, ...],
    placement: Mapping[int, Sequence[float]] | None,
    stored: Mapping[int, Sequence[float]] | None,
) -> tuple[np.ndarray, np.ndarray | None, str, list[str]]:
    """Where the fit starts, the reference that settles ties (``None`` when
    the start is the measured shape), what the reference is called, and
    notes."""
    notes = []
    source = "the placement"
    reference = _layout(anchors, placement)
    if reference is None:
        if placement:
            notes.append("the placement does not cover every anchor and was not used")
        source = "the stored positions"
        reference = _layout(anchors, stored)
    if reference is not None:
        reference = problem.aligned(reference)
        return problem.with_heights(reference), reference, source, notes
    start = problem.scaled_start()
    if start is None:
        raise SurveyFitError("Place the anchors roughly first")
    return start, None, source, notes


def _anchor_fits(
    problem: _Problem,
    anchors: tuple[int, ...],
    positions: np.ndarray,
    residual: np.ndarray,
    stored: Mapping[int, Sequence[float]] | None,
) -> tuple[dict[int, AnchorFit], dict[int, str]]:
    """The report of every anchor, and why the ranges do not locate the ones
    they do not."""
    sigma, loose = problem.uncertainty(positions)
    reports, reasons = {}, {}
    for i, anchor in enumerate(anchors):
        own = residual[(problem.first == i) | (problem.second == i)]
        kept = stored.get(anchor) if stored else None
        moved = None
        if kept is not None:
            moved = float(
                np.linalg.norm(positions[i] - np.array(kept[:3], dtype=float))
            )
        x, y, z = (float(value) for value in positions[i])
        reports[anchor] = AnchorFit(
            position=(x, y, z),
            links=len(own),
            residual_rms_m=float(np.sqrt(np.mean(own**2))) if len(own) else 0.0,
            worst_pair_m=float(np.abs(own).max(initial=0.0)),
            sigma_m=None if loose[i] else float(sigma[i]),
            moved_m=moved,
        )
        if loose[i]:
            reasons[anchor] = "the ranges do not fix its position"
    return reports, reasons


def fit_survey(
    anchors: Sequence[int],
    measurements: Mapping[tuple[int, int], DirectedRange],
    frame: Frame,
    *,
    placement: Mapping[int, Sequence[float]] | None = None,
    stored: Mapping[int, Sequence[float]] | None = None,
) -> SurveyFit:
    """Fits the positions of a surveyed group and grades the result.

    Args:
        anchors: the group, system IDs.
        measurements: by ``(initiator, responder)``, what the responder
            measured in the step the initiator led.
        frame: the frame declaration.
        placement: rough ``(x, y)`` or ``(x, y, z)`` per anchor in the
            declared frame, metres. Used only when it covers the group.
        stored: the ``(x, y, z)`` each anchor stores today, where it does.
            They start the fit when there is no placement and every anchor
            has one.

    Raises:
        SurveyFitError: the frame does not fit the group, or there is
            nothing to start from.
    """
    anchors = tuple(anchors)
    refusal = frame_refusal(frame, anchors)
    if refusal:
        raise SurveyFitError(refusal)
    problem = _Problem(anchors, frame, _pair_ranges(anchors, measurements))
    start, reference, source, notes = _start(problem, anchors, placement, stored)

    solutions = _distinct([problem.descend(v) for v in problem.variants(start)])
    winner, undecided = _choose(problem, solutions, reference)
    positions = winner.positions

    exchange = None if reference is None else _match(positions, reference)[1]
    if exchange:
        first, second = anchors[exchange[0]], anchors[exchange[1]]
        notes.append(f"anchors {first} and {second} are swapped against {source}")
    for i in problem.not_above(positions):
        notes.append(
            f"anchor {anchors[i]} is declared upper but the ranges put it"
            " below the origin's group"
        )

    residual, _ = problem.linearised(problem.pack(positions))
    pairs = tuple(
        PairFit(
            a=a,
            b=b,
            range_m=float(problem.ranges[k]),
            ab=measurements.get((a, b)),
            ba=measurements.get((b, a)),
            fitted_m=float(problem.ranges[k] + residual[k]),
            residual_m=float(residual[k]),
        )
        for k, (a, b) in enumerate(problem.pairs)
    )
    reports, reasons = _anchor_fits(problem, anchors, positions, residual, stored)
    for i in np.flatnonzero(undecided):
        reasons.setdefault(
            anchors[i],
            "another layout fits the ranges as well, and neither the placement"
            " nor the frame tells them apart",
        )
    blocked, warnings = _gate(
        anchors, measurements, pairs, reports, reasons, winner.rms
    )
    return SurveyFit(
        anchors=reports,
        pairs=pairs,
        rms_m=winner.rms,
        worst_m=float(np.abs(residual).max(initial=0.0)),
        planar=problem.planar,
        notes=tuple(notes),
        blocked=blocked,
        warnings=warnings,
    )
