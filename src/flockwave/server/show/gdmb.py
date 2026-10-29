"""Ground validator for `.gdmb` per-drone show payloads.

`.gdmb` is the per-drone binary of the `.gdm` show format. The normative
contract is AxioStack `contracts/gdm-format.md` (version 1); section numbers
in this module refer to it. The trajectory engine compiles and certifies each
`.gdmb` (ADR 0002 decision D2), so Axio Server never re-encodes one: it only
checks that the bytes it is about to upload satisfy every rule of §7.1 and
carry the expected show hash, and it rejects the whole file on the first
violation (fail closed).

The error codes in `GdmbValidationError.code` are the stable identifiers of
§7.1. The golden vectors in AxioStack `contracts/gdm-vectors/` name the code
each reject vector must produce.

v1 conformance covers FLAT trajectories, LIGHTS, EVENTS and COMMENT. The
MANEUVER, RTH_PLAN and ENVELOPES layouts are provisional (§4.11): by default
this validator refuses them, as a v1 reader must. `provisional=True` checks
them against their current layouts instead; nothing on the upload path uses
that mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, sin, sqrt
from struct import Struct

from .utils import crc32_mavftp

__all__ = (
    "GDMB_MAX_SIZE",
    "GdmbSummary",
    "GdmbValidationError",
    "validate_gdmb",
)

GDMB_MAX_SIZE = 262_144
"""Maximum size of a `.gdmb` file in bytes (§4, ADR 0002 D6)."""

GDMB_MAGIC = b"GDMB"
GDMB_VERSION = 1
"""The newest `.gdmb` version this reader implements; it accepts every
version from 1 up to this one (§1.8).
"""
GDMB_HEADER_LENGTH = 48
"""Length of the header defined by this version. Later versions may extend
it; a reader skips the bytes it does not know (§4.1).
"""

_CRC_OFFSET = 8
_SHOW_HASH_OFFSET = 12
_CAPS_OFFSET = 44

FEATURE_CRC32 = 0x01
FEATURE_MANEUVER = 0x02
FEATURE_REVERSIBLE_THRUST = 0x04
_FEATURES_DEFINED = FEATURE_CRC32 | FEATURE_MANEUVER | FEATURE_REVERSIBLE_THRUST
_FEATURES_PROVISIONAL = FEATURE_MANEUVER

CAPABILITY_MANEUVER = 0x01
CAPABILITY_REVERSIBLE_THRUST = 0x02
CAPABILITY_HIGH_TILT = 0x04
"""Reserved: certification refuses tilt above the normal-executor bound
until a high-tilt executor exists, so no v1 file sets it (E_CAPS)."""
_CAPABILITIES_DEFINED = CAPABILITY_MANEUVER | CAPABILITY_REVERSIBLE_THRUST
_CAPABILITIES_PROVISIONAL = CAPABILITY_MANEUVER

BLOCK_FLAT_TRAJECTORY = 0x01
BLOCK_LIGHTS = 0x02
BLOCK_COMMENT = 0x03
BLOCK_RTH_PLAN = 0x04
BLOCK_EVENTS = 0x06
BLOCK_MANEUVER = 0x07
BLOCK_ENVELOPES = 0x08
_BLOCK_FLAG_SKIPPABLE = 0x01

_ABORT_LEVEL_HOLD_LAND = 1
_EVENT_TYPE_PYRO = 1
_EVENT_PAYLOADS = (0x00000000, 0xFFFFFFFF)

_MIN_SEGMENT_US = 100_000
"""Shortest FLAT segment (§4.5): below it the §7.3 tolerances stop meaning
anything.
"""
_MIN_KNOT_US = 1_000
_MAX_TIMELINE_US = 2**32 - 1

#: Rank of each known block type in the canonical order of §4.4. Motion
#: blocks share rank 0.
_BLOCK_RANK = {
    BLOCK_FLAT_TRAJECTORY: 0,
    BLOCK_MANEUVER: 0,
    BLOCK_LIGHTS: 1,
    BLOCK_EVENTS: 2,
    BLOCK_RTH_PLAN: 3,
    BLOCK_ENVELOPES: 4,
    BLOCK_COMMENT: 5,
}

#: Block types that are never valid in version 1 (0x05 is the `.skyb`
#: YAW_CONTROL block; `.gdmb` keeps yaw inside FLAT segments).
_FORBIDDEN_BLOCK_TYPES = (0x00, 0x05)

#: Provisional block types (§4.11), reserved in v1
_PROVISIONAL_BLOCK_TYPES = (BLOCK_RTH_PLAN, BLOCK_MANEUVER, BLOCK_ENVELOPES)

# Quanta and scales (§1.5)
_POS_SCALE = 10_000.0
_YAW_SCALE = 1_000_000.0
_THRUST_SCALE = 1_000.0
_QUAT_SCALE = 32_767

# Continuity tolerances (§7.3)
_Q_POS = 1e-4
_Q_YAW = 1e-6
_SLACK_V = 1e-3
_SLACK_A = 1e-2
_SLACK_YAW_RATE = 1e-3
_TOL_ATTITUDE = 1e-2
_TOL_BODY_RATE = 5e-2
_TOL_THRUST = 5e-2

#: Standard gravity, identical to ArduPilot's GRAVITY_MSS (§1.1)
_G = 9.80665

_BLOCK_HEADER = Struct("<BBI")
_U16 = Struct("<H")
_U32 = Struct("<I")
_I32 = Struct("<i")
_I16 = Struct("<h")
_EVENT = Struct("<IBBI")
_ENVELOPE = Struct("<IIHH")
_MANEUVER_HEADER = Struct("<IIHHHBBiiii")


class GdmbValidationError(ValueError):
    """Raised when a `.gdmb` file violates a rule of §7.1."""

    code: str
    """Stable error code from §7.1, e.g. ``E_CRC``."""

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code


@dataclass(frozen=True)
class GdmbSummary:
    """What the validator learned about a valid `.gdmb` file."""

    size: int
    """Size of the file in bytes."""

    crc32: int
    """The header CRC (AP-CRC32 of the file with the CRC field zeroed)."""

    show_hash: bytes
    """The 32-byte show hash from the header."""

    features: int
    """Feature flags (§4.2)."""

    capabilities: int
    """Required capability bits (§4.3)."""

    duration_us: int
    """Length of the show timeline in microseconds."""

    num_maneuvers: int
    """Number of MANEUVER blocks."""

    rth_branch_times_us: tuple[int, ...]
    """Branch times of the RTH plan; empty if the file has no RTH plan."""


# ---------------------------------------------------------------------------
# Decoded model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Segment:
    """A FLAT segment record (§4.5). `ctrl` holds the control values
    `P_1 … P_n` of x, y, z and yaw (yaw may be absent for `p̂`), as integers
    on their grids, without the implicit `P_0`.
    """

    duration_us: int
    sigma: int
    ctrl: tuple[tuple[int, ...], ...]

    def end(self, start: tuple[int, ...]) -> tuple[int, ...]:
        return tuple(axis[-1] if axis else start[k] for k, axis in enumerate(self.ctrl))


@dataclass(frozen=True)
class _FlatBlock:
    start: tuple[int, int, int, int]
    segments: list[_Segment]


@dataclass(frozen=True)
class _ManeuverBlock:
    duration_us: int
    knot_us: int
    start: tuple[int, int, int]
    yaw_exit: int
    p_hat: _Segment
    quaternions: list[tuple[int, int, int, int]]
    thrust: list[int]


class _Reader:
    """Sequential little-endian reader over one block payload. Running past
    the end of the payload is a framing error.
    """

    def __init__(self, data: bytes, offset: int, end: int, where: str):
        self._data = data
        self._pos = offset
        self._end = end
        self._where = where

    @property
    def at_end(self) -> bool:
        return self._pos == self._end

    @property
    def remaining(self) -> int:
        return self._end - self._pos

    def read(self, st: Struct) -> tuple:
        if self._pos + st.size > self._end:
            raise GdmbValidationError(
                "E_FRAMING", f"{self._where}: payload ends inside a field"
            )
        values = st.unpack_from(self._data, self._pos)
        self._pos += st.size
        return values

    def u16(self) -> int:
        return self.read(_U16)[0]

    def u32(self) -> int:
        return self.read(_U32)[0]

    def i32(self) -> int:
        return self.read(_I32)[0]

    def i32s(self, count: int) -> tuple[int, ...]:
        if count == 0:
            return ()
        return self.read(Struct(f"<{count}i"))

    def i16s(self, count: int) -> tuple[int, ...]:
        return self.read(Struct(f"<{count}h"))


def _read_segment(r: _Reader, *, reversible: bool, where: str) -> _Segment:
    """Reads one FLAT segment record (§4.5), validating its header before
    consuming its control values.
    """
    header = r.u16()
    if header & 0xE000:
        raise GdmbValidationError(
            "E_RESERVED", f"{where}: reserved segment header bits set"
        )
    degrees = (header & 7, (header >> 3) & 7, (header >> 6) & 7, (header >> 9) & 7)
    if degrees[3] > 5:
        raise GdmbValidationError(
            "E_DEGREE", f"{where}: yaw degree {degrees[3]} is above 5"
        )
    sigma = -1 if header & 0x1000 else 1
    if sigma < 0 and not reversible:
        raise GdmbValidationError(
            "E_SIGMA", f"{where}: inverted segment without REVERSIBLE_THRUST"
        )
    duration = r.u32()
    if duration < _MIN_SEGMENT_US:
        raise GdmbValidationError(
            "E_DURATION", f"{where}: duration {duration} us is below 100 ms"
        )
    ctrl = tuple(r.i32s(d) for d in degrees)
    return _Segment(duration_us=duration, sigma=sigma, ctrl=ctrl)


# ---------------------------------------------------------------------------
# Curve evaluation (§6.2, §6.3)
# ---------------------------------------------------------------------------


def _endpoint_derivative(
    p0: int, ctrl: tuple[int, ...], order: int, at_end: bool, duration_us: int
) -> int | float:
    """Returns the `order`-th time derivative of a Bézier axis at s = 0 or
    s = 1, in grid units per second^order. The forward differences are formed
    in integer arithmetic before anything is converted to floating point.
    """
    points = (p0, *ctrl)
    n = len(points) - 1
    if order > n:
        return 0
    if order == 0:
        return points[-1] if at_end else points[0]

    window = points[n - order :] if at_end else points[: order + 1]
    diff = 0
    binomial = 1
    for i in range(order + 1):
        # Δ^m P_j = Σ (-1)^(m-i) C(m, i) P_{j+i}
        sign = -1 if (order - i) % 2 else 1
        diff += sign * binomial * window[i]
        binomial = binomial * (order - i) // (i + 1)

    falling = 1
    for k in range(order):
        falling *= n - k
    seconds = duration_us / 1e6
    return falling * diff / seconds**order


def _check_c2_joint(
    start_a: tuple[int, ...],
    seg_a: _Segment,
    start_b: tuple[int, ...],
    seg_b: _Segment,
    *,
    code: str,
    where: str,
) -> None:
    """FLAT ↔ FLAT continuity check of §7.3 between segment `a` (ending at
    the joint) and segment `b` (starting there). Position axes must be C²;
    yaw, when both sides carry it, C¹.
    """
    t_a = seg_a.duration_us / 1e6
    t_b = seg_b.duration_us / 1e6
    axes = min(len(seg_a.ctrl), len(seg_b.ctrl))
    for k in range(axes):
        is_yaw = k == 3
        scale = _YAW_SCALE if is_yaw else _POS_SCALE
        quantum = _Q_YAW if is_yaw else _Q_POS
        n_a, n_b = len(seg_a.ctrl[k]), len(seg_b.ctrl[k])
        name = "xyzψ"[k]

        v_a = _endpoint_derivative(
            start_a[k], seg_a.ctrl[k], 1, True, seg_a.duration_us
        )
        v_b = _endpoint_derivative(
            start_b[k], seg_b.ctrl[k], 1, False, seg_b.duration_us
        )
        tol_v = quantum * (n_a / t_a + n_b / t_b) + (
            _SLACK_YAW_RATE if is_yaw else _SLACK_V
        )
        jump = abs(v_a - v_b) / scale
        if jump > tol_v:
            raise GdmbValidationError(
                code,
                f"{where}: d{name}/dt jumps by {jump:.6g} (tolerance {tol_v:.3g})",
            )

        if is_yaw:
            continue

        a_a = _endpoint_derivative(
            start_a[k], seg_a.ctrl[k], 2, True, seg_a.duration_us
        )
        a_b = _endpoint_derivative(
            start_b[k], seg_b.ctrl[k], 2, False, seg_b.duration_us
        )
        tol_a = (
            2 * quantum * (n_a * (n_a - 1) / t_a**2 + n_b * (n_b - 1) / t_b**2)
            + _SLACK_A
        )
        jump = abs(a_a - a_b) / scale
        if jump > tol_a:
            raise GdmbValidationError(
                code,
                f"{where}: d²{name}/dt² jumps by {jump:.6g} (tolerance {tol_a:.3g})",
            )


# ---------------------------------------------------------------------------
# Attitude: flatness map (§6.4) and the SO(3) B-spline (§6.5)
# ---------------------------------------------------------------------------

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]  # rows


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return sqrt(_dot(a, a))


def _quat_mul(p: Quat, q: Quat) -> Quat:
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    )


def _quat_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def _quat_exp(phi: Vec3) -> Quat:
    angle = _norm(phi)
    if angle < 1e-12:
        w, x, y, z = 1.0, phi[0] / 2, phi[1] / 2, phi[2] / 2
        n = sqrt(w * w + x * x + y * y + z * z)
        return (w / n, x / n, y / n, z / n)
    s = sin(angle / 2) / angle
    return (cos(angle / 2), phi[0] * s, phi[1] * s, phi[2] * s)


def _quat_log(q: Quat) -> Vec3:
    v = (q[1], q[2], q[3])
    n = _norm(v)
    if n < 1e-12:
        return _scale(v, 2.0 / q[0])
    return _scale(v, 2.0 * atan2(n, q[0]) / n)


def _quat_to_matrix(q: Quat) -> Mat3:
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _transpose_times(m: Mat3, v: Vec3) -> Vec3:
    return (
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    )


def _rotation_angle(a: Mat3, b: Mat3) -> float:
    """∠(A, B) = acos((tr(AᵀB) − 1) / 2) (§7.3)."""
    trace = sum(a[r][c] * b[r][c] for r in range(3) for c in range(3))
    return acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))


def _flat_attitude(
    acc: Vec3, jerk: Vec3, yaw: float, yaw_rate: float, sigma: int
) -> tuple[Mat3, Vec3, float]:
    """Flatness map of §6.4: attitude (rows of R), body rates and signed
    specific thrust for a FLAT state.
    """
    f = (acc[0], acc[1], acc[2] + _G)
    f_norm = _norm(f)
    f_hat = _scale(f, 1 / f_norm)
    z_b = _scale(f_hat, sigma)
    z_b_dot = _scale(_sub(jerk, _scale(f_hat, _dot(f_hat, jerk))), sigma / f_norm)

    # ψ is clockwise-positive (§1.2), so the heading is a rotation by -ψ
    x_c = (cos(yaw), -sin(yaw), 0.0)
    x_c_dot = (-sin(yaw) * yaw_rate, -cos(yaw) * yaw_rate, 0.0)

    y = _cross(z_b, x_c)
    y_dot = _add(_cross(z_b_dot, x_c), _cross(z_b, x_c_dot))
    y_norm = _norm(y)
    y_b = _scale(y, 1 / y_norm)
    y_b_dot = _scale(_sub(y_dot, _scale(y_b, _dot(y_b, y_dot))), 1 / y_norm)

    x_b = _cross(y_b, z_b)
    x_b_dot = _add(_cross(y_b_dot, z_b), _cross(y_b, z_b_dot))

    rows = (
        (x_b[0], y_b[0], z_b[0]),
        (x_b[1], y_b[1], z_b[1]),
        (x_b[2], y_b[2], z_b[2]),
    )
    omega = (_dot(y_b_dot, z_b), _dot(z_b_dot, x_b), _dot(x_b_dot, y_b))
    return rows, omega, sigma * f_norm


def _maneuver_state(m: _ManeuverBlock, tau_us: int) -> tuple[Quat, Vec3, float]:
    """Attitude, body rate and specific thrust of a maneuver at local time
    `tau_us` (§6.5, §6.6).
    """
    n = len(m.quaternions)
    i = min(tau_us // m.knot_us, n - 4)
    u = (tau_us - i * m.knot_us) / m.knot_us

    quats: list[Quat] = []
    for q in m.quaternions[i : i + 4]:
        length = sqrt(sum(c * c for c in q))
        quats.append((q[0] / length, q[1] / length, q[2] / length, q[3] / length))

    # Cumulative basis λ_j(u) and dλ_j/du for j = 1, 2, 3
    lam = (
        (5 + 3 * u - 3 * u * u + u**3) / 6,
        (1 + 3 * u + 3 * u * u - 2 * u**3) / 6,
        u**3 / 6,
    )
    dlam = (
        (3 - 6 * u + 3 * u * u) / 6,
        (3 + 6 * u - 6 * u * u) / 6,
        3 * u * u / 6,
    )

    q = quats[0]
    omega: Vec3 = (0.0, 0.0, 0.0)
    for j in range(1, 4):
        d = _quat_log(_quat_mul(_quat_conj(quats[j - 1]), quats[j]))
        e = _quat_exp(_scale(d, lam[j - 1]))
        q = _quat_mul(q, e)
        omega = _add(
            _transpose_times(_quat_to_matrix(e), omega), _scale(d, dlam[j - 1])
        )

    knot_s = m.knot_us / 1e6
    omega = _scale(omega, 1 / knot_s)

    f = m.thrust[i : i + 4]
    thrust = (
        (1 - u) ** 3 * f[0]
        + (3 * u**3 - 6 * u * u + 4) * f[1]
        + (-3 * u**3 + 3 * u * u + 3 * u + 1) * f[2]
        + u**3 * f[3]
    ) / 6
    return q, omega, thrust / _THRUST_SCALE


def _flat_endpoint_state(
    start: tuple[int, ...], seg: _Segment, at_end: bool
) -> tuple[Vec3, Vec3, float, float]:
    """Acceleration, jerk, yaw and yaw rate of a FLAT segment at one end."""

    def derivative(axis: int, order: int) -> float:
        return _endpoint_derivative(
            start[axis], seg.ctrl[axis], order, at_end, seg.duration_us
        )

    acc = (derivative(0, 2), derivative(1, 2), derivative(2, 2))
    jerk = (derivative(0, 3), derivative(1, 3), derivative(2, 3))
    return (
        _scale(acc, 1 / _POS_SCALE),
        _scale(jerk, 1 / _POS_SCALE),
        derivative(3, 0) / _YAW_SCALE,
        derivative(3, 1) / _YAW_SCALE,
    )


def _check_regime_joint(
    flat_start: tuple[int, ...],
    flat_seg: _Segment,
    maneuver: _ManeuverBlock,
    *,
    entry: bool,
) -> None:
    """FLAT ↔ MANEUVER joint check of §7.3. At entry the FLAT segment ends at
    the maneuver's τ = 0; at exit it starts at the maneuver's τ = T.
    """
    which = "entry" if entry else "exit"
    p_hat_start = (*maneuver.start, 0)
    if entry:
        _check_c2_joint(
            flat_start,
            flat_seg,
            p_hat_start,
            maneuver.p_hat,
            code="E_REGIME",
            where=f"maneuver {which}",
        )
        tau = 0
    else:
        _check_c2_joint(
            p_hat_start,
            maneuver.p_hat,
            flat_start,
            flat_seg,
            code="E_REGIME",
            where=f"maneuver {which}",
        )
        tau = maneuver.duration_us

    acc, jerk, yaw, yaw_rate = _flat_endpoint_state(flat_start, flat_seg, entry)
    rot_flat, omega_flat, thrust_flat = _flat_attitude(
        acc, jerk, yaw, yaw_rate, flat_seg.sigma
    )
    q, omega, thrust = _maneuver_state(maneuver, tau)

    angle = _rotation_angle(rot_flat, _quat_to_matrix(q))
    if angle > _TOL_ATTITUDE:
        raise GdmbValidationError(
            "E_REGIME", f"maneuver {which}: attitude differs by {angle:.4g} rad"
        )
    rate_error = _norm(_sub(omega_flat, omega))
    if rate_error > _TOL_BODY_RATE:
        raise GdmbValidationError(
            "E_REGIME",
            f"maneuver {which}: body rate differs by {rate_error:.4g} rad/s",
        )
    thrust_error = abs(thrust_flat - thrust)
    if thrust_error > _TOL_THRUST:
        raise GdmbValidationError(
            "E_REGIME",
            f"maneuver {which}: thrust differs by {thrust_error:.4g} m/s²",
        )


# ---------------------------------------------------------------------------
# Blocks
# ---------------------------------------------------------------------------


def _parse_flat_block(
    data: bytes, offset: int, length: int, *, reversible: bool
) -> _FlatBlock:
    where = f"FLAT_TRAJECTORY at {offset - 6}"
    r = _Reader(data, offset, offset + length, where)
    start = (r.i32(), r.i32(), r.i32(), r.i32())
    segments: list[_Segment] = []
    while not r.at_end:
        segments.append(
            _read_segment(
                r, reversible=reversible, where=f"{where}, segment {len(segments)}"
            )
        )
    if not segments:
        raise GdmbValidationError("E_FRAMING", f"{where}: no segments")
    return _FlatBlock(start=start, segments=segments)


def _parse_maneuver_block(
    data: bytes, offset: int, length: int, *, reversible: bool
) -> _ManeuverBlock:
    where = f"MANEUVER at {offset - 6}"
    r = _Reader(data, offset, offset + length, where)
    (
        duration,
        knot,
        count,
        envelope_pos,
        envelope_att,
        abort,
        reserved,
        x,
        y,
        z,
        yaw_exit,
    ) = r.read(_MANEUVER_HEADER)

    if reserved:
        raise GdmbValidationError("E_RESERVED", f"{where}: reserved byte is not 0")
    if abort != _ABORT_LEVEL_HOLD_LAND:
        raise GdmbValidationError("E_ABORT", f"{where}: unknown abort action {abort}")
    if envelope_pos < 1 or envelope_att < 1:
        raise GdmbValidationError("E_ENVELOPE", f"{where}: zero envelope")
    if count < 4:
        raise GdmbValidationError("E_MANEUVER", f"{where}: {count} control points")
    if knot < _MIN_KNOT_US or duration != (count - 3) * knot:
        raise GdmbValidationError(
            "E_DURATION", f"{where}: duration is not (N - 3) * knot_us"
        )

    # The p̂ record never carries σ = -1, whatever the feature flags say, so
    # an inverted p̂ is reported as a maneuver error rather than E_SIGMA.
    p_hat = _read_segment(r, reversible=True, where=f"{where}, p_hat")
    if p_hat.ctrl[3] or p_hat.sigma != 1 or p_hat.duration_us != duration:
        raise GdmbValidationError(
            "E_MANEUVER",
            f"{where}: p_hat needs yaw degree 0, sigma +1 and the maneuver duration",
        )
    p_hat = _Segment(duration_us=duration, sigma=1, ctrl=p_hat.ctrl[:3])

    flat_quats = r.i16s(4 * count)
    thrust = list(r.i16s(count))
    if not r.at_end:
        raise GdmbValidationError(
            "E_FRAMING", f"{where}: {r.remaining} bytes after the thrust values"
        )

    quats = [
        (
            flat_quats[4 * k],
            flat_quats[4 * k + 1],
            flat_quats[4 * k + 2],
            flat_quats[4 * k + 3],
        )
        for k in range(count)
    ]
    for k, q in enumerate(quats):
        if -32768 in q:
            raise GdmbValidationError("E_QUAT", f"{where}: q[{k}] uses -32768")
        norm = sqrt(sum(c * c for c in q)) / _QUAT_SCALE
        if not 0.999 <= norm <= 1.001:
            raise GdmbValidationError(
                "E_QUAT", f"{where}: |q[{k}]| = {norm:.6f} is not within 0.1% of 1"
            )
    if next(c for c in quats[0] if c != 0) < 0:
        raise GdmbValidationError("E_QUAT", f"{where}: q[0] is not in canonical sign")
    for k in range(count - 1):
        if sum(a * b for a, b in zip(quats[k], quats[k + 1], strict=True)) <= 0:
            raise GdmbValidationError(
                "E_QUAT", f"{where}: q[{k}] and q[{k + 1}] break the hemisphere rule"
            )
    if not reversible and min(thrust) < 0:
        raise GdmbValidationError(
            "E_THRUST", f"{where}: negative thrust without REVERSIBLE_THRUST"
        )

    return _ManeuverBlock(
        duration_us=duration,
        knot_us=knot,
        start=(x, y, z),
        yaw_exit=yaw_exit,
        p_hat=p_hat,
        quaternions=quats,
        thrust=thrust,
    )


def _check_rth_plan(
    data: bytes,
    offset: int,
    length: int,
    *,
    branches: dict[int, tuple[tuple[int, ...], _Segment]],
    reversible: bool,
) -> tuple[int, ...]:
    """Parses the RTH_PLAN payload (§4.10) and checks E_RTH and the C²
    rule at every branch and inside every path. Returns the branch times.
    """
    where = f"RTH_PLAN at {offset - 6}"
    r = _Reader(data, offset, offset + length, where)
    target = (r.i32(), r.i32(), r.i32())
    num_entries = r.u16()
    if num_entries < 1:
        raise GdmbValidationError("E_RTH", f"{where}: no entries")

    times: list[int] = []
    for e in range(num_entries):
        branch_time = r.u32()
        num_segments = r.u16()
        if num_segments < 1:
            raise GdmbValidationError("E_RTH", f"{where}: entry {e} has no segments")
        segments = [
            _read_segment(
                r, reversible=reversible, where=f"{where}, entry {e}, segment {k}"
            )
            for k in range(num_segments)
        ]

        if times and branch_time <= times[-1]:
            raise GdmbValidationError(
                "E_RTH", f"{where}: branch times are not strictly increasing"
            )
        times.append(branch_time)

        joint = branches.get(branch_time)
        if joint is None:
            raise GdmbValidationError(
                "E_RTH",
                f"{where}: branch time {branch_time} us is not a FLAT/FLAT joint",
            )
        prev_start, prev_seg = joint
        if prev_seg.sigma != 1 or any(s.sigma != 1 for s in segments):
            raise GdmbValidationError("E_RTH", f"{where}: entry {e} is not upright")

        # The path starts from the show state at the branch time and must be
        # C² against the show segment ending there, and at each inner joint.
        cur = prev_seg.end(prev_start)
        for k, seg in enumerate(segments):
            _check_c2_joint(
                prev_start,
                prev_seg,
                cur,
                seg,
                code="E_C2",
                where=f"{where}, entry {e}, joint {k}",
            )
            prev_start, prev_seg = cur, seg
            cur = seg.end(cur)

        if cur[:3] != target:
            raise GdmbValidationError(
                "E_RTH", f"{where}: entry {e} does not end at the target"
            )
        last = segments[-1]
        for k in range(4):
            points = (prev_start[k], *last.ctrl[k])
            needed = min(len(last.ctrl[k]), 1 if k == 3 else 2) + 1
            if len(set(points[-needed:])) != 1:
                raise GdmbValidationError(
                    "E_RTH",
                    f"{where}: entry {e} does not end at rest on axis {'xyzψ'[k]}",
                )

    if not r.at_end:
        raise GdmbValidationError(
            "E_FRAMING", f"{where}: {r.remaining} bytes after the last entry"
        )
    return tuple(times)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def validate_gdmb(
    data: bytes, *, show_hash: bytes | None = None, provisional: bool = False
) -> GdmbSummary:
    """Validates a `.gdmb` file against every rule of §7.1.

    Parameters:
        data: the raw bytes of the file
        show_hash: the show hash of the `.gdm` that the file belongs to. When
            given, the header must carry exactly this hash (``E_SHOW_HASH``).
            The ground path always passes it.
        provisional: whether to check the provisional MANEUVER, RTH_PLAN and
            ENVELOPES layouts (§4.11) instead of refusing them as a v1 reader
            must. Outside v1 conformance; the upload path never sets it.

    Returns:
        a summary of the file

    Raises:
        GdmbValidationError: on the first violation found, with its §7.1 code
    """
    data = bytes(data)
    size = len(data)

    # Header (§4.1 - §4.3, §4.9)
    if not GDMB_HEADER_LENGTH <= size <= GDMB_MAX_SIZE:
        raise GdmbValidationError(
            "E_SIZE",
            f"file size {size} is outside [{GDMB_HEADER_LENGTH}, {GDMB_MAX_SIZE}]",
        )
    if data[:4] != GDMB_MAGIC:
        raise GdmbValidationError("E_MAGIC", "not a .gdmb file")
    if not 1 <= data[4] <= GDMB_VERSION:
        raise GdmbValidationError(
            "E_VERSION",
            f"version {data[4]} is not between 1 and {GDMB_VERSION}",
        )
    features = data[5]
    (header_length,) = _U16.unpack_from(data, 6)
    if not GDMB_HEADER_LENGTH <= header_length <= size:
        # Longer headers belong to later versions; their extra bytes are
        # skipped below (§1.8, §4.1)
        raise GdmbValidationError(
            "E_HEADER",
            f"header length {header_length} is not between "
            f"{GDMB_HEADER_LENGTH} and the file size",
        )
    (stored_crc,) = _U32.unpack_from(data, _CRC_OFFSET)
    crc = crc32_mavftp(
        data[:_CRC_OFFSET] + b"\x00\x00\x00\x00" + data[_CRC_OFFSET + 4 :]
    )
    if crc != stored_crc:
        raise GdmbValidationError(
            "E_CRC", f"CRC is 0x{crc:08x}, header says 0x{stored_crc:08x}"
        )
    if features & ~_FEATURES_DEFINED:
        raise GdmbValidationError("E_RESERVED", "reserved feature bits are set")
    if not provisional and features & _FEATURES_PROVISIONAL:
        raise GdmbValidationError(
            "E_RESERVED", "MANEUVER feature bit is reserved in v1 (provisional)"
        )
    if not features & FEATURE_CRC32:
        raise GdmbValidationError("E_FEATURES", "CRC32 feature bit is clear")

    header_hash = data[_SHOW_HASH_OFFSET : _SHOW_HASH_OFFSET + 32]
    if show_hash is not None and header_hash != bytes(show_hash):
        raise GdmbValidationError("E_SHOW_HASH", "the file belongs to a different show")

    (caps,) = _U32.unpack_from(data, _CAPS_OFFSET)
    if caps & ~_CAPABILITIES_DEFINED:
        raise GdmbValidationError("E_CAPS", "undefined capability bits are set")
    if not provisional and caps & _CAPABILITIES_PROVISIONAL:
        raise GdmbValidationError(
            "E_CAPS", "MANEUVER capability bit is reserved in v1 (provisional)"
        )
    if bool(caps & CAPABILITY_MANEUVER) != bool(features & FEATURE_MANEUVER):
        raise GdmbValidationError(
            "E_CAPS", "MANEUVER capability disagrees with the feature flag"
        )
    if bool(caps & CAPABILITY_REVERSIBLE_THRUST) != bool(
        features & FEATURE_REVERSIBLE_THRUST
    ):
        raise GdmbValidationError(
            "E_CAPS", "REVERSIBLE_THRUST capability disagrees with the feature flag"
        )
    reversible = bool(features & FEATURE_REVERSIBLE_THRUST)

    # Block framing (§4.4)
    blocks: list[tuple[int, int, int]] = []  # (type, payload offset, length)
    pos = header_length
    while pos < size:
        if pos + _BLOCK_HEADER.size > size:
            raise GdmbValidationError("E_FRAMING", f"truncated block header at {pos}")
        block_type, flags, length = _BLOCK_HEADER.unpack_from(data, pos)
        payload = pos + _BLOCK_HEADER.size
        if length > size - payload:
            raise GdmbValidationError("E_FRAMING", f"block at {pos} overruns the file")
        if flags & ~_BLOCK_FLAG_SKIPPABLE:
            raise GdmbValidationError("E_RESERVED", f"reserved flags on block at {pos}")
        if block_type in _FORBIDDEN_BLOCK_TYPES:
            raise GdmbValidationError(
                "E_BLOCK_TYPE", f"block type 0x{block_type:02x} is not valid"
            )
        if not provisional and block_type in _PROVISIONAL_BLOCK_TYPES:
            # Refused whatever its SKIPPABLE flag says (§4.11)
            raise GdmbValidationError(
                "E_BLOCK_TYPE",
                f"block type 0x{block_type:02x} is reserved in v1 (provisional)",
            )
        if block_type in _BLOCK_RANK:
            blocks.append((block_type, payload, length))
        elif not flags & _BLOCK_FLAG_SKIPPABLE:
            raise GdmbValidationError(
                "E_BLOCK_TYPE",
                f"unknown block type 0x{block_type:02x} is not skippable",
            )
        pos = payload + length

    # Canonical order and counts (§4.4)
    ranks = [_BLOCK_RANK[b[0]] for b in blocks]
    if ranks != sorted(ranks):
        raise GdmbValidationError("E_BLOCK_ORDER", "blocks are out of canonical order")
    for block_type in (
        BLOCK_LIGHTS,
        BLOCK_EVENTS,
        BLOCK_RTH_PLAN,
        BLOCK_ENVELOPES,
        BLOCK_COMMENT,
    ):
        if sum(1 for b in blocks if b[0] == block_type) > 1:
            raise GdmbValidationError(
                "E_BLOCK_ORDER", f"block type 0x{block_type:02x} appears twice"
            )
    motion = [b for b in blocks if _BLOCK_RANK[b[0]] == 0]
    if (
        not motion
        or motion[0][0] != BLOCK_FLAT_TRAJECTORY
        or motion[-1][0] != BLOCK_FLAT_TRAJECTORY
    ):
        raise GdmbValidationError(
            "E_BLOCK_ORDER", "motion must start and end with FLAT_TRAJECTORY"
        )
    for a, b in zip(motion, motion[1:], strict=False):
        if a[0] == b[0]:
            raise GdmbValidationError(
                "E_BLOCK_ORDER", "two adjacent motion blocks of the same type"
            )
    num_maneuvers = sum(1 for b in motion if b[0] == BLOCK_MANEUVER)
    if bool(features & FEATURE_MANEUVER) != (num_maneuvers > 0):
        raise GdmbValidationError(
            "E_FEATURES", "MANEUVER feature bit disagrees with the content"
        )

    # Motion blocks
    items: list[_FlatBlock | _ManeuverBlock] = []
    for block_type, offset, length in motion:
        if block_type == BLOCK_FLAT_TRAJECTORY:
            items.append(_parse_flat_block(data, offset, length, reversible=reversible))
        else:
            items.append(
                _parse_maneuver_block(data, offset, length, reversible=reversible)
            )

    uses_reverse_thrust = any(
        seg.sigma < 0
        for item in items
        if isinstance(item, _FlatBlock)
        for seg in item.segments
    ) or any(
        f < 0 for item in items if isinstance(item, _ManeuverBlock) for f in item.thrust
    )
    if reversible != uses_reverse_thrust:
        raise GdmbValidationError(
            "E_FEATURES", "REVERSIBLE_THRUST feature bit disagrees with the content"
        )

    # Timeline: continuity, C² and regime joints (§7.3)
    t = 0
    flat_spans: set[tuple[int, int]] = set()
    branches: dict[int, tuple[tuple[int, ...], _Segment]] = {}
    prev: _FlatBlock | _ManeuverBlock | None = None
    prev_end: tuple[int, ...] = ()
    last_start: tuple[int, ...] = ()  # start state of the last FLAT segment
    for item in items:
        if isinstance(item, _FlatBlock):
            if isinstance(prev, _ManeuverBlock):
                if item.start != prev_end:
                    raise GdmbValidationError(
                        "E_CONTINUITY",
                        f"FLAT block at t={t} us does not start where the maneuver ends",
                    )
                _check_regime_joint(item.start, item.segments[0], prev, entry=False)
            cur: tuple[int, ...] = item.start
            for k, seg in enumerate(item.segments):
                if k > 0:
                    before = item.segments[k - 1]
                    if before.sigma != seg.sigma:
                        raise GdmbValidationError(
                            "E_SIGMA", f"sigma changes at the FLAT joint at t={t} us"
                        )
                    _check_c2_joint(
                        last_start, before, cur, seg, code="E_C2", where=f"t={t} us"
                    )
                    branches[t] = (last_start, before)
                flat_spans.add((t, t + seg.duration_us))
                last_start = cur
                cur = seg.end(cur)
                t += seg.duration_us
            prev_end = cur
        else:
            assert isinstance(prev, _FlatBlock)
            if item.start != prev_end[:3]:
                raise GdmbValidationError(
                    "E_CONTINUITY",
                    f"maneuver at t={t} us does not start where the FLAT block ends",
                )
            _check_regime_joint(last_start, prev.segments[-1], item, entry=True)
            prev_end = (*item.p_hat.end((*item.start,)), item.yaw_exit)
            t += item.duration_us
        prev = item
    if t > _MAX_TIMELINE_US:
        raise GdmbValidationError(
            "E_DURATION", "the timeline is longer than 2^32 - 1 us"
        )

    _check_rest_at_timeline_ends(items)

    # Ancillary blocks
    rth_times: tuple[int, ...] = ()
    for block_type, offset, length in blocks:
        if block_type == BLOCK_LIGHTS:
            if length == 0:
                raise GdmbValidationError("E_LIGHTS", "LIGHTS block is empty")
        elif block_type == BLOCK_EVENTS:
            _check_events(data, offset, length)
        elif block_type == BLOCK_ENVELOPES:
            _check_envelopes(data, offset, length, flat_spans)
        elif block_type == BLOCK_RTH_PLAN:
            rth_times = _check_rth_plan(
                data, offset, length, branches=branches, reversible=reversible
            )

    return GdmbSummary(
        size=size,
        crc32=crc,
        show_hash=header_hash,
        features=features,
        capabilities=caps,
        duration_us=t,
        num_maneuvers=num_maneuvers,
        rth_branch_times_us=rth_times,
    )


def _check_rest_at_timeline_ends(items: list[_FlatBlock | _ManeuverBlock]) -> None:
    """E_REST: the reference is at rest at t = 0 and at the end of the
    timeline, where it meets the rest of §6.1. Velocity, acceleration and yaw
    rate must be zero within the one-sided §7.3 tolerance (what rounding alone
    can leave, plus the fixed slack).
    """
    first, last = items[0], items[-1]
    # The timeline starts and ends with FLAT blocks (E_BLOCK_ORDER)
    assert isinstance(first, _FlatBlock) and isinstance(last, _FlatBlock)

    end_start: tuple[int, ...] = last.start
    for seg in last.segments[:-1]:
        end_start = seg.end(end_start)

    for where, start, seg, at_end in (
        ("start", first.start, first.segments[0], False),
        ("end", end_start, last.segments[-1], True),
    ):
        seconds = seg.duration_us / 1e6
        for k in range(4):
            is_yaw = k == 3
            scale = _YAW_SCALE if is_yaw else _POS_SCALE
            quantum = _Q_YAW if is_yaw else _Q_POS
            n = len(seg.ctrl[k])
            name = "xyzψ"[k]

            velocity = (
                _endpoint_derivative(start[k], seg.ctrl[k], 1, at_end, seg.duration_us)
                / scale
            )
            tol_v = quantum * n / seconds + (_SLACK_YAW_RATE if is_yaw else _SLACK_V)
            if abs(velocity) > tol_v:
                raise GdmbValidationError(
                    "E_REST",
                    f"timeline {where}: d{name}/dt = {velocity:.6g} "
                    f"(tolerance {tol_v:.3g})",
                )

            if is_yaw:
                continue

            acceleration = (
                _endpoint_derivative(start[k], seg.ctrl[k], 2, at_end, seg.duration_us)
                / scale
            )
            tol_a = 2 * quantum * n * (n - 1) / seconds**2 + _SLACK_A
            if abs(acceleration) > tol_a:
                raise GdmbValidationError(
                    "E_REST",
                    f"timeline {where}: d²{name}/dt² = {acceleration:.6g} "
                    f"(tolerance {tol_a:.3g})",
                )


def _check_events(data: bytes, offset: int, length: int) -> None:
    if length % _EVENT.size:
        raise GdmbValidationError("E_EVENTS", "EVENTS length is not a multiple of 10")
    last = 0
    for pos in range(offset, offset + length, _EVENT.size):
        t_us, event_type, _channel, payload = _EVENT.unpack_from(data, pos)
        if t_us < last:
            raise GdmbValidationError("E_EVENTS", "events are not sorted by time")
        if event_type != _EVENT_TYPE_PYRO or payload not in _EVENT_PAYLOADS:
            raise GdmbValidationError("E_EVENTS", f"unsupported event at t={t_us} us")
        last = t_us


def _check_envelopes(
    data: bytes, offset: int, length: int, flat_spans: set[tuple[int, int]]
) -> None:
    if length % _ENVELOPE.size:
        raise GdmbValidationError(
            "E_ENVELOPE", "ENVELOPES length is not a multiple of 12"
        )
    last_start = -1
    for pos in range(offset, offset + length, _ENVELOPE.size):
        start, end, envelope, reserved = _ENVELOPE.unpack_from(data, pos)
        if reserved:
            raise GdmbValidationError("E_RESERVED", "ENVELOPES reserved field is not 0")
        if (start, end) not in flat_spans:
            raise GdmbValidationError(
                "E_ENVELOPE", f"[{start}, {end}) is not the span of a FLAT segment"
            )
        if start <= last_start:
            raise GdmbValidationError(
                "E_ENVELOPE", "ENVELOPES records are not strictly increasing"
            )
        if envelope < 1:
            raise GdmbValidationError("E_ENVELOPE", "zero position envelope")
        last_start = start
