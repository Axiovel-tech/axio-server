"""Fleet pre-flight verification (X-RTLS-VERIFY).

One message runs the whole "are my drones consistent?" rule set that
used to be checked by hand before every flight:

- **geometry** — every tag carries the majority cell geometry
  (delegates to the X-RTLS-GEO check, so origin, ``POS_YAW_DEG``,
  ``CELL_ID`` and the anchor table are all covered);
- **firmware** — uniform firmware version within each role group;
- **pairing** — every online tag is associated with a drone (and every
  drone reached through a tag);
- **yaw-source** — every paired drone's ArduPilot uses the virtual
  compass (``EK3_SRC1_YAW == 9`` on the axio fork) and the fleet agrees
  on ``EK3_SRC_VC_YAW``; the tags' ``POS_YAW_DEG`` values are reported
  alongside so the operator can judge the frame alignment;
- **uwb** — every online tag is actually solving (rate, fix age,
  cluster-clock sync, solve quality from the live stats stream);
- **params** (in-depth only, warnings only) — cross-drone consistency
  of the ArduPilot navigation/tuning set (EKF sources, VISO, WPNAV,
  LOIT, position/attitude controllers, IMU filters).

Rules report ``severity`` (``error`` blocks a flight, ``warning`` is
judgment material) and per-device findings; the response's ``passed``
is true when no error-severity rule failed. ArduPilot parameters are
read live over MAVLink through the UAV driver (no cache), one drone at
a time limit-free but concurrently across drones, with per-drone error
isolation — one unreachable drone never fails the whole run.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Optional

import trio

from .extension import _decoded_device_params
from .geometry import run_check

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("run_verify",)

#: hard ceiling of one ArduPilot parameter read, seconds (the MAVLink
#: driver retries internally on a ~0.7 s cadence)
PARAM_READ_TIMEOUT = 3.0

#: ArduPilot yaw-source parameter and the axio fork's virtual-compass value
YAW_SOURCE_PARAM = "EK3_SRC1_YAW"
YAW_SOURCE_VIRTUAL_COMPASS = 9
VC_YAW_PARAM = "EK3_SRC_VC_YAW"

#: solve-quality thresholds of the uwb rule
MIN_SOLVE_PCT = 80.0
MAX_FIX_AGE_MS = 5000

#: a stats snapshot older than this many seconds is STALE: the stream
#: went silent (even if heartbeats continue) and the tag cannot be
#: certified as solving on it
STATS_FRESH_S = 10.0

#: cross-drone consistency set of the in-depth pass. Values are compared
#: with a small relative tolerance; every difference is a WARNING, never
#: an error — deliberate per-drone tuning exists.
IN_DEPTH_PARAMS = (
    "AHRS_EKF_TYPE",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_VELZ",
    "VISO_TYPE",
    "VISO_DELAY_MS",
    "VISO_POS_M_NSE",
    "VISO_YAW_M_NSE",
    "WPNAV_SPEED",
    "WPNAV_ACCEL",
    "WPNAV_SPEED_UP",
    "WPNAV_SPEED_DN",
    "LOIT_SPEED",
    "LOIT_ACC_MAX",
    "LOIT_BRK_ACCEL",
    "PSC_POSXY_P",
    "PSC_VELXY_P",
    "PSC_VELXY_I",
    "PSC_VELXY_D",
    "ATC_ANG_RLL_P",
    "ATC_ANG_PIT_P",
    "ATC_ANG_YAW_P",
    "INS_GYRO_FILTER",
    "INS_ACCEL_FILTER",
)

#: relative tolerance for float parameter comparisons across drones
PARAM_RELATIVE_TOLERANCE = 1e-6


def _rule(
    rule_id: str,
    label: str,
    severity: str,
    status: str,
    detail: str,
    **extra: Any,
) -> dict[str, Any]:
    return {
        "id": rule_id,
        "label": label,
        "severity": severity,
        "status": status,
        "detail": detail,
        **extra,
    }


async def _read_uav_param(
    uav: Any, name: str
) -> tuple[Optional[float], Optional[str]]:
    """Reads one ArduPilot parameter from a UAV with error isolation:
    returns ``(value, None)`` or ``(None, reason)``."""
    try:
        with trio.fail_after(PARAM_READ_TIMEOUT):
            return float(await uav.get_parameter(name, fetch=True)), None
    except trio.TooSlowError:
        return None, "timeout"
    except Exception as ex:  # noqa: BLE001 — one drone must not kill the run
        return None, str(ex) or type(ex).__name__


def _values_agree(values: list[float]) -> bool:
    lo, hi = min(values), max(values)
    return hi - lo <= PARAM_RELATIVE_TOLERANCE * max(1.0, abs(hi), abs(lo))


def _angles_agree(values: list[float]) -> bool:
    """Wrapped-angle agreement: 0 and 360 are the same yaw (ArduPilot
    applies wrap_360 to the virtual-compass yaw)."""
    if len(values) < 2:
        return True
    reference = values[0] % 360.0
    for value in values[1:]:
        diff = abs((value % 360.0) - reference)
        if min(diff, 360.0 - diff) > 1e-6 * 360.0:
            return False
    return True


def _geometry_rule(geometry: Optional[dict[str, Any]], error: str) -> dict:
    if geometry is None:
        return _rule(
            "geometry",
            "Cell geometry consistency",
            "error",
            "fail",
            error,
        )
    drifted = [
        device_id
        for device_id, entry in geometry["devices"].items()
        if entry.get("status") != "consistent"
    ]
    if drifted:
        return _rule(
            "geometry",
            "Cell geometry consistency",
            "error",
            "fail",
            f"{len(drifted)} tag(s) disagree with the canonical geometry: "
            f"{', '.join(sorted(drifted))} — run a geometry sync",
        )
    return _rule(
        "geometry",
        "Cell geometry consistency",
        "error",
        "pass",
        "every tag matches the canonical geometry",
    )


def _firmware_rule(ext: "RtlsExtension") -> dict[str, Any]:
    protocol = ext._require_protocol()
    versions: dict[str, dict[str, list[int]]] = {}
    for system_id, device in protocol.devices.items():
        adv = ext._adv.get(system_id, {})
        params = _decoded_device_params(device)
        role = adv.get("role") or (
            "tag"
            if not params.get("UWB_ROLE") or params.get("UWB_ROLE") == 1
            else "anchor"
        )
        group = "tag" if str(role).startswith("tag") or role == "tag" else "anchor"
        version = adv.get("firmwareVersion") or params.get("FW_VERSION")
        versions.setdefault(group, {}).setdefault(
            str(version) if version else "unknown", []
        ).append(system_id)

    mixed = {
        group: by_version
        for group, by_version in versions.items()
        if len(by_version) > 1
    }
    if mixed:
        details = "; ".join(
            f"{group}s run {len(by_version)} versions: "
            + ", ".join(
                f"{version} ({', '.join(map(str, sorted(ids)))})"
                for version, ids in sorted(by_version.items())
            )
            for group, by_version in sorted(mixed.items())
        )
        return _rule(
            "firmware",
            "Firmware uniformity",
            "warning",
            "fail",
            details,
        )
    return _rule(
        "firmware",
        "Firmware uniformity",
        "warning",
        "pass",
        "each role group runs a single firmware version",
    )


def _pairing_rule(ext: "RtlsExtension") -> dict[str, Any]:
    protocol = ext._require_protocol()
    online_tags = [
        system_id
        for system_id, device in protocol.devices.items()
        if getattr(device, "last_seen", None) is not None
        and _decoded_device_params(device).get("UWB_ROLE", 1) == 1
    ]
    unpaired = sorted(set(online_tags) - set(ext._uav_map))
    total_uavs: Optional[int] = None
    try:
        from flockwave.server.model.uav import UAV

        registry = getattr(ext.app, "object_registry", None)
        if registry is not None:
            total_uavs = len(list(registry.ids_by_type(UAV)))
    except Exception:  # noqa: BLE001 — registry access is best-effort
        total_uavs = None

    problems = []
    if unpaired:
        problems.append(
            f"tag(s) {', '.join(map(str, unpaired))} are not associated "
            "with any drone"
        )
    if total_uavs is not None and total_uavs > len(ext._uav_map):
        problems.append(
            f"{total_uavs - len(ext._uav_map)} drone(s) have no "
            "associated tag"
        )
    if problems:
        return _rule(
            "pairing",
            "Tag ↔ drone pairing",
            "warning",
            "fail",
            "; ".join(problems),
        )
    return _rule(
        "pairing",
        "Tag ↔ drone pairing",
        "warning",
        "pass",
        f"{len(ext._uav_map)} tag(s) paired"
        + (f", {total_uavs} drone(s) known" if total_uavs is not None else ""),
    )


async def _yaw_rule(ext: "RtlsExtension") -> dict[str, Any]:
    pairs = sorted(ext._uav_map.items())
    if not pairs:
        return _rule(
            "yaw-source",
            "ArduPilot yaw source",
            "error",
            "skipped",
            "no tag↔drone pairs to verify",
        )

    per_drone: dict[str, dict[str, Any]] = {}

    async def read_one(tag_sysid: int, uav_id: str) -> None:
        entry: dict[str, Any] = {"tag": tag_sysid}
        uav = None
        try:
            uav = ext.app.find_uav_by_id(uav_id)
        except Exception:  # noqa: BLE001
            uav = None
        if uav is None:
            entry["error"] = "UAV not found"
        else:
            for key, name in (
                ("yawSource", YAW_SOURCE_PARAM),
                ("vcYaw", VC_YAW_PARAM),
            ):
                value, error = await _read_uav_param(uav, name)
                if error is not None:
                    entry.setdefault("errors", {})[name] = error
                else:
                    entry[key] = value
        device = (
            ext._require_protocol().devices.get(tag_sysid)
            if ext._protocol
            else None
        )
        if device is not None:
            yaw = _decoded_device_params(device).get("POS_YAW_DEG")
            if yaw is not None:
                entry["posYawDeg"] = float(yaw)
        per_drone[uav_id] = entry

    async with trio.open_nursery() as nursery:
        for tag_sysid, uav_id in pairs:
            nursery.start_soon(read_one, tag_sysid, uav_id)

    problems = []
    wrong_source = [
        uav_id
        for uav_id, entry in per_drone.items()
        if "yawSource" in entry
        and int(entry["yawSource"]) != YAW_SOURCE_VIRTUAL_COMPASS
    ]
    if wrong_source:
        problems.append(
            f"drone(s) {', '.join(sorted(wrong_source))} do not use the "
            f"virtual compass ({YAW_SOURCE_PARAM} != "
            f"{YAW_SOURCE_VIRTUAL_COMPASS})"
        )
    vc_values = [
        entry["vcYaw"] for entry in per_drone.values() if "vcYaw" in entry
    ]
    if len(vc_values) > 1 and not _angles_agree(vc_values):
        problems.append(
            f"{VC_YAW_PARAM} differs across the fleet "
            f"({min(vc_values):g} .. {max(vc_values):g})"
        )
    unreadable = [
        uav_id
        for uav_id, entry in per_drone.items()
        if "error" in entry or "errors" in entry
    ]
    if unreadable:
        problems.append(
            f"could not read parameters of drone(s) "
            f"{', '.join(sorted(unreadable))}"
        )

    return _rule(
        "yaw-source",
        "ArduPilot yaw source",
        "error",
        "fail" if problems else "pass",
        "; ".join(problems)
        if problems
        else (
            f"every paired drone uses the virtual compass; "
            f"{VC_YAW_PARAM} agrees"
        ),
        devices=per_drone,
    )


def _uwb_rule(ext: "RtlsExtension") -> dict[str, Any]:
    protocol = ext._require_protocol()
    now = time.monotonic()
    errors: list[str] = []
    warnings: list[str] = []
    for system_id, device in sorted(protocol.devices.items()):
        params = _decoded_device_params(device)
        role = params.get("UWB_ROLE")
        if role is not None and role != 1:
            continue  # anchors have no solver
        stats = ext._stats.get(system_id)
        if not stats:
            errors.append(f"tag {system_id}: no telemetry")
            continue
        stats_age = now - ext._stats_at.get(system_id, float("-inf"))
        if stats_age > STATS_FRESH_S:
            errors.append(
                f"tag {system_id}: telemetry went silent "
                f"({stats_age:.0f} s ago)"
            )
            continue
        if float(stats.get("solveRateHz", 0.0)) <= 0.0:
            errors.append(f"tag {system_id}: not solving")
            continue
        if int(stats.get("fixAgeMs", 0)) > MAX_FIX_AGE_MS:
            warnings.append(
                f"tag {system_id}: stale fix ({stats['fixAgeMs']} ms)"
            )
        if float(stats.get("solvePct", 100.0)) < MIN_SOLVE_PCT:
            warnings.append(
                f"tag {system_id}: solve quality "
                f"{stats['solvePct']:.0f}% < {MIN_SOLVE_PCT:.0f}%"
            )
        if stats.get("clockSyncOk") is False:
            warnings.append(f"tag {system_id}: cluster clock not synced")

    if errors:
        return _rule(
            "uwb",
            "UWB solving state",
            "error",
            "fail",
            "; ".join(errors + warnings),
        )
    if warnings:
        return _rule(
            "uwb",
            "UWB solving state",
            "warning",
            "fail",
            "; ".join(warnings),
        )
    return _rule(
        "uwb",
        "UWB solving state",
        "error",
        "pass",
        "every tag is solving with a fresh fix",
    )


async def _in_depth_rule(ext: "RtlsExtension") -> dict[str, Any]:
    pairs = sorted(ext._uav_map.items())
    if not pairs:
        return _rule(
            "params",
            "ArduPilot parameter consistency",
            "warning",
            "skipped",
            "no paired drones",
        )

    values: dict[str, dict[str, float]] = {}
    unreadable: dict[str, list[str]] = {}

    async def read_all(uav_id: str) -> None:
        try:
            uav = ext.app.find_uav_by_id(uav_id)
        except Exception:  # noqa: BLE001
            uav = None
        if uav is None:
            unreadable[uav_id] = ["*"]
            return
        for name in IN_DEPTH_PARAMS:
            value, error = await _read_uav_param(uav, name)
            if error is None:
                values.setdefault(name, {})[uav_id] = value
            else:
                unreadable.setdefault(uav_id, []).append(name)

    async with trio.open_nursery() as nursery:
        for _, uav_id in pairs:
            nursery.start_soon(read_all, uav_id)

    diffs: dict[str, dict[str, float]] = {}
    for name, by_uav in values.items():
        if len(by_uav) > 1 and not _values_agree(list(by_uav.values())):
            diffs[name] = by_uav

    problems = []
    if diffs:
        problems.append(
            f"{len(diffs)} parameter(s) differ across drones: "
            + ", ".join(sorted(diffs))
        )
    if unreadable:
        problems.append(
            "unreadable: "
            + "; ".join(
                f"{uav_id} ({len(names)} param(s))"
                for uav_id, names in sorted(unreadable.items())
            )
        )
    return _rule(
        "params",
        "ArduPilot parameter consistency",
        "warning",
        "fail" if problems else "pass",
        "; ".join(problems)
        if problems
        else f"{len(IN_DEPTH_PARAMS)} parameters agree across "
        f"{len(pairs)} drone(s)",
        diffs=diffs,
    )


async def run_verify(
    ext: "RtlsExtension", *, in_depth: bool = False
) -> dict[str, Any]:
    """Runs the fleet verification rule set; returns the X-RTLS-VERIFY
    response body. Raises ValueError when another verification is
    already running."""
    if ext._verify_running:
        raise ValueError("A fleet verification is already in progress")
    ext._verify_running = True
    try:
        geometry: Optional[dict[str, Any]] = None
        geometry_error = ""
        try:
            geometry = await run_check(ext)
        except ValueError as ex:
            geometry_error = str(ex)

        rules = [
            _geometry_rule(geometry, geometry_error),
            _firmware_rule(ext),
            _pairing_rule(ext),
            await _yaw_rule(ext),
            _uwb_rule(ext),
        ]
        if in_depth:
            rules.append(await _in_depth_rule(ext))
        else:
            rules.append(
                _rule(
                    "params",
                    "ArduPilot parameter consistency",
                    "warning",
                    "skipped",
                    "in-depth verification not requested",
                )
            )

        body: dict[str, Any] = {
            "type": "X-RTLS-VERIFY",
            "inDepth": bool(in_depth),
            "rules": rules,
            "passed": all(
                rule["status"] != "fail" or rule["severity"] != "error"
                for rule in rules
            ),
        }
        if geometry is not None:
            body["geometry"] = geometry
        return body
    finally:
        ext._verify_running = False
