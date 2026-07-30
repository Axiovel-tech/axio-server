"""Cell-geometry consistency across the tag fleet (X-RTLS-GEO).

Every drone's tag carries its own copy of the cell geometry in its
parameter registry (``ORIGIN_*``, ``POS_YAW_DEG``, ``UWB_AN_COUNT`` and
the ``UWB_AN{i}_*`` anchor table): the UWB solver on each tag positions
against its local copy, so two tags that disagree fly in two different
frames -- a pre-flight hazard that used to be checked and repaired by
hand, one drone at a time.

This module answers that with two operations over one message type:

- ``check`` diffs the geometry of every (or selected) live tag against a
  reference tag and reports per-device verdicts with per-parameter
  deltas;
- ``sync`` writes the reference geometry to the out-of-date tags
  (verified per-parameter acks via the extension's normal PARAM_EXT
  transactions), then reboots the rewritten tags over MCUmgr/SMP so the
  new geometry takes effect -- the firmware reads the anchor table at
  startup.

THE SERVER OWNS THE TRUTH: the canonical geometry of each cell is a
persisted document (``geometry.json`` in the extension's data dir),
adopted once from a known-good tag (or written by the fit) and diffed/
distributed from then on. Deriving truth from the fleet itself --
"whatever most tags happen to carry" -- was the root of a whole class
of consistency machinery this design deletes.

Device state is read from the server's parameter cache (filled on
discovery, kept fresh by the refill poller and by the server's own
writes -- the server is the only writer on the management channel);
``sync``'s device-side acks re-verify reality where it matters.

The write order mirrors the show-clock pin contract: the anchor table
and origin are written first and ``UWB_AN_COUNT`` LAST, so a partially
synced registry never declares a window onto a half-written table.
Activation is the reboot at the end; a device whose writes partially
failed is reported ``partial`` and deliberately NOT rebooted (rebooting
would activate a mixed geometry -- re-run the sync instead).
"""

from __future__ import annotations

import json
import re
import struct
import time
from functools import partial
from typing import TYPE_CHECKING, Any, Mapping, Optional

import trio
from rtlslink import param_type_to_name

from .cell_compat import cell_from_params
from .extension import (
    DEFAULT_PARAM_TIMEOUT,
    _cell_id_from_params,
    _cell_source_role,
    _decoded_device_params,
)

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("run_check", "run_sync")

#: the firmware caps the anchor table at 8 entries
MAX_ANCHORS = 8

#: default tolerance for float parameter comparisons, in the parameter's
#: own unit (meters for the anchor table and biases, degrees for
#: POS_YAW_DEG): synced values round-trip through the same real32
#: encoding and compare exactly, so the tolerance only absorbs float
#: representation noise from values written by other tools
DEFAULT_FLOAT_TOLERANCE = 1e-4

#: timeout of one SMP reset conversation, in seconds
RESET_TIMEOUT = 10.0

#: scalar geometry parameters as ``(name, kind, required)``; optional
#: names (absent on older firmware) are compared/synced only when the
#: reference tag itself carries them
SCALAR_PARAMS: tuple[tuple[str, str, bool], ...] = (
    ("ORIGIN_LAT_E7", "int", True),
    ("ORIGIN_LON_E7", "int", True),
    ("ORIGIN_ALT_MM", "int", True),
    ("POS_YAW_DEG", "float", False),
    ("CELL_ID", "str", False),
    ("UWB_AN_COUNT", "int", True),
)

#: per-anchor table parameters (``UWB_AN{i}_{suffix}``) as
#: ``(suffix, kind, required)``
ANCHOR_SUFFIXES: tuple[tuple[str, str, bool], ...] = (
    ("X", "float", True),
    ("Y", "float", True),
    ("Z", "float", True),
    ("MAC", "int", True),
    ("BIAS_M", "float", False),
)

_ANCHOR_NAME_RE = re.compile(r"^UWB_AN(\d+)_(.+)$")
_SUFFIX_KINDS = {suffix: kind for suffix, kind, _ in ANCHOR_SUFFIXES}
_SCALAR_KINDS = {name: kind for name, kind, _ in SCALAR_PARAMS}


# ---- pure geometry-set helpers ------------------------------------------


def _param_kind(name: str) -> str:
    kind = _SCALAR_KINDS.get(name)
    if kind is not None:
        return kind
    match = _ANCHOR_NAME_RE.match(name)
    if match is not None:
        return _SUFFIX_KINDS.get(match.group(2), "float")
    return "float"


def _coerce(kind: str, value: Any) -> Any:
    if kind == "int":
        return int(value)
    if kind == "float":
        return float(value)
    return str(value)


def extract_geometry(
    params: Mapping[str, Any]
) -> tuple[dict[str, Any], list[str]]:
    """Pulls the geometry subset out of a decoded parameter snapshot.

    Returns ``(subset, missing)`` where ``missing`` lists the REQUIRED
    names the snapshot lacks (optional names are simply left out of the
    subset). An undecodable value counts as missing/absent -- it cannot
    be compared or re-written meaningfully."""
    subset: dict[str, Any] = {}
    missing: list[str] = []

    for name, kind, required in SCALAR_PARAMS:
        if name in params:
            try:
                subset[name] = _coerce(kind, params[name])
                continue
            except (TypeError, ValueError):
                pass
        if required:
            missing.append(name)

    count = subset.get("UWB_AN_COUNT")
    if count is None or not 0 <= count <= MAX_ANCHORS:
        # without a sane count the table cannot be enumerated
        if count is not None:
            subset.pop("UWB_AN_COUNT")
            missing.append("UWB_AN_COUNT")
        return subset, missing

    for index in range(count):
        for suffix, kind, required in ANCHOR_SUFFIXES:
            name = f"UWB_AN{index}_{suffix}"
            if name in params:
                try:
                    subset[name] = _coerce(kind, params[name])
                    continue
                except (TypeError, ValueError):
                    pass
            if required:
                missing.append(name)
    return subset, missing


def _write_order(subset: Mapping[str, Any]) -> list[str]:
    """The canonical write (and diff-report) order of a geometry subset:
    scalars, then the anchor table, then ``UWB_AN_COUNT`` last (see the
    module docstring for why the count trails)."""
    scalars = [
        name
        for name, _, _ in SCALAR_PARAMS
        if name != "UWB_AN_COUNT" and name in subset
    ]
    table = [
        f"UWB_AN{index}_{suffix}"
        for index in range(int(subset.get("UWB_AN_COUNT", 0)))
        for suffix, _, _ in ANCHOR_SUFFIXES
        if f"UWB_AN{index}_{suffix}" in subset
    ]
    tail = ["UWB_AN_COUNT"] if "UWB_AN_COUNT" in subset else []
    return scalars + table + tail


def _values_equal(name: str, expected: Any, actual: Any, tolerance: float) -> bool:
    kind = _param_kind(name)
    try:
        if kind == "float":
            return abs(float(expected) - float(actual)) <= tolerance
        if kind == "int":
            return int(expected) == int(actual)
        return str(expected) == str(actual)
    except (TypeError, ValueError):
        return False


def diff_geometry(
    reference: Mapping[str, Any],
    target: Mapping[str, Any],
    *,
    tolerance: float = DEFAULT_FLOAT_TOLERANCE,
) -> tuple[list[str], dict[str, dict[str, Any]]]:
    """Diffs a target geometry subset against the reference: returns
    ``(missing, deltas)`` where ``missing`` lists reference names absent
    from the target and ``deltas`` maps mismatched names to their
    ``{"expected", "actual"}`` pair. Names only the target carries are
    ignored -- with a lower reference count the extra table rows are
    outside the declared window (and the count itself is compared)."""
    missing: list[str] = []
    deltas: dict[str, dict[str, Any]] = {}
    for name in _write_order(reference):
        if name not in target:
            missing.append(name)
        elif not _values_equal(name, reference[name], target[name], tolerance):
            deltas[name] = {"expected": reference[name], "actual": target[name]}
    return missing, deltas


# ---- reference / target resolution --------------------------------------


def _snapshot_complete(device) -> bool:
    """Whether the server-side param snapshot of a device is
    count-complete."""
    return (
        device.param_count is not None
        and len(device.params) >= device.param_count
    )


def _geometry_fully_present(params: Mapping[str, Any]) -> bool:
    """Whether EVERY geometry parameter — required and optional alike —
    is present in a snapshot, table included."""
    if not all(name in params for name, _, _ in SCALAR_PARAMS):
        return False
    try:
        count = int(params["UWB_AN_COUNT"])
    except (TypeError, ValueError):
        return False
    if not 0 <= count <= MAX_ANCHORS:
        return False
    return all(
        f"UWB_AN{index}_{suffix}" in params
        for index in range(count)
        for suffix, _, _ in ANCHOR_SUFFIXES
    )


def _geometry_knowable(device, params: Mapping[str, Any]) -> bool:
    """Whether a device's geometry can be trusted from its snapshot.

    A count-complete snapshot is authoritative: whatever is absent from
    it is genuinely absent from the registry. An INCOMPLETE snapshot is
    still usable when every geometry parameter — optional ones included
    — happens to be present; but an optional name (``POS_YAW_DEG``,
    ``CELL_ID``, biases) missing from a partial snapshot may be a
    lossy-dump hole rather than genuinely absent, and the refill poller
    only repairs identity params, so such a hole can persist. Treating
    that omission as "not configured" would silently narrow the check
    and report a false ``consistent`` — refuse instead."""
    return _snapshot_complete(device) or _geometry_fully_present(params)


def _snapshot_detail(device) -> str:
    return (
        f"parameter snapshot incomplete "
        f"({len(device.params)}/{device.param_count or '?'} params known) "
        "and the geometry cannot be fully resolved from it"
    )


def _device_role(ext: "RtlsExtension", device) -> Optional[str]:
    """A device's role for targeting purposes: the (fresher) state
    advertisement wins, then the parameter registry -- including the
    legacy compatibility bridge that treats a role-less device carrying a
    complete cell as a tag (mirrors the cell-sourcing logic)."""
    adv_role = ext._adv.get(device.system_id, {}).get("role")
    if adv_role is not None:
        return adv_role
    return _cell_source_role(device, _decoded_device_params(device))


def _store_path(ext: "RtlsExtension"):
    """Path of the canonical-geometry store; ``None`` when the extension
    runs without an app data dir (bare test harness) -- the store is
    then in-memory only."""
    if ext._geo_store_path is not None:
        return ext._geo_store_path
    try:
        return ext.get_data_dir() / "geometry.json"
    except Exception:  # noqa: BLE001 -- harness without app dirs
        return None


def _load_store(ext: "RtlsExtension") -> dict[str, dict[str, Any]]:
    if ext._geo_canonical is None:
        ext._geo_canonical = {}
        path = _store_path(ext)
        if path is not None and path.exists():
            try:
                data = json.loads(path.read_text())
                if isinstance(data, dict):
                    ext._geo_canonical = {
                        str(cell): dict(subset)
                        for cell, subset in data.items()
                        if isinstance(subset, dict)
                    }
            except (OSError, ValueError):
                pass  # a corrupt store must not take the ops down
    return ext._geo_canonical


def _save_store(ext: "RtlsExtension") -> None:
    path = _store_path(ext)
    if path is None:
        return
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(ext._geo_canonical or {}, indent=2))
        tmp.replace(path)
    except OSError:
        pass  # persistence is best-effort; the in-memory copy still rules


def get_canonical(ext: "RtlsExtension", cell):
    """The canonical geometry of a cell as ``(subset, cell_id)``. With
    ``cell`` omitted the single stored cell is used; raises ValueError
    (client-presentable) when the store is empty or ambiguous."""
    store = _load_store(ext)
    if not store:
        raise ValueError(
            "no canonical geometry yet — adopt one from a tag "
            "(op 'adopt') or run a calibration fit"
        )
    if cell is None:
        if len(store) > 1:
            raise ValueError(
                f"multiple cells stored ({', '.join(sorted(store))}); "
                "specify 'cell'"
            )
        cell = next(iter(store))
    if cell not in store:
        raise ValueError(f"No canonical geometry for cell {cell!r}")
    subset, missing = extract_geometry(store[cell])
    if missing:
        raise ValueError(
            f"stored canonical geometry of cell {cell!r} is incomplete "
            f"(missing {', '.join(missing)}) — re-adopt it"
        )
    return subset, cell


def set_canonical(ext: "RtlsExtension", subset):
    """Validates and stores a geometry subset as the canonical geometry
    of its cell (from its ``CELL_ID``, default cell otherwise)."""
    validated, missing = extract_geometry(subset)
    if missing:
        raise ValueError(
            "geometry is incomplete (missing " + ", ".join(missing) + ")"
        )
    cell = _cell_id_from_params(validated)
    store = _load_store(ext)
    store[cell] = dict(validated)
    _save_store(ext)
    return validated, cell


async def run_adopt(
    ext: "RtlsExtension",
    *,
    reference: Optional[int] = None,
    tolerance: float = DEFAULT_FLOAT_TOLERANCE,
) -> dict[str, Any]:
    """Adopts a tag's geometry as the canonical one. With an explicit
    ``reference`` that tag's geometry is taken verbatim; without one the
    fleet must be unanimous (every live tag mutually consistent), so a
    drifted tag can never be adopted by accident."""
    protocol = ext._require_protocol()
    if reference is not None:
        device = protocol.devices.get(reference)
        if device is None:
            raise ValueError(f"No such device: {reference}")
        if _device_role(ext, device) != "tag":
            raise ValueError(f"Device {reference} is not a tag")
        params = _decoded_device_params(device)
        if not _geometry_knowable(device, params):
            raise ValueError(f"Device {reference}: {_snapshot_detail(device)}")
        subset, missing = extract_geometry(params)
        if missing:
            raise ValueError(
                f"Geometry of device {reference} is incomplete "
                f"(missing {', '.join(missing)})"
            )
    else:
        candidates = []
        for system_id, device in sorted(protocol.devices.items()):
            if _device_role(ext, device) != "tag":
                continue
            params = _decoded_device_params(device)
            if not _geometry_knowable(device, params):
                continue
            candidate, missing = extract_geometry(params)
            if not missing:
                candidates.append((system_id, candidate))
        if not candidates:
            raise ValueError("no live tag carries a complete geometry to adopt")
        reference, subset = candidates[0]
        for system_id, other in candidates[1:]:
            fwd = diff_geometry(subset, other, tolerance=tolerance)
            rev = diff_geometry(other, subset, tolerance=tolerance)
            if any((*fwd, *rev)):
                raise ValueError(
                    f"the fleet disagrees (tag {system_id} differs from "
                    f"tag {reference}) — adopt explicitly with 'reference'"
                )

    validated, cell = set_canonical(ext, subset)
    return {
        "type": "X-RTLS-GEO",
        "op": "adopt",
        "cell": cell,
        "reference": reference,
        "paramCount": len(validated),
    }


def _fallback_param_type(ext: "RtlsExtension", name: str):
    """The raw wire type of a parameter, from ANY live device that has it
    cached — the tags run the same firmware family, so the type is
    fleet-wide."""
    protocol = ext._require_protocol()
    for device in protocol.devices.values():
        raw = device.param_types.get(name)
        if raw is not None:
            return raw
    return None


def _target_ids(
    ext: "RtlsExtension", ids: Optional[list[int]], reference_id: int
) -> list[int]:
    """The tags to check/sync: the explicit ``ids`` (minus the reference)
    when given, otherwise every live tag except the reference."""
    if ids is not None:
        return [i for i in dict.fromkeys(ids) if i != reference_id]
    protocol = ext._require_protocol()
    return sorted(
        system_id
        for system_id, device in protocol.devices.items()
        if system_id != reference_id and _device_role(ext, device) == "tag"
    )


# ---- check ---------------------------------------------------------------


async def run_check(
    ext: "RtlsExtension",
    *,
    cell: Optional[str] = None,
    ids: Optional[list[int]] = None,
    tolerance: float = DEFAULT_FLOAT_TOLERANCE,
) -> dict[str, Any]:
    """Diffs the geometry of EVERY live tag against the canonical
    geometry; returns the X-RTLS-GEO ``check`` response body. Raises
    ValueError when no canonical geometry is stored yet."""
    ref_subset, cell_id = get_canonical(ext, cell)
    protocol = ext._require_protocol()

    devices: dict[str, dict[str, Any]] = {}
    for system_id in _target_ids(ext, ids, -1):
        device = protocol.devices.get(system_id)
        if device is None:
            devices[str(system_id)] = {
                "status": "error",
                "detail": f"No such device: {system_id}",
            }
            continue
        role = _device_role(ext, device)
        if role != "tag":
            devices[str(system_id)] = {
                "status": "error",
                "detail": f"Not a tag (role: {role or 'unknown'})",
            }
            continue
        params = _decoded_device_params(device)
        if not _geometry_knowable(device, params):
            # an optional-param omission in a partial snapshot may be a
            # lossy-dump hole; comparing would risk a false "consistent"
            devices[str(system_id)] = {
                "status": "incomplete",
                "detail": _snapshot_detail(device),
            }
            continue
        subset, _ = extract_geometry(params)
        missing, deltas = diff_geometry(ref_subset, subset, tolerance=tolerance)
        if deltas:
            status = "mismatch"
        elif missing:
            status = "incomplete"
        else:
            status = "consistent"
        entry: dict[str, Any] = {"status": status}
        if missing:
            entry["missing"] = missing
        if deltas:
            entry["deltas"] = deltas
        devices[str(system_id)] = entry

    return {
        "type": "X-RTLS-GEO",
        "op": "check",
        "cell": cell_id,
        "consistent": all(
            entry["status"] == "consistent" for entry in devices.values()
        ),
        "devices": devices,
    }


# ---- sync ----------------------------------------------------------------


def _rehome_cell_or_drop(
    ext: "RtlsExtension", cell_id: str, *, exclude: set[int]
) -> None:
    """Re-homes a cell onto some live tag advertising it — skipping the
    ``exclude``d (mixed-geometry) devices — or drops the mapping when no
    clean source exists."""
    protocol = ext._require_protocol()
    for device in protocol.devices.values():
        if device.system_id in exclude:
            continue
        params = _decoded_device_params(device)
        if _cell_source_role(device, params) != "tag":
            continue
        try:
            cell = cell_from_params(
                params, cell_id=_cell_id_from_params(params)
            )
        except (KeyError, TypeError, ValueError):
            continue
        if cell.cell_id == cell_id:
            ext._sync_anchor_beacons(device, params)
            return
    ext._drop_anchor_cell(cell_id)


#: The fixed hardware MCUmgr/SMP port (rtlslink.ota.SMP_PORT). Simulated
#: devices override it per device through the ``devices`` config.
DEFAULT_SMP_PORT = 1337


def _smp_port_for(ext: "RtlsExtension", device) -> int:
    """The SMP port this device's reset must target: the one its config
    entry declared, or the fixed hardware port."""
    ports = getattr(ext, "_smp_ports", None) or {}
    return ports.get(tuple(device.address), DEFAULT_SMP_PORT)


def _default_reset(
    address: str, *, port: int = DEFAULT_SMP_PORT, timeout: float = RESET_TIMEOUT
) -> None:
    """Blocking MCUmgr/SMP os-reset of one device (run in a worker
    thread from Trio) -- the same management surface the OTA path uses
    for its post-upload reset, so no new firmware capability is
    required. Delegates to :func:`rtlslink.ota.reset`, which is
    port-aware (smpclient's high-level client pins :1337, so simulated
    per-device endpoints would otherwise be unreachable) and retries
    before declaring a timeout. Raises on any transport/SMP failure."""
    from rtlslink.ota import reset

    reset(address, port=port, timeout=timeout)


async def _sync_one(
    ext: "RtlsExtension",
    ref_subset: Mapping[str, Any],
    system_id: int,
    *,
    tolerance: float,
    reboot: bool,
    timeout: float,
) -> dict[str, Any]:
    """Writes the reference geometry to one tag; returns its per-device
    result entry. Never raises -- failures land in the entry so one
    device cannot cancel its siblings."""
    protocol = ext._require_protocol()
    device = protocol.devices.get(system_id)
    if device is None:
        return {"status": "error", "detail": f"No such device: {system_id}"}
    role = _device_role(ext, device)
    if role != "tag":
        return {
            "status": "error",
            "detail": f"Not a tag (role: {role or 'unknown'})",
        }

    target_subset, _ = extract_geometry(_decoded_device_params(device))
    written: list[str] = []
    skipped: list[str] = []
    failures: dict[str, str] = {}
    aborted: Optional[str] = None

    for name in _write_order(ref_subset):
        value = ref_subset[name]
        if name in target_subset and _values_equal(
            name, value, target_subset[name], tolerance
        ):
            skipped.append(name)
            continue
        if name == "UWB_AN_COUNT" and failures:
            # the count trails every other write so that a partially
            # synced registry never declares a window onto a mixed
            # table — which means it must ALSO be withheld when any
            # earlier write failed, or the next reboot would activate
            # exactly that mixed window
            failures[name] = (
                "withheld: earlier writes failed (writing the count "
                "would declare a window onto a mixed anchor table)"
            )
            continue
        # the target's own cached type wins; a cache hole falls back to
        # any live tag that knows the type (same firmware family), so a
        # lossy dump cannot block the repair of the very hole it caused
        param_type: Optional[str] = None
        if name not in device.param_types:
            raw_type = _fallback_param_type(ext, name)
            if raw_type is None:
                failures[name] = "parameter type unknown"
                continue
            param_type = param_type_to_name(raw_type)
        try:
            result = await ext.set_param(
                system_id, name, value, param_type, timeout=timeout
            )
        except trio.TooSlowError:
            # a write timeout means the device stopped answering; the
            # remaining writes would each eat the full timeout for the
            # same silence -- abort this device instead
            aborted = f"timeout while writing {name}"
            break
        except (KeyError, ValueError, struct.error) as ex:
            # struct.error: an (explicit-geometry) value outside its wire
            # type must fail THIS parameter, not crash the whole sync
            failures[name] = str(ex)
            continue
        if result["accepted"]:
            written.append(name)
        else:
            failures[name] = (
                f"rejected by device (ack {result['result']}, "
                f"applied value {result['value']!r})"
            )

    entry: dict[str, Any] = {
        "written": written,
        "skipped": skipped,
        "failures": failures,
    }
    if aborted is not None:
        entry["status"] = "error"
        entry["detail"] = aborted
        return entry

    # re-resolve the device: a target lost and rediscovered mid-sync is
    # a NEW object, and set_param mirrored the acks into that one —
    # verifying (or rebooting) the detached pre-rediscovery object would
    # flag phantom drift forever and never activate the geometry
    device = protocol.devices.get(system_id)
    if device is None:
        entry["status"] = "error"
        entry["detail"] = "device lost during the sync"
        return entry

    if not failures:
        # Post-write verification before anything gets activated: the
        # param cache mirrors every device-side ack — ours AND any
        # concurrent writer's (e.g. an interleaved X-RTLS-PARAM-SET) —
        # so a final re-diff catches geometry that drifted between our
        # writes. Anything caught here downgrades to partial and blocks
        # the reboot below.
        final_subset, _ = extract_geometry(_decoded_device_params(device))
        drift_missing, drift = diff_geometry(
            ref_subset, final_subset, tolerance=tolerance
        )
        for name in list(drift_missing) + list(drift):
            failures[name] = (
                "post-write verification: the value drifted during the "
                "sync (concurrent writer?)"
            )
    entry["status"] = "partial" if failures else "synced"

    if reboot:
        rebooted = False
        if failures:
            entry["rebootDetail"] = (
                "not rebooted: some writes failed (rebooting would "
                "activate a mixed geometry; re-run the sync)"
            )
        elif not written:
            entry["rebootDetail"] = "not rebooted: no changes were needed"
        else:
            reset = ext._geo_reset or _default_reset
            try:
                await trio.to_thread.run_sync(
                    partial(
                        reset,
                        device.address[0],
                        port=_smp_port_for(ext, device),
                    )
                )
                rebooted = True
            except ImportError:
                entry["rebootDetail"] = (
                    "smpclient is not installed (pip install "
                    "'rtls-link[ota]'); geometry takes effect on the "
                    "next manual reboot"
                )
            except Exception as ex:  # noqa: BLE001
                entry["rebootDetail"] = f"reset failed: {ex}"
        entry["rebooted"] = rebooted
    return entry


async def run_sync(
    ext: "RtlsExtension",
    *,
    cell: Optional[str] = None,
    ids: Optional[list[int]] = None,
    geometry=None,
    tolerance: float = DEFAULT_FLOAT_TOLERANCE,
    reboot: bool = True,
    timeout: float = DEFAULT_PARAM_TIMEOUT,
) -> dict[str, Any]:
    """Writes the canonical geometry to every (or the selected) tag with
    verified per-parameter acks — one device's failure never cancels
    another — and reboots the fully rewritten tags when ``reboot`` is
    set. An explicit ``geometry`` payload (the fit's apply path) is
    validated, STORED as the new canonical geometry, then distributed
    the same way. Raises ValueError when no canonical geometry exists,
    the payload is incomplete, or another sync is already running (two
    interleaved syncs would race their PARAM_EXT acks per device —
    matching is by device + name only — and could reboot a device the
    other sync is still writing)."""
    if ext._geo_sync_running:
        raise ValueError("A geometry sync is already in progress")
    ext._geo_sync_running = True
    devices: dict[str, dict[str, Any]] = {}
    try:
        if geometry is not None:
            ref_subset, cell_id = set_canonical(ext, geometry)
        else:
            ref_subset, cell_id = get_canonical(ext, cell)

        async def run_one(system_id: int) -> None:
            devices[str(system_id)] = await _sync_one(
                ext,
                ref_subset,
                system_id,
                tolerance=tolerance,
                reboot=reboot,
                timeout=timeout,
            )

        async with trio.open_nursery() as nursery:
            for system_id in _target_ids(ext, ids, -1):
                nursery.start_soon(run_one, system_id)
    finally:
        ext._geo_sync_running = False

    if any(entry.get("written") for entry in devices.values()):
        # geometry changed: push the device list so clients re-render the
        # site anchors without waiting for the slow INF refresh
        await ext._on_inf_change(time.monotonic())

    return {
        "type": "X-RTLS-GEO",
        "op": "sync",
        "cell": cell_id,
        "devices": devices,
    }
