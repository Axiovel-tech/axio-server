"""Live end-to-end check of the X-RTLS-GEO anchor-geometry calibration
against REAL rtls-link firmware (native_sim): eight anchors on a known
two-rectangle four-tripod geometry feed responder-owned A0 TWR summaries,
and the server-side fit must recover the true dimensions.

Not collected by pytest (no ``test_`` prefix): it boots nine real
firmware processes and talks to them over real UDP sockets. Run
manually from the repo root, with the branch SDK (the ``twr_summary``
event and the sim runner live there, not in the pinned rtls-link):

    RTLS_FW_TAG=<fw-worktree>/build-e2e-tag/tag/zephyr/zephyr.exe \\
    RTLS_FW_ANCHOR=<fw-worktree>/build-e2e-anchor/anchor/zephyr/zephyr.exe \\
    uv run --with-editable <fw-worktree>/py \\
        python test/e2e/rtls_calibration_live.py

The images are the plain sysbuild native_sim builds of the two apps,
built inside the firmware repo's dev container:

    docker compose -f docker/compose.yml run --rm dev \\
        west build --sysbuild -p -b native_sim/native/64 \\
        -d build-e2e-anchor app/anchor
    docker compose -f docker/compose.yml run --rm dev \\
        west build --sysbuild -p -b native_sim/native/64 \\
        -d build-e2e-tag app/tag

Bench isolation: every instance boots on a NON-default management port
(``RTLS_E2E_MGMT_PORT`` base, default 13333) so a live skybrushd on the
same host -- which broadcast-probes :3333 -- never discovers the sim,
the extension's :3343 advertisement listener is disabled (the
native_sim image has no phone-home module anyway), the scenario has no
``gcs_out`` (nothing touches :14550), and the UWB ether runs on a
non-default multicast group/port so a concurrently live sim stack
cannot cross-talk.

Sequence:

1.  boot 8 anchor instances via the SDK's single-start SimRunner on the
    reference two-rectangle geometry (L=12, W=9, H=2: lower rectangle
    A0(0,0,0) A1(12,0,0) A2(0,9,0) A3(12,9,0); upper A4..A7 at the same
    XY, z=-2). The runner pushes the cell over PARAM_EXT after boot;
    each anchor self-locates from the table by its UWB_MAC, so the
    pushed table IS the simulated TRUE geometry. A0 (UWB_MAC ==
    UWB_AN0_MAC) runs UWB_ROLE=2 (initiator), A1..A7 run UWB_ROLE=3;
    plus one tag image (no SITL);
2.  run the server RtlsExtension against all nine devices, wait for
    discovery + count-complete param snapshots;
3.  wait for coherent ``twr_summary`` events from A1..A7 (trcap == 1),
    assert that each carries only its A0 spoke at ~1 Hz with count >= 20,
    and assert that A0 publishes none;
4.  adopt the real tag's geometry as the canonical one (it carries the
    8-anchor MAC table the fit resolves spokes against);
5.  ``run_fit(mode='strict')`` -> accepted, lengthM/widthM/heightM
    within tolerance of 12/9/2 (SIM_UWB_NOISE_M defaults to a 0.05 m
    per-sample sigma; the robust per-pair medians of >= 20 samples
    leave well under 0.05 m of aggregate error), applyGeometry present
    with UWB_AN7_Z ~= -2 and POS_YAW_DEG == 0;
6.  ``run_fit(mode='refined', capture_id=<pinned>)`` -> reuses the
    pinned distributed capture and reports no meaningful improvement (the
    nominal build has no construction error);
7.  apply: ``run_sync`` of the strict applyGeometry to the real tag
    (reboot off: native_sim has no SMP reset surface) -> every write
    acked -> ``run_check`` consistent, also after a fresh re-list from
    the device (device truth, not server-cache optimism);
8.  negative: SIGTERM one responder anchor, wait for it to be declared
    lost, then a strict fit must fail naming the missing anchor slot;
9.  tear the whole stack down (the runner only ever signals PIDs whose
    /proc cmdline matches this run's firmware+flash).
"""

from __future__ import annotations

import logging
import os
import shutil
import signal
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from statistics import median

import trio

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from rtlslink.cell import Anchor, Origin, RtlsCell  # noqa: E402
from rtlslink.sim import (  # noqa: E402
    AnchorDevice,
    EtherGroup,
    SimDevice,
    SimRunner,
    SimScenario,
)

from flockwave.server.ext.rtls.extension import (  # noqa: E402
    RtlsExtension,
    _decoded_device_params,
)
from flockwave.server.ext.rtls.fit import run_fit  # noqa: E402
from flockwave.server.ext.rtls.geometry import (  # noqa: E402
    run_adopt,
    run_check,
    run_sync,
)

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger("e2e")

MGMT_BASE = int(os.environ.get("RTLS_E2E_MGMT_PORT", "13333"))

TAG_SYSID = 199
A0_SYSID = 201

#: the true two-rectangle four-tripod geometry (NED, meters)
LENGTH_M = 12.0
WIDTH_M = 9.0
HEIGHT_M = 2.0
TRUE_POSITIONS = (
    (0.0, 0.0, 0.0),
    (LENGTH_M, 0.0, 0.0),
    (0.0, WIDTH_M, 0.0),
    (LENGTH_M, WIDTH_M, 0.0),
    (0.0, 0.0, -HEIGHT_M),
    (LENGTH_M, 0.0, -HEIGHT_M),
    (0.0, WIDTH_M, -HEIGHT_M),
    (LENGTH_M, WIDTH_M, -HEIGHT_M),
)

#: fitted-dimension tolerance: the sim adds a zero-mean 0.05 m-sigma
#: Gaussian per sample (SIM_UWB_NOISE_M default) and the firmware
#: aggregates >= 20 samples per pair with a MAD-rejecting median, so the
#: per-spoke error is ~0.01 m; 0.05 m is a comfortable bound
DIM_TOLERANCE_M = 0.05

#: rolling-summary contract mirrored from the firmware: each responder
#: publishes one A0 spoke at 1 Hz with >= 20 samples and peer mask bit 0.
SUMMARY_MIN_COUNT = 20
SUMMARY_A0_MASK = 0b0000_0001
CADENCE_GENERATIONS = 3
#: generous cadence bounds around the nominal 1 Hz: a concurrent
#: twister/build matrix on this host can stall a 1 Hz tick briefly
CADENCE_MIN_S = 0.5
CADENCE_MAX_S = 2.0


class _StubHub:
    def __init__(self):
        self.broadcasts = []

    @contextmanager
    def use_message_handlers(self, handlers):
        yield

    def create_notification(self, body=None):
        return body

    async def broadcast_message(self, message):
        self.broadcasts.append(message)

    def enqueue_broadcast_message(self, message):
        self.broadcasts.append(message)


class _StubApp:
    def __init__(self):
        self.message_hub = _StubHub()

    def import_api(self, name):
        raise KeyError(name)  # no sibling extensions in this harness


def _scenario() -> SimScenario:
    """The calibration cell: 1 tag + 8 anchors, isolated ports/ether."""
    cell = RtlsCell(
        Origin(413900000, 21500000, 12000),
        [
            Anchor(index, north, east, down, mac=index + 1)
            for index, (north, east, down) in enumerate(TRUE_POSITIONS)
        ],
        cell_id="e2e-calib",
    )
    tag = SimDevice(
        name="tag0",
        role="tag",
        sysid=TAG_SYSID,
        uwb_mac=0x00FE,
        mgmt_port=MGMT_BASE,
        sitl=False,
    )
    anchors = tuple(
        AnchorDevice(
            slot=anchor.index,
            device=SimDevice(
                name=f"anchor{anchor.index}",
                role="anchor-initiator" if anchor.index == 0 else "anchor-responder",
                sysid=A0_SYSID + anchor.index,
                uwb_mac=int(anchor.mac),
                mgmt_port=MGMT_BASE + 1 + anchor.index,
            ),
        )
        for anchor in cell.anchors
    )
    return SimScenario(
        "calibration-e2e",
        cell,
        (tag,),
        anchors,
        # non-default group/port: a concurrently live default sim stack
        # (239.255.42.1:45454) must not cross-talk into this cell
        EtherGroup(group="239.255.42.31", port=46454),
        gcs_out=None,  # no SITL vehicles, and nothing may touch :14550
    )


async def _wait_discovery(
    ext: RtlsExtension, expected: set[int], timeout: float = 120.0
) -> None:
    with trio.fail_after(timeout):
        while not expected <= set(ext._get_devices()):
            await trio.sleep(0.2)


async def _ensure_snapshot(
    ext: RtlsExtension, sysid: int, timeout: float = 20.0
) -> None:
    """Waits until the device's param snapshot is count-complete; nine
    boards dumping at once over UDP is lossy, so fall back to an explicit
    re-list (the refill poller only repairs identity params)."""
    with trio.move_on_after(timeout):
        while True:
            device = ext._get_devices().get(sysid)
            if (
                device is not None
                and device.param_count is not None
                and len(device.params) >= device.param_count
            ):
                return
            await trio.sleep(0.2)
    log.info("sysid %d snapshot incomplete after discovery; re-listing", sysid)
    await ext.get_param_list(sysid, timeout=60.0)


async def _relist(ext: RtlsExtension, sysid: int) -> None:
    """Drops the server-side param cache of the device and re-fills it
    from the live device, so a subsequent check reflects device truth."""
    device = ext._get_devices()[sysid]
    device.params.clear()
    device.param_types.clear()
    device.param_count = None
    await ext.get_param_list(sysid, timeout=60.0)


def _summary_complete(summary) -> bool:
    """One coherent responder generation carrying one well-sampled A0 spoke."""
    macs = {item.peer_mac for item in summary.ranges}
    return (
        summary.version == 1  # trcap: rolling-summary protocol v1
        and summary.valid_mask == SUMMARY_A0_MASK
        and macs == {1}
        and all(item.count >= SUMMARY_MIN_COUNT for item in summary.ranges)
    )


async def _observe_summaries(ext: RtlsExtension) -> None:
    """Step 3: one coherent A0-spoke stream per responder at ~1 Hz."""
    responder_sysids = range(A0_SYSID + 1, A0_SYSID + 8)
    # warm-up: the window needs >= 20 samples per pair before a spoke is
    # published at all, and nine freshly booted instances contend for CPU
    with trio.fail_after(120):
        while True:
            if all(
                (summary := ext._twr_summaries.get(system_id)) is not None
                and _summary_complete(summary)
                for system_id in responder_sysids
            ):
                break
            await trio.sleep(0.1)
    assert A0_SYSID not in ext._twr_summaries, "A0 must not publish TWR summaries"

    seen = {
        system_id: [ext._twr_summaries[system_id]] for system_id in responder_sysids
    }
    with trio.fail_after(30):
        while any(
            len(generations) < CADENCE_GENERATIONS for generations in seen.values()
        ):
            for system_id, generations in seen.items():
                summary = ext._twr_summaries.get(system_id)
                if summary is not None and summary.sequence != generations[-1].sequence:
                    generations.append(summary)
            await trio.sleep(0.05)

    cadences = []
    counts = []
    for system_id, generations in seen.items():
        for summary in generations:
            assert _summary_complete(summary), (
                f"sysid {system_id} incomplete generation "
                f"{summary.sequence}: {summary.ranges}"
            )
        sequences = [summary.sequence for summary in generations]
        assert sequences == sorted(sequences), sequences
        intervals = [
            later.received_at - earlier.received_at
            for earlier, later in zip(generations, generations[1:])
        ]
        cadence = median(intervals)
        assert CADENCE_MIN_S <= cadence <= CADENCE_MAX_S, (
            f"sysid {system_id} cadence {cadence:.2f} s is not ~1 Hz "
            f"(intervals {intervals})"
        )
        cadences.append(cadence)
        counts.append(generations[-1].ranges[0].count)
    log.info(
        "responder summaries: %d streams x %d generations, "
        "median interval %.2f s, spoke counts %s",
        len(seen),
        CADENCE_GENERATIONS,
        median(cadences),
        counts,
    )


async def _run_strict_fit(ext: RtlsExtension) -> dict:
    """One strict fit, retrying only a summary-freshness timeout (a
    concurrent build matrix can stall the 1 Hz publish tick past the
    fit wait); every other failure propagates untouched."""
    for attempt in range(3):
        try:
            return await run_fit(ext, mode="strict")
        except ValueError as ex:
            if "no fresh rolling TWR summary" in str(ex) and attempt < 2:
                log.warning("strict fit retried: %s", ex)
                continue
            raise
    raise AssertionError("unreachable")


async def _checks(ext: RtlsExtension, manifest: dict) -> None:
    all_sysids = {TAG_SYSID, *range(A0_SYSID, A0_SYSID + 8)}
    await _wait_discovery(ext, all_sysids)
    log.info("all %d devices discovered", len(all_sysids))
    for sysid in sorted(all_sysids):
        await _ensure_snapshot(ext, sysid)
    log.info("param snapshots complete")

    # The configured A0 is still the unique initiator, but the responders
    # own and publish the seven measurements.
    a0_params = _decoded_device_params(ext._get_devices()[A0_SYSID])
    assert int(a0_params["UWB_ROLE"]) == 2, a0_params.get("UWB_ROLE")
    assert int(a0_params["UWB_MAC"]) == 1, a0_params.get("UWB_MAC")

    await _observe_summaries(ext)

    # adopt the canonical geometry from the real tag: it carries the
    # 8-anchor MAC table (pushed by the runner) the fit resolves against
    adopted = await run_adopt(ext, reference=TAG_SYSID)
    assert adopted["cell"] == "e2e-calib", adopted
    log.info("adopted canonical geometry from tag (%s)", adopted)

    # strict fit: recover the true dimensions from the live summaries
    fit = await _run_strict_fit(ext)
    strict = fit["strict"]
    assert fit["mode"] == "strict" and fit["selectedModel"] == "strict", fit
    assert strict["accepted"] is True, strict
    parameters = strict["parameters"]
    for name, expected in (
        ("lengthM", LENGTH_M),
        ("widthM", WIDTH_M),
        ("heightM", HEIGHT_M),
    ):
        assert abs(parameters[name] - expected) <= DIM_TOLERANCE_M, (
            f"{name}={parameters[name]} is not within {DIM_TOLERANCE_M} m of {expected}"
        )
    payload = fit["applyGeometry"]
    assert payload is not None, fit
    assert payload["POS_YAW_DEG"] == 0.0, payload
    assert abs(payload["UWB_AN7_Z"] + HEIGHT_M) <= DIM_TOLERANCE_M, payload
    log.info(
        "strict fit accepted: lengthM=%.3f widthM=%.3f heightM=%.3f rmsM=%.4f",
        parameters["lengthM"],
        parameters["widthM"],
        parameters["heightM"],
        strict["rmsM"],
    )

    # refined fit against the PINNED capture: the nominal build has no
    # construction error, so no meaningful improvement may be reported
    capture_id = fit["summary"]["captureId"]
    refined = await run_fit(ext, mode="refined", capture_id=capture_id)
    assert refined["summary"]["captureId"] == capture_id, refined["summary"]

    def provenance(source):
        return (
            source["anchorIndex"],
            source["systemId"],
            source["sequence"],
            source["timeBootMs"],
        )

    assert [provenance(item) for item in refined["summary"]["sources"]] == [
        provenance(item) for item in fit["summary"]["sources"]
    ], "the refined fit must reuse the pinned device generations"
    comparison = refined["comparison"]
    assert comparison["meaningfulImprovement"] is False, comparison
    log.info("refined comparison: %s", comparison)

    # apply: distribute the strict geometry to the real tag with
    # verified acks (reboot off: native_sim has no SMP reset surface)
    sync = await run_sync(ext, geometry=payload, reboot=False)
    entry = sync["devices"][str(TAG_SYSID)]
    assert entry["status"] == "synced", entry
    assert not entry["failures"], entry
    log.info(
        "sync: wrote %d params to the tag (%d already consistent)",
        len(entry["written"]),
        len(entry["skipped"]),
    )
    report = await run_check(ext)
    assert report["consistent"] is True, report
    # ... and it must STILL agree when the cache is rebuilt from the
    # live device (device truth, not server optimism)
    await _relist(ext, TAG_SYSID)
    report = await run_check(ext)
    assert report["consistent"] is True, report
    log.info("post-apply check from a fresh device dump: consistent")

    # negative: a dead responder must fail the fit BY NAME, not skew it
    victim = next(
        process for process in manifest["processes"] if process["name"] == "anchor7"
    )
    os.killpg(int(victim["pid"]), signal.SIGTERM)
    log.info("killed anchor7 (pid %s)", victim["pid"])
    with trio.fail_after(60):
        while A0_SYSID + 7 in ext._get_devices():
            await trio.sleep(0.2)
    try:
        await _run_strict_fit(ext)
    except ValueError as ex:
        nak = str(ex)
    else:
        raise AssertionError("strict fit must refuse a missing responder")
    assert "A7" in nak and "not online" in nak, nak
    log.info("missing-anchor NAK: %s", nak)


async def _drive(tag_binary: str, anchor_binary: str) -> None:
    run_dir = Path(tempfile.mkdtemp(prefix="rtls-e2e-calib-"))
    runner = SimRunner(
        _scenario(),
        tag_firmware=tag_binary,
        anchor_firmware=anchor_binary,
        run_dir=run_dir,
    )
    passed = False
    try:
        log.info("booting 1 tag + 8 anchors (run dir %s)", run_dir)
        manifest = await runner.up(replace=True, timeout=60.0)

        ext = RtlsExtension()
        app = _StubApp()
        ext.app = app  # the extension framework sets this before run()
        async with trio.open_nursery() as nursery:

            async def run_ext():
                await ext.run(
                    app,
                    {
                        "devices": list(manifest["server_devices"]),
                        "broadcast": [],
                        "port": MGMT_BASE,
                        "advertisement_port": 0,  # never bind :3343
                        "register_beacons": False,
                        "heartbeat_interval": 0.5,
                        "device_timeout": 10,
                    },
                    logging.getLogger("rtls"),
                )

            nursery.start_soon(run_ext)
            await _checks(ext, manifest)
            nursery.cancel_scope.cancel()
        passed = True
    finally:
        runner.down()
        if passed:
            shutil.rmtree(run_dir, ignore_errors=True)
        else:
            log.error("FAILED: firmware logs preserved under %s", run_dir)


def main() -> int:
    tag = os.environ.get("RTLS_FW_TAG")
    anchor = os.environ.get("RTLS_FW_ANCHOR")
    if not tag or not anchor or not Path(tag).exists() or not Path(anchor).exists():
        print("set RTLS_FW_TAG / RTLS_FW_ANCHOR to built native_sim binaries")
        return 2

    # The RTLS_LINK_SIM_* env overrides WIN over the per-instance boot
    # config file (sim_config.cpp); an inherited RTLS_LINK_SIM_MGMT_PORT
    # would collapse all nine instances onto one port. The runner's
    # config files are the single interface here.
    for name in [key for key in os.environ if key.startswith("RTLS_LINK_SIM_")]:
        del os.environ[name]

    trio.run(_drive, tag, anchor)
    print(
        "E2E OK: rolling-summary calibration fit verified against live "
        "anchor+tag firmware"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
