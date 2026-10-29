"""Cluster->GPS "pin" distribution for the UWB-timebase show clock.

Tags recover a shared absolute cluster time from the UWB anchor cluster
(the DL-TDoA time-reference anchor) and report it in their health stats
as ``clkh`` + ``clks`` (cluster seconds, split hi/lo for float precision)
with a ``clkok`` freshness flag. This module estimates the offset between
cluster time and the server's wall clock from those reports, mints a
*pin* from it -- "at cluster tick C0 the GPS time is (week, tow_ms)" --
and distributes the IDENTICAL pin to every tag as the ``GPS_PIN_*``
parameters.

Each tag then derives the SAME GPS time from its own cluster clock and
streams it to its autopilot as the show clock, against which the
scheduled show start (in GPS time) is resolved. Inter-drone agreement
rides entirely on the shared cluster clock; the pin's absolute accuracy
only offsets the show against the server's wall clock (and the music it
plays), equally for every drone.

A report reaches the server some latency after the tag sampled its
cluster time, so each report gives an upper bound of the wall time at
that cluster time. The estimate is the smallest such offset over a short
window across all tags (the least delayed report). The pin is re-minted
from it when it drifts away from the distributed pin.

Write-ordering contract (mirrors the firmware param docs): the four pin
params arrive in separate PARAM_EXT transactions, so a (re-)pin writes
``GPS_PIN_WEEK=0`` first and the real week LAST -- a half-applied pin
reads as disabled on the tag instead of mixing old and new C0 halves.

A cluster restart (time-reference anchor power-cycle) makes every
distributed pin meaningless: tags quarantine their cluster clock and
withhold GPS_INPUT on their own, and this module detects the restart
(a tag's reported cluster time far off the pin's prediction) and mints +
redistributes a fresh pin.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import TYPE_CHECKING, Any, Callable, Optional

from flockwave.gps.time import unix_to_gps_time_of_week
from rtlslink import TICK_SECONDS

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("GpsPin", "ShowClockPinManager", "mint_pin", "pin_writes")

#: A tag's reported cluster seconds may deviate from the pin's prediction
#: by transport / stats-throttle latency; beyond this the cluster restarted
#: (or the pin is stale) and a fresh pin must be minted. Generous vs the
#: ~1 s stats cadence, tiny vs a restart's seconds-to-hours jump.
RESTART_TOLERANCE_S = 5.0

#: Reports older than this do not take part in the offset estimate.
OFFSET_WINDOW_S = 20.0

#: All devices are re-pinned when the offset estimate drifts further than
#: this from the distributed pin...
REPIN_DRIFT_S = 0.005

#: ...but at most this often.
REPIN_INTERVAL_S = 60.0

#: ...and not in this many seconds before a scheduled show start: the drones
#: freeze their start ten seconds before it, and a re-pin reaching the tags
#: one by one around then would freeze part of the fleet on the old pin.
REPIN_HOLD_BEFORE_START_S = 60.0

#: Parameter names, in the REQUIRED write order for a (re-)pin.
_PIN_PARAM_TYPES = {
    "GPS_PIN_WEEK": "uint16",
    "GPS_PIN_TOW_MS": "uint32",
    "GPS_PIN_C0_HI": "uint32",
    "GPS_PIN_C0_LO": "uint32",
}


class GpsPin:
    """One minted pin: at cluster tick ``c0_ticks`` the GPS time is
    (``week``, ``tow_ms``). ``wall_unix``/``c0_seconds`` retain the mint
    context for restart prediction."""

    def __init__(
        self, week: int, tow_ms: int, c0_ticks: int, wall_unix: float, c0_seconds: float
    ):
        self.week = week
        self.tow_ms = tow_ms
        self.c0_ticks = c0_ticks
        self.wall_unix = wall_unix
        self.c0_seconds = c0_seconds

    @property
    def offset(self) -> float:
        """Wall time minus cluster time, as distributed."""
        return self.wall_unix - self.c0_seconds

    def predicted_cluster_seconds(self, now_unix: float) -> float:
        """The cluster time a live tag should report around ``now_unix``."""
        return self.c0_seconds + (now_unix - self.wall_unix)

    @property
    def json(self) -> dict[str, Any]:
        return {
            "week": self.week,
            "towMs": self.tow_ms,
            "c0Ticks": self.c0_ticks,
            "mintedAt": self.wall_unix,
        }


def mint_pin(cluster_seconds: float, now_unix: Optional[float] = None) -> GpsPin:
    """Mints a pin pairing ``cluster_seconds`` with the wall time
    ``now_unix``. The pin is anchored on the whole millisecond at or before
    ``now_unix`` (and the matching cluster time) so the millisecond
    resolution of the time of week loses nothing."""
    if now_unix is None:
        now_unix = time.time()
    wall_unix = math.floor(now_unix * 1000.0) / 1000.0
    c0_seconds = cluster_seconds - (now_unix - wall_unix)
    # Half a millisecond in, so float error cannot floor to the previous one.
    week, tow_s = unix_to_gps_time_of_week(wall_unix + 0.0005)
    return GpsPin(
        week=int(week),
        tow_ms=int(tow_s * 1000.0),
        c0_ticks=round(c0_seconds / TICK_SECONDS),
        wall_unix=wall_unix,
        c0_seconds=c0_seconds,
    )


def pin_writes(pin: GpsPin) -> list[tuple[str, int, str]]:
    """The ``(name, value, type)`` parameter writes for one pin, in the
    contract order: week zeroed first, payload, week last."""
    return [
        ("GPS_PIN_WEEK", 0, "uint16"),
        ("GPS_PIN_TOW_MS", pin.tow_ms, "uint32"),
        ("GPS_PIN_C0_HI", (pin.c0_ticks >> 32) & 0xFFFFFFFF, "uint32"),
        ("GPS_PIN_C0_LO", pin.c0_ticks & 0xFFFFFFFF, "uint32"),
        ("GPS_PIN_WEEK", pin.week, "uint16"),
    ]


class ShowClockPinManager:
    """Owns the pin lifecycle: mint on the first fresh cluster sample,
    push to every device (retry until acked), re-mint + redistribute when
    the offset estimate drifts or on a detected cluster restart. Driven by
    the extension's stats events; all device I/O is spawned into the
    extension's nursery so the protocol loop never blocks on parameter
    round-trips."""

    def __init__(
        self,
        ext: "RtlsExtension",
        scheduled_start: Callable[[], Optional[float]] = lambda: None,
    ):
        self._ext = ext
        #: The scheduled show start as a UNIX timestamp, if any.
        self._scheduled_start = scheduled_start
        self._pin: Optional[GpsPin] = None
        #: Wall time when the current pin was minted.
        self._minted_at = 0.0
        #: (receive wall time, wall minus cluster time) of recent reports
        #: that agree with the pin, from every device.
        self._offsets: deque[tuple[float, float]] = deque()
        #: Last distinct cluster-clock sample evaluated per device. The SDK
        #: emits an accumulated stats snapshot for every individual
        #: NAMED_VALUE_FLOAT frame, so the same clkh/clks pair is repeated
        #: while the other fields in the cycle arrive (and while parameter
        #: writes temporarily delay the next clock frame).
        self._last_cluster_seconds: dict[int, float] = {}
        #: Devices whose last distinct sample deviated from the pin; see on_stats.
        self._deviating: set[int] = set()
        #: devices whose LAST push completed with every write accepted
        self._pinned: set[int] = set()
        #: devices with a push currently in flight (dedup guard)
        self._in_flight: set[int] = set()

    @property
    def pin(self) -> Optional[GpsPin]:
        return self._pin

    @property
    def offset_estimate(self) -> Optional[float]:
        """Wall time minus cluster time, from the least delayed recent
        report."""
        return min(offset for _, offset in self._offsets) if self._offsets else None

    def reset(self) -> None:
        self._pin = None
        self._offsets.clear()
        self._last_cluster_seconds.clear()
        self._deviating.clear()
        self._pinned.clear()
        self._in_flight.clear()

    def forget_device(self, system_id: int) -> None:
        """A lost device must be re-pinned when it returns (it may have
        rebooted with default params)."""
        self._last_cluster_seconds.pop(system_id, None)
        self._deviating.discard(system_id)
        self._pinned.discard(system_id)

    def on_stats(self, system_id: int, data: dict[str, Any], nursery) -> None:
        """Feed one device stats snapshot (the extension's ``_on_stats``).
        Synchronous and cheap; spawns any needed pushes into ``nursery``."""
        if data.get("clkok") != 1.0:
            return
        clkh = data.get("clkh")
        clks = data.get("clks")
        if clkh is None or clks is None:
            return
        cluster_seconds = float(clkh) + float(clks)
        if self._last_cluster_seconds.get(system_id) == cluster_seconds:
            return
        self._last_cluster_seconds[system_id] = cluster_seconds
        now_unix = time.time()

        if self._pin is not None:
            predicted = self._pin.predicted_cluster_seconds(now_unix)
            if abs(cluster_seconds - predicted) <= RESTART_TOLERANCE_S:
                self._deviating.discard(system_id)
                self._add_offset(now_unix, cluster_seconds)
                self._repin_if_drifted(now_unix, cluster_seconds)
            elif system_id not in self._deviating:
                # clkh and clks arrive in separate frames and the snapshot pairs
                # the latest of each, so a report crossing a 4096 s boundary, or
                # one whose clkh frame was lost, reads 4096 s off until the other
                # half arrives. A restart persists: wait for the next distinct
                # sample before invalidating every device's pin.
                self._deviating.add(system_id)
                return
            else:
                # Every other device's pending deviation was against the old pin.
                self._deviating.clear()
                # The cluster no longer matches the pin (time-reference
                # anchor restarted, or the pin is ancient): every
                # distributed pin is invalid. Mint fresh, push everywhere.
                if self._ext.log:
                    self._ext.log.warning(
                        "rtls: cluster time deviates from the show-clock pin "
                        f"by {abs(cluster_seconds - predicted):.1f}s "
                        f"(sysid {system_id}); re-pinning all devices"
                    )
                self._offsets.clear()
                self._add_offset(now_unix, cluster_seconds)
                self._mint(cluster_seconds, now_unix)
        else:
            self._add_offset(now_unix, cluster_seconds)
            pin = self._mint(cluster_seconds, now_unix)
            if self._ext.log:
                self._ext.log.info(
                    f"rtls: show-clock pin minted from sysid {system_id} "
                    f"(cluster {cluster_seconds:.3f}s = GPS week "
                    f"{pin.week} tow {pin.tow_ms} ms)"
                )

        # Push to the reporting device if it does not carry the current pin
        # yet. Stats arrive continuously, so this doubles as the retry path
        # (a failed push simply leaves the device unpinned).
        if system_id not in self._pinned and system_id not in self._in_flight:
            self._in_flight.add(system_id)
            nursery.start_soon(self._push_pin, system_id, self._pin)

    def _add_offset(self, now_unix: float, cluster_seconds: float) -> None:
        self._offsets.append((now_unix, now_unix - cluster_seconds))
        while self._offsets[0][0] < now_unix - OFFSET_WINDOW_S:
            self._offsets.popleft()

    def _mint(self, cluster_seconds: float, now_unix: float) -> GpsPin:
        """Mints a pin for ``cluster_seconds`` from the offset estimate and
        marks every device as needing it."""
        offset = self.offset_estimate
        assert offset is not None
        self._pin = mint_pin(cluster_seconds, cluster_seconds + offset)
        self._minted_at = now_unix
        self._pinned.clear()
        return self._pin

    def _repin_if_drifted(self, now_unix: float, cluster_seconds: float) -> None:
        assert self._pin is not None
        offset = self.offset_estimate
        if offset is None or now_unix - self._minted_at < REPIN_INTERVAL_S:
            return
        start = self._scheduled_start()
        if start is not None and 0 <= start - now_unix <= REPIN_HOLD_BEFORE_START_S:
            return
        drift = offset - self._pin.offset
        if abs(drift) > REPIN_DRIFT_S:
            self._mint(cluster_seconds, now_unix)
            if self._ext.log:
                self._ext.log.info(
                    f"rtls: cluster-to-wall offset moved {drift * 1000:+.1f} ms "
                    "from the show-clock pin; re-pinning all devices"
                )

    async def _push_pin(self, system_id: int, pin: GpsPin) -> None:
        try:
            for name, value, param_type in pin_writes(pin):
                result = await self._ext.set_param(system_id, name, value, param_type)
                if not result.get("accepted"):
                    if self._ext.log:
                        self._ext.log.warning(
                            f"rtls: sysid {system_id} rejected {name}; "
                            "show-clock pin left disabled on the device"
                        )
                    return
            # Only mark pinned if this push delivered the CURRENT pin (a
            # re-mint may have raced this coroutine).
            if self._pin is pin:
                self._pinned.add(system_id)
                if self._ext.log:
                    self._ext.log.info(
                        f"rtls: show-clock pin distributed to sysid {system_id}"
                    )
        except KeyError:
            pass  # device vanished mid-push; rediscovery re-pins it
        except Exception as ex:  # noqa: BLE001 -- timeouts and transport errors
            if self._ext.log:
                self._ext.log.warning(
                    f"rtls: show-clock pin push to sysid {system_id} failed: {ex}"
                )
        finally:
            self._in_flight.discard(system_id)
