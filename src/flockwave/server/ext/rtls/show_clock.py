"""Cluster->GPS "pin" distribution for the UWB-timebase show clock.

Tags recover a shared absolute cluster time from the UWB anchor cluster
(the DL-TDoA time-reference anchor) and report it in their health stats
as ``clkh`` + ``clks`` (cluster seconds, split hi/lo for float precision)
with a ``clkok`` freshness flag. Given one reference sample paired with
the server's own wall clock, this module mints a *pin* -- "at cluster
tick C0 the GPS time is (week, tow_ms)" -- and distributes the IDENTICAL
pin to every tag as the ``GPS_PIN_*`` parameters.

Each tag then synthesizes the SAME GPS time from its own cluster clock
and feeds it to its autopilot as GPS_INPUT, so ArduPilot's native
GPS-synchronized show scheduler (``SHOW_SYNC_MODE=1`` +
``SHOW_START_TIME``) self-triggers in lockstep across the fleet with no
go-time packet. Inter-drone agreement rides entirely on the shared
cluster clock; the pin's absolute accuracy (server clock + stats
latency, tens of ms) only offsets the show against wall clock, equally
for every drone.

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

import time
from typing import TYPE_CHECKING, Any, Optional

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
    """Mints a pin pairing a tag's reported cluster time with the server's
    wall clock: C0 = the cluster tick for ``cluster_seconds``, (week, tow)
    = the GPS time for ``now_unix``."""
    if now_unix is None:
        now_unix = time.time()
    week, tow_s = unix_to_gps_time_of_week(now_unix)
    return GpsPin(
        week=int(week),
        tow_ms=int(tow_s * 1000.0),
        c0_ticks=round(cluster_seconds / TICK_SECONDS),
        wall_unix=now_unix,
        c0_seconds=cluster_seconds,
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
    push to every device (retry until acked), re-mint + redistribute on a
    detected cluster restart. Driven by the extension's stats events; all
    device I/O is spawned into the extension's nursery so the protocol
    loop never blocks on parameter round-trips."""

    def __init__(self, ext: "RtlsExtension"):
        self._ext = ext
        self._pin: Optional[GpsPin] = None
        #: devices whose LAST push completed with every write accepted
        self._pinned: set[int] = set()
        #: devices with a push currently in flight (dedup guard)
        self._in_flight: set[int] = set()

    @property
    def pin(self) -> Optional[GpsPin]:
        return self._pin

    def reset(self) -> None:
        self._pin = None
        self._pinned.clear()
        self._in_flight.clear()

    def forget_device(self, system_id: int) -> None:
        """A lost device must be re-pinned when it returns (it may have
        rebooted with default params)."""
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
        now_unix = time.time()

        if self._pin is not None:
            predicted = self._pin.predicted_cluster_seconds(now_unix)
            if abs(cluster_seconds - predicted) > RESTART_TOLERANCE_S:
                # The cluster no longer matches the pin (time-reference
                # anchor restarted, or the pin is ancient): every
                # distributed pin is invalid. Mint fresh, push everywhere.
                if self._ext.log:
                    self._ext.log.warning(
                        "rtls: cluster time deviates from the show-clock pin "
                        f"by {abs(cluster_seconds - predicted):.1f}s "
                        f"(sysid {system_id}); re-pinning all devices"
                    )
                self._pin = mint_pin(cluster_seconds, now_unix)
                self._pinned.clear()
        else:
            self._pin = mint_pin(cluster_seconds, now_unix)
            if self._ext.log:
                self._ext.log.info(
                    f"rtls: show-clock pin minted from sysid {system_id} "
                    f"(cluster {cluster_seconds:.3f}s = GPS week "
                    f"{self._pin.week} tow {self._pin.tow_ms} ms)"
                )

        # Push to the reporting device if it does not carry the current pin
        # yet. Stats arrive continuously, so this doubles as the retry path
        # (a failed push simply leaves the device unpinned).
        if system_id not in self._pinned and system_id not in self._in_flight:
            self._in_flight.add(system_id)
            nursery.start_soon(self._push_pin, system_id, self._pin)

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
