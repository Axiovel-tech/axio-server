"""Wall-clock estimate of the UWB cluster clock.

Tags recover a shared absolute cluster time from the UWB anchor cluster
(the DL-TDoA time-reference anchor) and report it in their health stats
as ``clkh`` + ``clks`` (cluster seconds, split hi/lo for float precision)
with a ``clkok`` freshness flag. The drones count the show start in this
cluster time; the server is the only place that relates it to wall time,
to send the start in cluster time and to read back the starts the drones
hold.

A report reaches the server some latency after the tag sampled its cluster
time, so each report gives an upper bound of the wall time at that cluster
time. The estimate is the smallest such offset over a short window across
all tags (the least delayed report).
"""

from __future__ import annotations

import time
from collections import deque
from logging import Logger
from typing import Any, Optional

__all__ = ("ClusterClock",)

#: A tag's reported cluster seconds may deviate from the estimate's
#: prediction by transport / stats-throttle latency; beyond this the
#: cluster restarted (time-reference anchor power-cycle). Generous vs the
#: ~1 s stats cadence, tiny vs a restart's seconds-to-hours jump.
RESTART_TOLERANCE_S = 5.0

#: Reports older than this do not take part in the offset estimate.
OFFSET_WINDOW_S = 20.0


class ClusterClock:
    """Estimates wall time minus cluster time from the tags' stats reports
    and detects cluster restarts."""

    def __init__(self, log: Optional[Logger] = None):
        self.log = log
        #: Number of cluster restarts seen; each one starts a new timebase.
        self.epoch = 0
        #: (receive wall time, wall minus cluster time) of recent reports
        #: in the current timebase, from every device.
        self._offsets: deque[tuple[float, float]] = deque()
        #: Last distinct cluster-clock sample evaluated per device. The SDK
        #: emits an accumulated stats snapshot for every individual
        #: NAMED_VALUE_FLOAT frame, so the same clkh/clks pair is repeated
        #: while the other fields in the cycle arrive.
        self._last_cluster_seconds: dict[int, float] = {}
        #: Devices whose last distinct sample deviated from the estimate; see
        #: on_stats.
        self._deviating: set[int] = set()

    def estimate(self) -> Optional[float]:
        """Wall time minus cluster time in seconds, from the least delayed
        recent report; ``None`` before the first one."""
        return min(offset for _, offset in self._offsets) if self._offsets else None

    def cluster_us_at(self, wall: float) -> Optional[int]:
        """The cluster time in microseconds at the given UNIX timestamp."""
        offset = self.estimate()
        return None if offset is None else round((wall - offset) * 1e6)

    def wall_at(self, cluster_us: int) -> Optional[float]:
        """The UNIX timestamp at the given cluster time in microseconds."""
        offset = self.estimate()
        return None if offset is None else cluster_us / 1e6 + offset

    def forget_device(self, system_id: int) -> None:
        self._last_cluster_seconds.pop(system_id, None)
        self._deviating.discard(system_id)

    def on_stats(self, system_id: int, data: dict[str, Any]) -> None:
        """Feeds one device stats snapshot."""
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
        now = time.time()

        offset = self.estimate()
        deviation = 0.0 if offset is None else cluster_seconds - (now - offset)
        if abs(deviation) <= RESTART_TOLERANCE_S:
            self._deviating.discard(system_id)
        elif system_id not in self._deviating:
            # clkh and clks arrive in separate frames and the snapshot pairs
            # the latest of each, so a report crossing a 4096 s boundary, or
            # one whose clkh frame was lost, reads 4096 s off until the other
            # half arrives. A restart persists: wait for the next distinct
            # sample before starting a new timebase.
            self._deviating.add(system_id)
            return
        else:
            # Every other device's pending deviation was against the old
            # timebase.
            self._deviating.clear()
            self._offsets.clear()
            self.epoch += 1
            if self.log:
                self.log.warning(
                    f"rtls: cluster time jumped by {deviation:+.1f}s "
                    f"(sysid {system_id}); cluster clock restarted"
                )

        self._offsets.append((now, now - cluster_seconds))
        while self._offsets[0][0] < now - OFFSET_WINDOW_S:
            self._offsets.popleft()
