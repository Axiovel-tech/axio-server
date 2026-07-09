"""Extension that provides an SMPTE timecode clock named ``mtc`` fed by a
network timecode stream, so a drone show can be scheduled on (and follow)
the master timecode of a larger production.

A hardware LTC reader (e.g. `ltc-tools`' ``ltcdump -f`` on an audio input,
or any console feeding frames) streams one timecode per UDP datagram to
this extension as ``HH:MM:SS:FF`` (drop-frame ``;``/``.``/``,`` separators
are accepted). The extension registers the clock under the id ``mtc`` --
the identifier Skybrush Live already offers as the "SMPTE timecode" start
reference -- and the show extension's clock-synchronization machinery does
the rest: an operator schedules the start at a timecode instant, the show
clock locks to the feed, and the scheduled-start seam converts the
resulting wall-clock start into the drones' GPS-time-of-week
``SHOW_START_TIME`` (real GPS, or the UWB-timebase synthetic GPS pin).

The clock STOPS when the feed goes silent (freshness window): a stopped
primary clock stops the show clock, which is the safe posture for a lost
timecode master. When frames resume or jump (the desk rewinds/scrubs), the
clock emits the corresponding started/changed signals so the
synchronization handler re-aligns -- or refuses to, past its point of no
return.
"""

from __future__ import annotations

import time
from typing import Optional

import trio
import trio.socket

from flockwave.server.model import ClockBase

__all__ = ("TimecodeClock", "parse_timecode", "run")

#: The feed is considered lost after this many seconds without a frame.
DEFAULT_FRESHNESS = 1.0

#: Reported vs extrapolated frame mismatch (in frames) beyond which the
#: feed JUMPED (scrub/rewind) rather than drifted: emit ``changed``.
JUMP_TOLERANCE_FRAMES = 3.0


def parse_timecode(text: str, frame_rate: float) -> Optional[float]:
    """Parses ``HH:MM:SS:FF`` (any of ``:;.,`` before FF) into a frame
    count, or ``None`` if malformed."""
    text = text.strip()
    for sep in (";", ".", ","):
        text = text.replace(sep, ":")
    parts = text.split(":")
    if len(parts) != 4:
        return None
    try:
        hh, mm, ss, ff = (int(p) for p in parts)
    except ValueError:
        return None
    if not (0 <= mm < 60 and 0 <= ss < 60 and 0 <= ff < frame_rate and hh >= 0):
        return None
    return ((hh * 60 + mm) * 60 + ss) * frame_rate + ff


class TimecodeClock(ClockBase):
    """A clock driven by an external timecode feed. Ticks are FRAMES;
    between datagrams the count extrapolates on the server's monotonic
    clock, and it freezes (and reports not running) once the feed goes
    silent for longer than the freshness window."""

    def __init__(self, frame_rate: float = 30.0, freshness: float = DEFAULT_FRESHNESS):
        super().__init__(id="mtc", epoch=None)
        self._frame_rate = float(frame_rate)
        self._freshness = float(freshness)
        self._last_frames: Optional[float] = None
        self._last_wall: float = 0.0

    @property
    def running(self) -> bool:
        return self._running_at(time.time())

    def _running_at(self, now: float) -> bool:
        return (
            self._last_frames is not None
            and self._last_wall > 0
            and now - self._last_wall <= self._freshness
        )

    @property
    def ticks_per_second(self) -> float:
        return self._frame_rate

    def ticks_given_time(self, now: float) -> float:
        if self._last_frames is None:
            return 0.0
        elapsed = now - self._last_wall
        if elapsed > self._freshness:
            return self._last_frames  # feed lost: freeze at the last frame
        return self._last_frames + elapsed * self._frame_rate

    def feed_frames(self, frames: float, now: Optional[float] = None) -> None:
        """Feed one received timecode (in frames). Emits started on the
        first frame after silence and changed on a discontinuous jump."""
        if now is None:
            now = time.time()
        was_running = self._running_at(now)
        expected = self.ticks_given_time(now)
        had_frames = self._last_frames is not None

        self._last_frames = frames
        self._last_wall = now

        if not was_running:
            self.started.send(self)
        if had_frames and abs(frames - expected) > JUMP_TOLERANCE_FRAMES:
            self.changed.send(self, delta=frames - expected)

    def check_freshness(self) -> None:
        """Emit ``stopped`` when the feed just went silent (call periodically)."""
        if self._last_frames is not None and not self.running:
            if self._last_wall > 0:
                self._last_frames = self.ticks_given_time(time.time())
                self._last_wall = 0.0  # sentinel: stop reported
                self.stopped.send(self)


async def run(app, configuration, logger):
    """Runs the extension: listen for timecode datagrams and drive the clock."""
    port = int(configuration.get("port", 5005))
    frame_rate = float(configuration.get("frame_rate", 30))
    freshness = float(configuration.get("freshness", DEFAULT_FRESHNESS))

    clock = TimecodeClock(frame_rate=frame_rate, freshness=freshness)

    sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
    await sock.bind(("0.0.0.0", port))

    logger.info(f"LTC timecode clock 'mtc' listening on UDP :{port} @ {frame_rate} fps")

    async def watchdog():
        while True:
            clock.check_freshness()
            await trio.sleep(freshness / 2)

    with app.import_api("clocks").use_clock(clock):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(watchdog)
            while True:
                data, _addr = await sock.recvfrom(64)
                try:
                    text = data.decode("ascii", "ignore")
                except Exception:  # noqa: BLE001
                    continue
                frames = parse_timecode(text, frame_rate)
                if frames is not None:
                    clock.feed_frames(frames)


dependencies = ("clocks",)
description = "SMPTE timecode clock ('mtc') fed by a network LTC stream"
schema = {
    "properties": {
        "port": {
            "type": "integer",
            "title": "UDP port",
            "description": "Port to listen on for timecode datagrams (HH:MM:SS:FF)",
            "default": 5005,
        },
        "frame_rate": {
            "type": "number",
            "title": "Frame rate",
            "description": "Timecode frames per second",
            "default": 30,
        },
        "freshness": {
            "type": "number",
            "title": "Freshness window (s)",
            "description": "Feed silence after which the clock stops",
            "default": 1.0,
        },
    }
}
