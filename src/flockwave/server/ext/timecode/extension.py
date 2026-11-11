"""Runs the configured timecode source and feeds its frames to the ``mtc``
clock (the clock Axio Control offers as the "SMPTE timecode" start
reference). The show extension keeps the show clock synchronized to it.
"""

from __future__ import annotations

import trio

from .clock import TimecodeClock, TimecodeFrame
from .sources import SOURCES
from .timecode import FrameRate

__all__ = ("dependencies", "description", "run", "schema")


async def run(app, configuration, logger):
    source_name = configuration.get("source", "ltc_audio")
    source = SOURCES.get(source_name)
    if source is None:
        logger.error(
            f"Unknown timecode source {source_name!r}; "
            f"choose one of {', '.join(sorted(SOURCES))}"
        )
        return

    rate = FrameRate.from_fps(configuration.get("fps", 25))
    freshness = float(configuration.get("freshness", 1.0))
    clock = TimecodeClock(rate, freshness=freshness)

    def on_frame(frame: TimecodeFrame) -> None:
        was_running = clock.running
        clock.feed(frame)
        if clock.running and not was_running:
            logger.info(f"Timecode running from {frame.timecode}")

    async def watch_freshness() -> None:
        while True:
            await trio.sleep(freshness / 2)
            was_running = clock.running
            clock.check_freshness()
            if was_running and not clock.running and clock.last_frame is not None:
                logger.warning(f"Timecode stopped at {clock.last_frame.timecode}")

    with app.import_api("clocks").use_clock(clock):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(watch_freshness)
            await source(configuration, rate, logger, on_frame)


dependencies = ("clocks",)
description = "SMPTE timecode clock ('mtc') fed by LTC audio or a network feed"
schema = {
    "properties": {
        "source": {
            "type": "string",
            "title": "Timecode source",
            "enum": sorted(SOURCES),
            "default": "ltc_audio",
        },
        "fps": {
            "type": "number",
            "title": "Frame rate",
            "description": "24, 25, 29.97 (drop-frame) or 30",
            "default": 25,
        },
        "freshness": {
            "type": "number",
            "title": "Freshness window (s)",
            "description": "Silence after which the timecode counts as stopped",
            "default": 1.0,
        },
        "device": {
            "type": ["integer", "string", "null"],
            "title": "Audio input device (ltc_audio)",
            "description": "PortAudio device index or name; empty for the default input",
            "default": None,
        },
        "channel": {
            "type": "integer",
            "title": "Audio input channel (ltc_audio)",
            "description": "Channel carrying the LTC, numbered from 1",
            "default": 1,
        },
        "sample_rate": {
            "type": ["number", "null"],
            "title": "Audio sample rate (ltc_audio)",
            "description": "Empty for the device's default rate",
            "default": None,
        },
        "port": {
            "type": "integer",
            "title": "UDP port (udp)",
            "description": "Port receiving one HH:MM:SS:FF timecode per datagram",
            "default": 5005,
        },
    }
}
