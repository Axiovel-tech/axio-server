"""Tests for the LTC timecode clock extension."""

from flockwave.server.ext.ltc_timecode import (
    TimecodeClock,
    parse_timecode,
)

T0 = 1_752_000_000.0


def test_parse_timecode():
    assert parse_timecode("01:00:00:00", 30) == 108_000
    assert parse_timecode("00:00:01:15", 30) == 45
    assert parse_timecode("00:59:30;00", 30) == (59 * 60 + 30) * 30  # drop-frame sep
    assert parse_timecode("10:00:00.29", 30) == 10 * 3600 * 30 + 29
    assert parse_timecode("garbage", 30) is None
    assert parse_timecode("00:00:00:31", 30) is None  # frame >= rate
    assert parse_timecode("00:61:00:00", 30) is None


def test_clock_runs_and_extrapolates_between_frames():
    clock = TimecodeClock(frame_rate=30, freshness=1.0)
    assert clock.ticks_given_time(T0) == 0.0

    clock.feed_frames(3000.0, now=T0)
    assert clock.ticks_given_time(T0) == 3000.0
    # half a second later, +15 frames extrapolated
    assert clock.ticks_given_time(T0 + 0.5) == 3015.0
    # ticks/seconds contract
    assert clock.ticks_per_second == 30


def test_clock_freezes_when_feed_lost():
    clock = TimecodeClock(frame_rate=30, freshness=1.0)
    clock.feed_frames(3000.0, now=T0)
    # beyond the freshness window the count freezes at the last frame
    assert clock.ticks_given_time(T0 + 5.0) == 3000.0


def test_clock_signals_started_and_jump():
    clock = TimecodeClock(frame_rate=30, freshness=1.0)
    events = []
    # blinker holds weak references by default; keep the receivers strong
    clock.started.connect(
        lambda s, **kw: events.append(("started",)), sender=clock, weak=False
    )
    clock.changed.connect(
        lambda s, **kw: events.append(("changed", kw.get("delta"))),
        sender=clock,
        weak=False,
    )

    clock.feed_frames(3000.0, now=T0)
    assert ("started",) in events
    events.clear()

    # steady feed: one frame later, no jump signal
    clock.feed_frames(3003.0, now=T0 + 0.1)
    assert not [e for e in events if e[0] == "changed"]

    # the desk scrubs backward by 10 seconds -> changed with negative delta
    clock.feed_frames(3003.0 - 300.0, now=T0 + 0.2)
    jumps = [e for e in events if e[0] == "changed"]
    assert len(jumps) == 1
    assert jumps[0][1] < -290
