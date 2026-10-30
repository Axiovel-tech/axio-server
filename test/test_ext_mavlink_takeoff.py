from types import SimpleNamespace

from flockwave.spec.errors import FlockwaveErrorCode

from flockwave.server.ext.mavlink.takeoff import MAVLinkScheduledTakeoffManager
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.takeoff import TakeoffConfiguration
from flockwave.server.model.log import Severity

CONFIG = TakeoffConfiguration(
    authorization_scope=AuthorizationScope.LIVE, takeoff_time=1_760_000_012.75
)


def make_uav(seconds, msec, *, errors=(), scheduled=True, elapsed=None, **kwds):
    # Status received 10.2 s before the start; the drone floors its elapsed time
    received_at = CONFIG.takeoff_time - 10.2
    status = SimpleNamespace(
        has_scheduled_start=scheduled,
        elapsed_time=-11 if elapsed is None else elapsed,
    )
    return SimpleNamespace(
        id=kwds.get("id", "1"),
        scheduled_takeoff_authorization_scope=AuthorizationScope.LIVE,
        scheduled_takeoff_time=seconds,
        scheduled_takeoff_time_msec=msec,
        show_schedule_status=None,
        status=SimpleNamespace(errors=set(errors)),
        show_status=(status, received_at),
        messages=[],
    )


def test_uav_needs_update_compares_the_milliseconds_too():
    needs_update = MAVLinkScheduledTakeoffManager.uav_needs_update
    assert not needs_update(None, make_uav(1_760_000_012, 750), CONFIG)  # type: ignore[arg-type]
    assert needs_update(None, make_uav(1_760_000_011, 750), CONFIG)  # type: ignore[arg-type]
    assert needs_update(None, make_uav(1_760_000_012, 500), CONFIG)  # type: ignore[arg-type]


async def test_update_uav_writes_the_milliseconds_of_the_start():
    calls = []

    async def set_scheduled_takeoff_time(seconds, msec=0):
        calls.append((seconds, msec))

    uav = make_uav(1_760_000_012, 500)
    uav.set_scheduled_takeoff_time = set_scheduled_takeoff_time

    await MAVLinkScheduledTakeoffManager.update_uav(None, uav, CONFIG)  # type: ignore[arg-type]

    assert calls == [(1_760_000_012, 750)]


def test_start_agreement_check_warns_about_each_uav_out_of_sync():
    in_sync = make_uav(1_760_000_012, 750, id="1")
    stale = make_uav(1_760_000_012, 500, id="2")
    # The start moved by 5 ms (timecode re-anchored); the next broadcast follows
    moved = make_uav(1_760_000_012, 745, id="4")
    no_clock = make_uav(
        1_760_000_012, 750, id="3", errors=[FlockwaveErrorCode.TIMESYNC_ERROR.value]
    )
    # Holds the right start but did not schedule it, or scheduled it a week on
    unscheduled = make_uav(1_760_000_012, 750, id="5", scheduled=False)
    next_week = make_uav(1_760_000_012, 750, id="6", elapsed=-32768)
    uavs = (in_sync, stale, no_clock, moved, unscheduled, next_week)
    for uav in uavs:
        uav.send_log_message_to_gcs = lambda message, severity, uav=uav: (
            uav.messages.append((message, severity))
        )

    manager = MAVLinkScheduledTakeoffManager.__new__(MAVLinkScheduledTakeoffManager)
    manager._log = None
    manager.iter_uavs_to_schedule = lambda: iter(uavs)  # type: ignore[method-assign]

    manager.check_start_agreement(CONFIG)

    assert in_sync.messages == []
    assert moved.messages == []
    assert [severity for _, severity in stale.messages] == [Severity.WARNING]
    assert "differs" in stale.messages[0][0]
    assert "show clock" in no_clock.messages[0][0]
    assert "not scheduled" in unscheduled.messages[0][0]
    assert "own clock puts the start" in next_week.messages[0][0]


async def test_fallback_writes_show_start_msec_with_every_start():
    from flockwave.server.ext.mavlink.driver import MAVLinkUAV

    writes = []
    warnings = []

    async def set_parameter(name, value):
        writes.append((name, value))

    uav = SimpleNamespace(
        id="1",
        set_parameter=set_parameter,
        driver=SimpleNamespace(
            log=SimpleNamespace(warning=lambda *a, **k: warnings.append(a))
        ),
    )

    await MAVLinkUAV.set_scheduled_takeoff_time(uav, 1_760_000_012)  # type: ignore[arg-type]
    assert writes[0] == ("SHOW_START_MSEC", 0)
    assert writes[1][0] == "SHOW_START_TIME"

    writes.clear()
    await MAVLinkUAV.set_scheduled_takeoff_time(uav, None)  # type: ignore[arg-type]
    assert writes == [("SHOW_START_TIME", -1)]

    # Firmware without SHOW_START_MSEC still gets the whole seconds
    async def set_parameter_without_msec(name, value):
        if name == "SHOW_START_MSEC":
            raise RuntimeError("unknown parameter")
        writes.append((name, value))

    uav.set_parameter = set_parameter_without_msec
    await MAVLinkUAV.set_scheduled_takeoff_time(uav, 1_760_000_012, 750)  # type: ignore[arg-type]
    assert writes[-1][0] == "SHOW_START_TIME"
    assert len(warnings) == 1


def test_reported_start_resolves_to_the_nearest_gps_week():
    from flockwave.server.ext.mavlink.driver import nearest_week

    week = 604_800
    now = 1_760_000_000.0
    assert nearest_week(now + 30, now) == now + 30
    # Saturday 23:59:50 -> start at Sunday 00:00:20, read in the current week
    assert nearest_week(now + 30 - week, now) == now + 30
    assert nearest_week(now - 30 + week, now) == now - 30
