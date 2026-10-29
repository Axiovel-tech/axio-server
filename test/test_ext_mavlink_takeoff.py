from types import SimpleNamespace

from flockwave.spec.errors import FlockwaveErrorCode

from flockwave.server.ext.mavlink.takeoff import MAVLinkScheduledTakeoffManager
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.takeoff import TakeoffConfiguration
from flockwave.server.model.log import Severity

CONFIG = TakeoffConfiguration(
    authorization_scope=AuthorizationScope.LIVE, takeoff_time=1_760_000_012.75
)


def make_uav(seconds, msec, *, errors=(), **kwds):
    return SimpleNamespace(
        id=kwds.get("id", "1"),
        scheduled_takeoff_authorization_scope=AuthorizationScope.LIVE,
        scheduled_takeoff_time=seconds,
        scheduled_takeoff_time_msec=msec,
        status=SimpleNamespace(errors=set(errors)),
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
    for uav in (in_sync, stale, no_clock, moved):
        uav.send_log_message_to_gcs = lambda message, severity, uav=uav: (
            uav.messages.append((message, severity))
        )

    manager = MAVLinkScheduledTakeoffManager.__new__(MAVLinkScheduledTakeoffManager)
    manager._log = None
    manager.iter_uavs_to_schedule = lambda: iter((in_sync, stale, no_clock, moved))  # type: ignore[method-assign]

    manager.check_start_agreement(CONFIG)

    assert in_sync.messages == []
    assert moved.messages == []
    assert [severity for _, severity in stale.messages] == [Severity.WARNING]
    assert "differs" in stale.messages[0][0]
    assert "show clock" in no_clock.messages[0][0]


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
