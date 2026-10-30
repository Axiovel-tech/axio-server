import logging
from struct import pack
from time import time
from types import SimpleNamespace

from flockwave.server.ext.mavlink.packets import ShowScheduleStatus
from flockwave.server.ext.mavlink.schedule import ShowSchedule
from flockwave.server.ext.mavlink.takeoff import MAVLinkScheduledTakeoffManager
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.takeoff import TakeoffConfiguration
from flockwave.server.model.log import Severity

CONFIG = TakeoffConfiguration(
    authorization_scope=AuthorizationScope.LIVE, takeoff_time=1_760_000_012.75
)


def make_uav(seconds, msec, *, echo=None, **kwds):
    return SimpleNamespace(
        id=kwds.get("id", "1"),
        scheduled_takeoff_authorization_scope=kwds.get(
            "scope", AuthorizationScope.LIVE
        ),
        scheduled_takeoff_time=seconds,
        scheduled_takeoff_time_msec=msec,
        show_schedule_status=echo,
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


SENT_START_US = 10_000_000_000


def make_echo(schedule_id, start_offset_us=0, *, flags=0x01 | 0x04, age=0.5):
    data = bytes([0x42]) + pack(
        "<HBQ", schedule_id, flags, SENT_START_US + start_offset_us if flags & 1 else 0
    )
    message = SimpleNamespace(len=len(data), data=data)
    return ShowScheduleStatus.from_mavlink_message(message, time() - age)


def check_start_agreement(uavs, lead, schedule_id=5):
    schedule = ShowSchedule()
    schedule.id = schedule_id
    schedule.start_cluster_us = SENT_START_US
    for uav in uavs:
        uav.send_log_message_to_gcs = lambda message, severity, uav=uav: (
            uav.messages.append((message, severity))
        )

    manager = MAVLinkScheduledTakeoffManager.__new__(MAVLinkScheduledTakeoffManager)
    manager._log = logging.getLogger("test_ext_mavlink_takeoff")
    manager._network = SimpleNamespace(driver=SimpleNamespace(show_schedule=schedule))
    manager.iter_uavs_to_schedule = lambda: iter(uavs)  # type: ignore[method-assign]
    manager.check_start_agreement(CONFIG, lead)


def test_start_agreement_before_the_drones_commit():
    uavs = {
        "in sync": make_uav(None, 0, echo=make_echo(5, 800)),
        "stale": make_uav(None, 0, echo=make_echo(5, age=4)),
        "no clock": make_uav(None, 0, echo=make_echo(5, flags=0x01)),
        "other schedule": make_uav(None, 0, echo=make_echo(4)),
        "no start": make_uav(None, 0, echo=make_echo(5, flags=0x04)),
        "start off": make_uav(None, 0, echo=make_echo(5, 1200)),
        "authorization": make_uav(
            None, 0, echo=make_echo(5), scope=AuthorizationScope.REHEARSAL
        ),
        # Without an echo: the start and the authorization only
        "legacy": make_uav(1_760_000_012, 750),
        "legacy off": make_uav(1_760_000_012, 500),
    }
    check_start_agreement(list(uavs.values()), 15)

    problems = {
        name: [message for message, _ in uav.messages] for name, uav in uavs.items()
    }
    assert problems["in sync"] == []
    assert problems["legacy"] == []
    for name, expected in {
        "stale": "echo is 4 s old",
        "no clock": "no valid show clock",
        "other schedule": "holds schedule 4, not 5",
        "no start": "holds no start",
        "start off": "+1.2 ms off",
        "authorization": "authorization differs",
        "legacy off": "start time or authorization differs",
    }.items():
        [message] = problems[name]
        assert message.startswith("Not in sync for the scheduled start: ")
        assert expected in message
    assert uavs["stale"].messages[0][1] is Severity.WARNING


def test_start_agreement_after_the_drones_commit(caplog):
    caplog.set_level(logging.INFO)
    committed = 0x01 | 0x02 | 0x04
    uavs = {
        "drifting": make_uav(None, 0, echo=make_echo(5, -3000, flags=committed)),
        "not committed": make_uav(None, 0, echo=make_echo(5)),
        "other schedule": make_uav(None, 0, echo=make_echo(4, flags=committed)),
        "start off": make_uav(None, 0, echo=make_echo(5, 6000, flags=committed)),
    }
    check_start_agreement(list(uavs.values()), 5)

    assert uavs["drifting"].messages == []
    assert "not committed" in uavs["not committed"].messages[0][0]
    assert "holds schedule 4, not 5" in uavs["other schedule"].messages[0][0]
    assert "+6.0 ms off" in uavs["start off"].messages[0][0]

    offsets = [
        record.getMessage()
        for record in caplog.records
        if record.levelno == logging.INFO
    ]
    assert "Start offset at T-5 s: -3.00 ms" in offsets
    assert len(offsets) == 4


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
