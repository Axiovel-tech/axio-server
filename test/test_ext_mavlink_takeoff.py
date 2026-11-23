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
        is_connected=kwds.get("connected", True),
        supports_scheduled_takeoff=True,
    )


SENT_START_US = 10_000_000_000


def make_echo(schedule_id, start_offset_us=0, *, flags=0x01 | 0x04, age=0.5):
    data = bytes([0x42]) + pack(
        "<HBQ", schedule_id, flags, SENT_START_US + start_offset_us if flags & 1 else 0
    )
    message = SimpleNamespace(len=len(data), data=data)
    return ShowScheduleStatus.from_mavlink_message(message, time() - age)


def check_start_agreement(
    uavs, lead, schedule_id=5, *, show=None, network_uavs=None, elsewhere=()
):
    """Runs the start agreement check on the given connected drones; returns
    the reasons the start was cancelled for. `show` lists the IDs of the
    drones of the show, `network_uavs` every drone of the network,
    `elsewhere` the IDs the server knows from other networks."""
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
    manager._network.uavs = lambda: list(network_uavs or uavs)
    manager._config = SimpleNamespace(uav_ids=show or [])  # type: ignore[assignment]
    refused = []
    manager.refuse_start = refused.append
    manager.find_uav = lambda id: object() if id in elsewhere else None
    manager.check_start_agreement(CONFIG, lead)
    return refused


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


def test_the_start_is_cancelled_unless_every_drone_of_the_show_is_ready():
    ready = make_uav(None, 0, echo=make_echo(5), id="1")
    stale = make_uav(None, 0, echo=make_echo(5, age=4), id="2")
    offline = make_uav(None, 0, id="3", connected=False)
    spare = make_uav(None, 0, echo=make_echo(4), id="9")
    connected = [ready, stale, spare]

    [reason] = check_start_agreement(
        connected,
        15,
        show=["1", "2", "3", "4", "5", None],
        network_uavs=[ready, stale, offline, spare],
        elsewhere={"5"},
    )
    assert reason.startswith("not every drone of the show is ready at T-15 s: ")
    assert "2: its show schedule echo is 4 s old" in reason
    assert "3: it is not connected" in reason
    assert "4: the server does not know it" in reason
    # Ready, checked by its own network, or not in the show
    for id in ("1:", "5:", "9:"):
        assert id not in reason


def test_a_ready_show_starts_even_with_a_spare_drone_out_of_sync():
    ready = make_uav(None, 0, echo=make_echo(5), id="1")
    spare = make_uav(None, 0, echo=make_echo(4), id="9")
    assert check_start_agreement([ready, spare], 15, show=["1"]) == []
    assert spare.messages  # still warned about


def test_without_an_assignment_every_drone_has_to_be_ready():
    ready = make_uav(None, 0, echo=make_echo(5), id="1")
    behind = make_uav(None, 0, echo=make_echo(4), id="2")
    [reason] = check_start_agreement([ready, behind], 15)
    assert "2: it holds schedule 4, not 5" in reason


def test_after_the_drones_commit_the_check_only_warns():
    behind = make_uav(None, 0, echo=make_echo(4, flags=0x01 | 0x02 | 0x04), id="1")
    assert check_start_agreement([behind], 5, show=["1"]) == []
    assert behind.messages


def test_many_drones_that_are_not_ready_are_summed_up():
    drones = [make_uav(None, 0, id=str(n), connected=False) for n in range(1, 9)]
    [reason] = check_start_agreement(
        [], 15, show=[d.id for d in drones], network_uavs=drones
    )
    assert reason.endswith("; and 3 more")


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
