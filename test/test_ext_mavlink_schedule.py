from struct import pack, unpack
from types import SimpleNamespace

import trio
from pytest import fixture

from flockwave.server.ext.mavlink import takeoff
from flockwave.server.ext.mavlink.channel import Channel
from flockwave.server.ext.mavlink.driver import MAVLinkDriver, MAVLinkUAV
from flockwave.server.ext.mavlink.packets import (
    ShowScheduleStatus,
    create_show_schedule_packet,
)
from flockwave.server.ext.mavlink.schedule import ShowSchedule
from flockwave.server.ext.mavlink.takeoff import MAVLinkScheduledTakeoffManager
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.takeoff import TakeoffConfiguration

EPOCH = 1_760_000_000.0
"""Wall time at trio time zero in the tests that run the schedule sender."""

WALL_MINUS_CLUSTER = 1_759_990_000.0


class FakeClusterClock:
    def __init__(self, offset: float | None = WALL_MINUS_CLUSTER):
        self.offset = offset

    def cluster_us_at(self, wall):
        return None if self.offset is None else round((wall - self.offset) * 1e6)

    def wall_at(self, cluster_us):
        return None if self.offset is None else cluster_us / 1e6 + self.offset


def auto(start: float | None) -> TakeoffConfiguration:
    return TakeoffConfiguration(
        authorization_scope=AuthorizationScope.LIVE, takeoff_time=start
    )


RC_AUTHORIZED = TakeoffConfiguration(
    authorization_scope=AuthorizationScope.LIVE, should_update_takeoff_time=False
)


def cluster_us(wall: float, offset: float = WALL_MINUS_CLUSTER) -> int:
    return round((wall - offset) * 1e6)


def echo_message(schedule_id, flags, start_cluster_us, *, marker=0x42):
    data = bytes([marker]) + pack("<HBQ", schedule_id, flags, start_cluster_us)
    return SimpleNamespace(type=0x5E, len=len(data), data=data.ljust(16, b"\0"))


def echo(schedule_id, start_cluster_us, *, committed=False, received_at=EPOCH):
    flags = 0x01 | 0x04 | (0x02 if committed else 0)
    status = ShowScheduleStatus.from_mavlink_message(
        echo_message(schedule_id, flags, start_cluster_us), received_at
    )
    assert status is not None
    return status


@fixture
def clock() -> FakeClusterClock:
    return FakeClusterClock()


@fixture
def schedule(clock) -> ShowSchedule:
    schedule = ShowSchedule()
    schedule.cluster_clock = lambda: clock
    return schedule


# Packets


def test_show_schedule_packet_layout():
    name, fields = create_show_schedule_packet(0x1234, 0x0102030405060708)
    assert name == "DATA16"
    assert fields["type"] == 0x5C
    assert fields["len"] == 12
    data = fields["data"]
    assert len(data) == 16
    assert data[0] == 0x42
    assert unpack("<HQB", data[1:12]) == (0x1234, 0x0102030405060708, 0)


def test_show_schedule_status_decoding():
    status = ShowScheduleStatus.from_mavlink_message(
        echo_message(7, 0x01 | 0x02 | 0x04 | 0x80, 123_456_789), EPOCH
    )
    assert status is not None
    assert (status.schedule_id, status.start_cluster_us) == (7, 123_456_789)
    assert status.has_start and status.committed and status.clock_valid
    assert status.received_at == EPOCH

    rc_set = ShowScheduleStatus.from_mavlink_message(echo_message(0, 0x08, 5), EPOCH)
    assert rc_set is not None
    assert not rc_set.has_start and not rc_set.committed and not rc_set.clock_valid

    assert (
        ShowScheduleStatus.from_mavlink_message(echo_message(7, 1, 1, marker=1), EPOCH)
        is None
    )


# Schedule state


def test_schedule_id_changes_when_the_operator_changes_the_start(schedule):
    start = EPOCH + 60
    first_id = schedule.id

    schedule.update(auto(start))
    assert schedule.id == first_id + 1
    assert schedule.start_cluster_us == cluster_us(start)

    schedule.update(auto(start + 0.0004))
    assert schedule.id == first_id + 1

    schedule.update(auto(start + 0.002))
    assert schedule.id == first_id + 2
    assert schedule.start_cluster_us == cluster_us(start + 0.002)

    schedule.update(auto(None))
    assert schedule.id == first_id + 3
    assert schedule.start_cluster_us == 0

    schedule.update(auto(None))
    assert schedule.id == first_id + 3


def test_schedule_id_wraps_to_one(schedule):
    schedule.id = 0xFFFF
    schedule.update(auto(EPOCH + 60))
    assert schedule.id == 1


def test_schedule_follows_the_cluster_clock_beyond_1_ms_until_locked(schedule, clock):
    locked = [False]
    schedule.is_locked = lambda: locked[0]
    start = EPOCH + 60
    schedule.update(auto(start))
    schedule_id = schedule.id

    clock.offset += 0.0008
    schedule.tick()
    assert schedule.start_cluster_us == cluster_us(start)

    clock.offset += 0.0004
    schedule.tick()
    assert schedule.start_cluster_us == cluster_us(start, clock.offset)
    assert schedule.id == schedule_id

    # A cluster restart: re-sent in the new timebase under the same ID
    clock.offset -= 3600
    schedule.tick()
    assert schedule.start_cluster_us == cluster_us(start, clock.offset)
    assert schedule.id == schedule_id

    locked[0] = True
    frozen = schedule.start_cluster_us
    clock.offset += 0.01
    schedule.tick()
    assert schedule.start_cluster_us == frozen

    # The abort: a new ID without a start
    schedule.update(auto(None))
    assert schedule.id == schedule_id + 1
    assert schedule.start_cluster_us == 0


def test_schedule_packet(schedule, clock):
    start = EPOCH + 60
    assert schedule.packet(EPOCH) is None

    schedule.update(auto(None))
    _, fields = schedule.packet(EPOCH)
    assert unpack("<HQB", fields["data"][1:12]) == (schedule.id, 0, 0)

    clock.offset = None
    schedule.update(auto(start))
    assert schedule.packet(EPOCH) is None

    clock.offset = WALL_MINUS_CLUSTER
    schedule.tick()
    _, fields = schedule.packet(EPOCH)
    assert unpack("<HQB", fields["data"][1:12]) == (
        schedule.id,
        cluster_us(start),
        0,
    )
    assert schedule.packet(start + 1) is None

    # The RC holder owns the start: nothing is sent, the schedule stays
    schedule_id = schedule.id
    schedule.update(RC_AUTHORIZED)
    assert schedule.packet(EPOCH) is None
    schedule.update(auto(start))
    assert schedule.id == schedule_id
    assert schedule.packet(EPOCH) is not None


def test_schedule_period(schedule):
    start = EPOCH + 60
    assert schedule.period(EPOCH) == 1.0

    schedule.update(auto(start))
    assert schedule.period(start - 15.5) == 1.0
    assert schedule.period(start - 15) == 0.2
    assert schedule.period(start - 0.1) == 0.2
    assert schedule.period(start) == 1.0

    # An abort keeps the pace of the aborted start
    schedule.update(auto(None))
    assert schedule.period(start - 5) == 0.2
    assert schedule.period(start + 1) == 1.0


def test_schedule_disagreement(schedule):
    start = EPOCH + 60
    schedule.update(auto(start))
    sent = schedule.start_cluster_us

    assert not schedule.disagrees_with(echo(schedule.id, sent))
    assert schedule.disagrees_with(echo(schedule.id - 1, sent))
    assert schedule.disagrees_with(echo(schedule.id, sent + 1))
    # A committed drone's start moves with its crystal
    assert not schedule.disagrees_with(echo(schedule.id, sent + 3000, committed=True))


# Takeoff manager


def make_manager(schedule):
    sent = []

    async def broadcast_packet(spec, channel=None):
        sent.append(("broadcast", trio.current_time(), spec, channel))

    async def send_packet(spec, target, channel=None):
        sent.append(("unicast", target.id, spec, channel))

    network = SimpleNamespace(
        log=None,
        driver=SimpleNamespace(show_schedule=schedule),
        broadcast_packet=broadcast_packet,
        send_packet=send_packet,
    )
    return MAVLinkScheduledTakeoffManager(network), sent  # type: ignore[arg-type]


def make_uav(status=None, start=None):
    writes = []

    async def set_scheduled_takeoff_time(seconds, msec=0):
        writes.append(seconds)

    return SimpleNamespace(
        id="1",
        scheduled_takeoff_authorization_scope=AuthorizationScope.LIVE,
        scheduled_takeoff_time=start,
        show_schedule_status=status,
        set_scheduled_takeoff_time=set_scheduled_takeoff_time,
        writes=writes,
    )


async def test_drones_echoing_the_schedule_get_it_unicast_never_as_parameters(
    schedule, monkeypatch
):
    monkeypatch.setattr(takeoff, "time", lambda: EPOCH)
    manager, sent = make_manager(schedule)
    start = EPOCH + 60
    config = auto(start)
    schedule.update(config)

    in_sync = make_uav(echo(schedule.id, schedule.start_cluster_us))
    behind = make_uav(echo(schedule.id - 1, 0))
    assert not manager.uav_needs_update(in_sync, config)
    assert manager.uav_needs_update(behind, config)

    await manager.update_uav(behind, config)
    assert behind.writes == []
    [(kind, target, spec, channel)] = sent
    assert (kind, target, channel) == ("unicast", "1", Channel.SHOW_CONTROL)
    assert spec == schedule.packet(EPOCH)

    # With the RC start the RC holder owns the start
    schedule.update(RC_AUTHORIZED)
    assert not manager.uav_needs_update(behind, RC_AUTHORIZED)

    # Without an echo the start still goes by parameters
    legacy = make_uav()
    assert manager.uav_needs_update(legacy, config)
    await manager.update_uav(legacy, config)
    assert legacy.writes == [int(start)]


async def test_schedule_broadcast_pace(schedule, monkeypatch, autojump_clock, nursery):
    monkeypatch.setattr(takeoff, "time", lambda: EPOCH + trio.current_time())
    manager, sent = make_manager(schedule)
    schedule.update(auto(EPOCH + 30))

    nursery.start_soon(manager._broadcast_show_schedule)
    await trio.sleep(10.5)
    times = [at for _, at, _, _ in sent]
    assert times == [float(second) for second in range(11)]

    # A changed start goes out at once
    schedule.update(auto(EPOCH + 40))
    await trio.sleep(0.01)
    assert sent[-1][1] == 10.5
    assert sent[-1][3] == Channel.SHOW_CONTROL

    # Every 0.2 s within 15 s of the start, nothing once it has passed
    sent.clear()
    await trio.sleep_until(45)
    times = [round(at, 1) for _, at, _, _ in sent]
    assert times[:15] == [11.5 + step for step in range(15)]
    assert times[15:] == [round(25.7 + 0.2 * step, 1) for step in range(72)]


@fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


# Driver


def test_reported_start_comes_from_the_echo(schedule):
    driver = MAVLinkDriver()
    driver.show_schedule = schedule
    uav = MAVLinkUAV("1", driver=driver)
    uav.notify_updated = lambda: None

    start = EPOCH + 60.25
    uav.handle_message_show_schedule_status(
        echo_message(3, 0x01 | 0x04, cluster_us(start))
    )
    assert uav.show_schedule_status is not None
    assert uav.scheduled_takeoff_time == int(start)
    assert uav.scheduled_takeoff_time_msec == 250

    # Mode 2 reports no start in the status packet
    status = (-1).to_bytes(4, "little", signed=True).ljust(14, b"\0")
    uav.handle_message_drone_show_status(
        SimpleNamespace(type=0x5B, len=len(status), data=status)
    )
    assert uav.scheduled_takeoff_time == int(start)

    uav.handle_message_show_schedule_status(echo_message(4, 0x04, 0))
    assert uav.scheduled_takeoff_time is None
    assert uav.scheduled_takeoff_time_msec == 0
