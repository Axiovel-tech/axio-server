"""Broadcast commands: one broadcast, the acknowledgment of each drone, and the
command on its own address for each drone that did not acknowledge it."""

from collections import defaultdict
from functools import partial
from types import SimpleNamespace

import pytest
import trio
import trio.testing

from flockwave.server.errors import NotSupportedError
from flockwave.server.ext.mavlink.driver import MAVLinkDriver
from flockwave.server.ext.mavlink.enums import MAVCommand, MAVResult
from flockwave.server.ext.mavlink.network import MAVLinkNetwork
from flockwave.server.model.transport import TransportOptions


class Drone:
    def __init__(self, system_id: int, *, connected: bool = True):
        self.system_id = system_id
        self.id = str(system_id)
        self.network_id = "mav"
        self.is_connected = connected


def ack(command: int, result: int = MAVResult.ACCEPTED) -> SimpleNamespace:
    return SimpleNamespace(command=command, result=result)


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


# Network


def make_network(replying: dict[int, SimpleNamespace]):
    """A network whose drones with the given system IDs reply to a broadcast
    at once, while it is being sent, as the receive loop would deliver them."""
    sent = []
    network = SimpleNamespace(_matchers=defaultdict(list))
    network.expect_packet = partial(MAVLinkNetwork.expect_packet, network)

    async def broadcast_packet(spec, channel=None):
        sent.append((spec, channel))
        for system_id, params, future in list(network._matchers["COMMAND_ACK"]):
            reply = replying.get(system_id)
            if reply is not None and reply.command == params["command"]:
                future.set_result(reply)

    network.broadcast_packet = broadcast_packet
    return network, sent


async def test_replies_are_collected_even_when_they_beat_the_wait(autojump_clock):
    fast, silent = Drone(1), Drone(2)
    reply = ack(MAVCommand.NAV_LAND)
    network, sent = make_network({1: reply})

    started = trio.current_time()
    replies = await MAVLinkNetwork.broadcast_packet_and_collect_replies(
        network,  # type: ignore[arg-type]
        ("COMMAND_LONG", {}),
        [fast, silent],
        ("COMMAND_ACK", {"command": MAVCommand.NAV_LAND}),
        timeout=0.5,
        channel="primary",
    )

    assert replies == {fast: reply}
    assert sent == [(("COMMAND_LONG", {}), "primary")]
    assert trio.current_time() - started == pytest.approx(0.5)
    assert network._matchers["COMMAND_ACK"] == []


async def test_the_wait_ends_when_every_drone_replied(autojump_clock):
    drones = [Drone(1), Drone(2)]
    network, _ = make_network({1: ack(1), 2: ack(1)})

    started = trio.current_time()
    replies = await MAVLinkNetwork.broadcast_packet_and_collect_replies(
        network,  # type: ignore[arg-type]
        ("COMMAND_LONG", {}),
        drones,
        ("COMMAND_ACK", {"command": 1}),
        timeout=0.5,
    )

    assert set(replies) == set(drones)
    assert trio.current_time() == started


# Driver


def make_driver(acks: dict[Drone, int] | list[dict[Drone, int]]):
    """A driver whose broadcasts are acknowledged with the given results, one
    dictionary per broadcast, and whose commands to a single drone are
    recorded."""
    driver = MAVLinkDriver()
    driver.log = SimpleNamespace(warning=lambda message: warnings.append(message))  # type: ignore[assignment]
    rounds = list(acks) if isinstance(acks, list) else [acks]
    broadcasts, singles, warnings = [], [], []

    async def collect(spec, uavs, reply, *, timeout, channel=None):
        broadcasts.append((spec, list(uavs), reply, timeout, channel))
        return {
            uav: ack(reply[1]["command"], result)
            for uav, result in rounds[len(broadcasts) - 1].items()
        }

    driver.broadcast_packet_and_collect_replies = collect

    def record(name):
        async def single(uav, **kwds):
            singles.append((name, uav, kwds))

        return single

    driver._send_landing_signal_single = record("land")  # type: ignore[method-assign]
    driver._send_shutdown_signal_single = record("shutdown")  # type: ignore[method-assign]
    return driver, broadcasts, singles, warnings


async def test_broadcast_land_reports_each_drone_and_unicasts_the_silent_one():
    accepted, refused, unsupported, silent = (Drone(n) for n in (1, 2, 3, 4))
    driver, broadcasts, singles, warnings = make_driver(
        {
            accepted: MAVResult.ACCEPTED,
            refused: MAVResult.DENIED,
            unsupported: MAVResult.UNSUPPORTED,
        }
    )
    uavs = [accepted, refused, unsupported, silent]
    transport = TransportOptions.from_json({"broadcast": True, "ignoreIds": True})

    results = await driver.send_landing_signal(uavs, transport=transport)  # type: ignore[arg-type]

    # One broadcast, addressed to all, the acknowledgment of every drone awaited
    [(spec, collected, reply, timeout, _)] = broadcasts
    assert spec[0] == "COMMAND_LONG"
    assert spec[1]["command"] == MAVCommand.NAV_LAND
    assert (spec[1]["target_system"], spec[1]["target_component"]) == (0, 0)
    assert collected == uavs
    assert reply == ("COMMAND_ACK", {"command": MAVCommand.NAV_LAND})
    assert timeout == driver.broadcast_ack_timeout

    assert results[accepted] is None
    assert isinstance(results[refused], RuntimeError)
    assert "DENIED" in str(results[refused])
    assert isinstance(results[unsupported], NotSupportedError)

    # The silent drone gets the command on its own address, as in unicast mode
    assert singles == []
    await results[silent]
    assert singles == [("land", silent, {"transport": transport})]
    [warning] = warnings
    assert "1 of 4 drones" in warning
    assert warning.endswith(": 4")


async def test_a_silent_drone_that_is_not_connected_is_not_sent_the_command_again():
    # Its last address may now be a drone that another ground computer flies
    heard, gone = Drone(1), Drone(2, connected=False)
    driver, _, singles, warnings = make_driver({heard: MAVResult.ACCEPTED})

    results = await driver.send_landing_signal(
        [heard, gone],  # type: ignore[list-item]
        transport=TransportOptions.from_json({"broadcast": True}),
    )

    assert results[heard] is None
    assert isinstance(results[gone], RuntimeError)
    assert "not connected" in str(results[gone])
    assert singles == []
    [warning] = warnings
    assert "1 of 2 drones" in warning and "not connected" in warning
    assert warning.endswith(": 2")


async def test_shutdown_goes_to_a_drone_on_its_own_address_if_it_missed_a_step():
    both, missed_disarm, missed_shutdown = Drone(1), Drone(2), Drone(3)
    driver, broadcasts, singles, _ = make_driver(
        [
            {both: MAVResult.ACCEPTED, missed_shutdown: MAVResult.ACCEPTED},
            {both: MAVResult.ACCEPTED, missed_disarm: MAVResult.ACCEPTED},
        ]
    )
    transport = TransportOptions.from_json({"broadcast": True})

    results = await driver.send_shutdown_signal(
        [both, missed_disarm, missed_shutdown],  # type: ignore[list-item]
        transport=transport,
    )

    commands = [spec[1]["command"] for spec, *_ in broadcasts]
    assert commands == [
        MAVCommand.COMPONENT_ARM_DISARM,
        MAVCommand.PREFLIGHT_REBOOT_SHUTDOWN,
    ]
    assert results[both] is None
    for uav in (missed_disarm, missed_shutdown):
        await results[uav]
    assert [(name, uav) for name, uav, _ in singles] == [
        ("shutdown", missed_disarm),
        ("shutdown", missed_shutdown),
    ]


async def test_a_refused_disarm_is_the_result_of_the_shutdown():
    drone = Drone(1)
    driver, _, singles, _ = make_driver(
        [{drone: MAVResult.TEMPORARILY_REJECTED}, {drone: MAVResult.ACCEPTED}]
    )

    results = await driver.send_shutdown_signal(
        [drone],  # type: ignore[list-item]
        transport=TransportOptions.from_json({"broadcast": True}),
    )

    assert "TEMPORARILY_REJECTED" in str(results[drone])
    assert singles == []
