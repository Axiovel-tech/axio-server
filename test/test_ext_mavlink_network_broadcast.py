from types import SimpleNamespace

from pytest import raises

from flockwave.server.ext.mavlink.channel import Channel
from flockwave.server.ext.mavlink.enums import MAVCommand
from flockwave.server.ext.mavlink.network import MAVLinkNetwork
from flockwave.server.ext.mavlink.packets import create_show_schedule_packet
from flockwave.server.ext.mavlink.types import spec as message


class Drone:
    mavlink_version = 2

    def __init__(self, system_id: int):
        self.system_id = system_id


def make_network(system_ids=(7, 8), *, broadcast_error=None, unreachable=()):
    """A network with one drone per system ID, each at its own address, that
    records what leaves it instead of sending it."""
    sent = []

    async def broadcast_packet(spec, destination=None):
        if broadcast_error is not None:
            raise broadcast_error
        sent.append(("broadcast", spec, destination))

    async def send_packet(spec, destination):
        channel, (host, _) = destination
        if host in unreachable:
            raise RuntimeError("no route to host")
        sent.append((host, spec, channel))

    network = SimpleNamespace(
        _uav_addresses={
            Drone(system_id): (f"192.0.2.{system_id}", 14550)
            for system_id in system_ids
        },
        manager=SimpleNamespace(
            broadcast_packet=broadcast_packet, send_packet=send_packet
        ),
    )
    network.send_packet = lambda spec, target, **kwds: MAVLinkNetwork.send_packet(
        network,  # type: ignore[arg-type]
        spec,
        target,
        **kwds,
    )
    return network, sent


def land_all():
    return message.command_long(
        command=MAVCommand.NAV_LAND, target_system=0, target_component=0
    )


async def test_a_broadcast_also_goes_to_every_drone_on_its_own_address():
    network, sent = make_network()
    spec = land_all()

    await MAVLinkNetwork.broadcast_packet(network, spec, Channel.SHOW_CONTROL)  # type: ignore[arg-type]

    assert [(to, channel) for to, _, channel in sent] == [
        ("broadcast", Channel.SHOW_CONTROL),
        ("192.0.2.7", Channel.SHOW_CONTROL),
        ("192.0.2.8", Channel.SHOW_CONTROL),
    ]
    # The broadcast addresses every system, each other copy its own drone
    targets = [
        (fields["target_system"], fields["target_component"])
        for _, (_, fields), _ in sent
    ]
    assert targets == [(0, 0), (7, 1), (8, 1)]
    assert spec[1]["target_system"] == 0


async def test_a_custom_data_packet_goes_to_every_drone_without_a_target():
    network, sent = make_network()
    spec = create_show_schedule_packet(3, 123_456_789)

    await MAVLinkNetwork.broadcast_packet(network, spec)  # type: ignore[arg-type]

    assert [to for to, _, _ in sent] == ["broadcast", "192.0.2.7", "192.0.2.8"]
    assert all(kind == "DATA16" for _, (kind, _), _ in sent)
    assert all("target_system" not in fields for _, (_, fields), _ in sent)
    # Without a channel the drones are reached on the primary one
    assert [channel for _, _, channel in sent] == [
        None,
        Channel.PRIMARY,
        Channel.PRIMARY,
    ]


async def test_a_drone_that_cannot_be_reached_does_not_stop_the_others():
    network, sent = make_network((7, 8, 9), unreachable={"192.0.2.8"})

    await MAVLinkNetwork.broadcast_packet(network, land_all())  # type: ignore[arg-type]

    assert [to for to, _, _ in sent] == ["broadcast", "192.0.2.7", "192.0.2.9"]


async def test_a_failed_broadcast_is_not_an_error_while_the_drones_are_reached():
    network, sent = make_network(broadcast_error=RuntimeError("no broadcast address"))

    await MAVLinkNetwork.broadcast_packet(network, land_all())  # type: ignore[arg-type]

    assert [to for to, _, _ in sent] == ["192.0.2.7", "192.0.2.8"]


async def test_a_failed_broadcast_is_an_error_when_no_drone_is_reached_either():
    error = RuntimeError("no broadcast address")
    for network, _ in (
        make_network((), broadcast_error=error),
        make_network((7,), broadcast_error=error, unreachable={"192.0.2.7"}),
    ):
        with raises(RuntimeError, match="no broadcast address"):
            await MAVLinkNetwork.broadcast_packet(network, land_all())  # type: ignore[arg-type]


async def test_a_broadcast_without_drones_is_only_broadcast():
    network, sent = make_network(())

    await MAVLinkNetwork.broadcast_packet(network, land_all())  # type: ignore[arg-type]

    assert [to for to, _, _ in sent] == ["broadcast"]
