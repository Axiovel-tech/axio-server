from struct import pack
from types import SimpleNamespace

from flockwave.protocols.mavlink.introspection import import_dialect

from flockwave.server.ext.mavlink.channel import encode_mavlink_message_from_spec
from flockwave.server.ext.mavlink.network import MAVLinkNetwork
from flockwave.server.ext.mavlink.packets import create_show_schedule_packet


class Drone:
    system_id = 7
    mavlink_version = 2


def make_network():
    sent = []

    async def send_packet(spec, destination):
        sent.append((spec, destination))

    target = Drone()
    network = SimpleNamespace(
        _uav_addresses={target: ("192.0.2.7", 14550)},
        manager=SimpleNamespace(send_packet=send_packet),
    )
    return network, target, sent


def encode_and_decode(spec):
    link = import_dialect("ardupilotmega").MAVLink(
        None, srcSystem=255, srcComponent=190
    )
    frame = encode_mavlink_message_from_spec(spec, link)
    return link.decode(bytearray(frame))


async def test_a_custom_data_packet_is_unicast_without_target_fields():
    network, target, sent = make_network()
    spec = create_show_schedule_packet(3, 123_456_789)

    await MAVLinkNetwork.send_packet(network, spec, target)  # type: ignore[arg-type]

    [(sent_spec, _)] = sent
    message = encode_and_decode(sent_spec)
    assert message.get_type() == "DATA16"
    assert message.type == 0x5C
    assert bytes(message.data[: message.len]) == bytes([0x42]) + pack(
        "<HQB", 3, 123_456_789, 0
    )


async def test_an_addressed_message_still_gets_its_target():
    network, target, sent = make_network()
    spec = ("PARAM_SET", {"param_id": b"SHOW_START_AUTH", "param_value": 1.0, "param_type": 9})

    await MAVLinkNetwork.send_packet(network, spec, target)  # type: ignore[arg-type]

    [(sent_spec, _)] = sent
    message = encode_and_decode(sent_spec)
    assert (message.target_system, message.target_component) == (7, 1)
