from types import SimpleNamespace

from flockwave.server.ext.mavlink.extension import MAVLinkDronesExtension


def network(*uavs):
    return SimpleNamespace(
        uavs=lambda: [SimpleNamespace(system_id=sid, id=uid) for sid, uid in uavs]
    )


def test_uav_ids_by_system_id_skip_a_system_id_shared_by_networks():
    extension = MAVLinkDronesExtension()
    extension._networks = {
        "a": network((1, "a/1"), (2, "a/2")),
        "b": network((2, "b/2"), (3, "b/3")),
    }

    assert extension._get_uav_ids_by_system_id() == {1: "a/1", 3: "b/3"}


def test_uav_ids_by_system_id_without_networks():
    extension = MAVLinkDronesExtension()
    extension._networks = {}

    assert extension._get_uav_ids_by_system_id() == {}
