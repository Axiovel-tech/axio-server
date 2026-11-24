"""A virtual UAV publishes its own arming state as the status's ``armed``."""

from flockwave.spec.errors import FlockwaveErrorCode

from flockwave.server.ext.virtual_uavs.driver import VirtualUAV, VirtualUAVDriver


def test_a_virtual_uav_publishes_armed_beside_disarmed():
    uav = VirtualUAV(
        "1",
        driver=VirtualUAVDriver(),
        use_battery_percentage=False,
        battery_auto_recharging=False,
    )
    # not booted yet: unknown
    assert "armed" not in uav.status.json

    uav.armed = False
    assert uav.status.json["armed"] is False
    assert FlockwaveErrorCode.DISARMED.value in uav.status.errors

    uav.armed = True
    assert uav.status.json["armed"] is True
    assert FlockwaveErrorCode.DISARMED.value not in uav.status.errors
