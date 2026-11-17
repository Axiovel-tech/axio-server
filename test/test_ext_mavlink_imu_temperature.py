"""IMU temperatures of a MAVLink UAV: one per IMU from its SCALED_IMU
messages, unknown while an IMU is silent, stale or reports no temperature."""

from collections import defaultdict
from types import SimpleNamespace

import pytest

from flockwave.server.ext.mavlink.driver import (
    IMU_TEMPERATURE_MAX_AGE,
    MAVLinkMessageRecord,
    MAVLinkUAV,
)
from flockwave.server.ext.mavlink.enums import MAVMessageType


class FakeIMUMessage(SimpleNamespace):
    def get_msgId(self) -> int:
        return int(self.msg_id)


def make_uav():
    uav = SimpleNamespace(
        _last_messages=defaultdict(MAVLinkMessageRecord),
        _compass_motor_interference_calibration=None,
    )
    for method in (
        "get_age_of_message",
        "get_imu_temperatures",
        "get_last_message",
        "handle_message_scaled_imu",
        "_store_message",
    ):
        setattr(uav, method, getattr(MAVLinkUAV, method).__get__(uav))
    return uav


def receive(uav, type: MAVMessageType, centidegrees: int, age: float = 0.0):
    uav.handle_message_scaled_imu(FakeIMUMessage(msg_id=type, temperature=centidegrees))
    uav._last_messages[int(type)].timestamp -= age


def test_temperatures_follow_the_imu_order():
    uav = make_uav()
    receive(uav, MAVMessageType.SCALED_IMU2, 4955)
    receive(uav, MAVMessageType.SCALED_IMU, 5212)

    assert uav.get_imu_temperatures() == [pytest.approx(52.12), pytest.approx(49.55)]


def test_no_reading_is_an_empty_list():
    assert make_uav().get_imu_temperatures() == []


def test_zero_means_no_temperature_and_trailing_unknowns_are_dropped():
    uav = make_uav()
    receive(uav, MAVMessageType.SCALED_IMU, 0)
    receive(uav, MAVMessageType.SCALED_IMU2, 3810)
    receive(uav, MAVMessageType.SCALED_IMU3, 0)

    assert uav.get_imu_temperatures() == [None, pytest.approx(38.1)]


def test_a_stale_imu_is_unknown():
    uav = make_uav()
    receive(uav, MAVMessageType.SCALED_IMU, 4000, age=IMU_TEMPERATURE_MAX_AGE + 1)
    receive(uav, MAVMessageType.SCALED_IMU2, 4100)

    assert uav.get_imu_temperatures() == [None, pytest.approx(41.0)]


def test_a_silent_imu_keeps_its_slot():
    uav = make_uav()
    receive(uav, MAVMessageType.SCALED_IMU, 4000)
    receive(uav, MAVMessageType.SCALED_IMU2, 4100, age=IMU_TEMPERATURE_MAX_AGE + 1)

    assert uav.get_imu_temperatures() == [pytest.approx(40.0), None]
