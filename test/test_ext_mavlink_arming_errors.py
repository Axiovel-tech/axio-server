"""What an ArduPilot heartbeat and SYS_STATUS make of a UAV's status.

``armed`` is the autopilot's own arming state, from the heartbeat's
``MAV_MODE_FLAG_SAFETY_ARMED``. Control's armed-drone guard (RTLS cell Apply,
anchor sleep and unlock wording) reads only that.

The error codes keep their operator meaning: ``DISARMED`` is the hardware
safety switch (``MOTOR_OUTPUTS`` not enabled), ``ON_GROUND`` and
``MOTORS_RUNNING_WHILE_ON_GROUND`` are the standby indications.
"""

import pytest
from flockwave.protocols.mavlink.introspection import import_dialect
from flockwave.spec.errors import FlockwaveErrorCode

from flockwave.server.ext.mavlink.autopilots import ArduPilot
from flockwave.server.ext.mavlink.driver import (
    ARMED_HEARTBEAT_MAX_AGE,
    MAVLinkDriver,
    MAVLinkUAV,
)
from flockwave.server.ext.mavlink.enums import MAVMessageType
from flockwave.server.model.uav import UAVStatusInfo

dialect = import_dialect("ardupilotmega")
LINK = dialect.MAVLink(None, srcSystem=17, srcComponent=1)
MOTOR_OUTPUTS = dialect.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS
ARMED = dialect.MAV_MODE_FLAG_SAFETY_ARMED
CUSTOM_MODE = dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
STABILIZE = 0
STANDBY = dialect.MAV_STATE_STANDBY


def _uav() -> MAVLinkUAV:
    driver = MAVLinkDriver()
    driver.autopilot_factory = ArduPilot
    driver.assume_data_streams_configured = True
    driver.run_in_background = lambda *args, **kwds: None  # type: ignore[assignment]
    uav = MAVLinkUAV("17", driver=driver)
    uav.notify_updated = lambda: None  # type: ignore[assignment]
    return uav


def _sys_status(*, outputs_enabled: bool):
    message = dialect.MAVLink_sys_status_message(
        onboard_control_sensors_present=MOTOR_OUTPUTS,
        onboard_control_sensors_enabled=MOTOR_OUTPUTS if outputs_enabled else 0,
        onboard_control_sensors_health=MOTOR_OUTPUTS,
        load=0,
        voltage_battery=7800,
        current_battery=-1,
        battery_remaining=-1,
        drop_rate_comm=0,
        errors_comm=0,
        errors_count1=0,
        errors_count2=0,
        errors_count3=0,
        errors_count4=0,
    )
    message.pack(LINK)
    return message


def _heartbeat(*, armed: bool, system_status: int = STANDBY):
    message = dialect.MAVLink_heartbeat_message(
        type=dialect.MAV_TYPE_QUADROTOR,
        autopilot=dialect.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=CUSTOM_MODE | (ARMED if armed else 0),
        custom_mode=STABILIZE,
        system_status=system_status,
        mavlink_version=3,
    )
    message.pack(LINK)
    return message


def _receive(
    uav: MAVLinkUAV,
    *,
    armed: bool,
    outputs_enabled: bool,
    system_status: int = STANDBY,
) -> set[int]:
    uav.handle_message_sys_status(_sys_status(outputs_enabled=outputs_enabled))
    uav.handle_message_heartbeat(_heartbeat(armed=armed, system_status=system_status))
    return set(uav.status.errors)


def _age_heartbeat(uav: MAVLinkUAV, seconds: float) -> None:
    uav._last_messages[MAVMessageType.HEARTBEAT].timestamp -= seconds


# --- armed: the heartbeat's arming flag ---------------------------------------


def test_armed_follows_the_heartbeat_when_the_safety_switch_disagrees():
    # D17 on 2026-11-24: disarmed heartbeat, motor outputs enabled (no safety
    # switch), so no DISARMED error: armed is still False
    uav = _uav()
    errors = _receive(uav, armed=False, outputs_enabled=True)
    assert FlockwaveErrorCode.DISARMED.value not in errors
    assert uav.status.armed is False
    assert uav.status.json["armed"] is False

    # safety switch engaged (DISARMED) on an armed heartbeat: armed is True
    errors = _receive(uav, armed=True, outputs_enabled=False)
    assert FlockwaveErrorCode.DISARMED.value in errors
    assert uav.status.armed is True


def test_armed_tracks_disarmed_armed_disarmed():
    uav = _uav()
    seen = []
    for armed in (False, True, False):
        _receive(uav, armed=armed, outputs_enabled=True)
        seen.append(uav.status.json.get("armed"))
    assert seen == [False, True, False]


def test_a_duplicate_heartbeat_keeps_armed():
    uav = _uav()
    _receive(uav, armed=True, outputs_enabled=True)
    uav.handle_message_heartbeat(_heartbeat(armed=True))
    assert uav.status.armed is True


def test_armed_is_unknown_before_a_heartbeat():
    uav = _uav()
    uav.handle_message_sys_status(_sys_status(outputs_enabled=True))
    assert "armed" not in uav.status.json


def test_a_stale_heartbeat_with_fresh_sys_status_is_unknown():
    uav = _uav()
    _receive(uav, armed=False, outputs_enabled=True)
    assert uav.status.armed is False

    _age_heartbeat(uav, ARMED_HEARTBEAT_MAX_AGE + 0.5)
    before = uav.status.timestamp
    uav.handle_message_sys_status(_sys_status(outputs_enabled=True))
    assert uav.status.timestamp >= before
    assert "armed" not in uav.status.json

    # the next heartbeat restores it
    uav.handle_message_heartbeat(_heartbeat(armed=False))
    assert uav.status.armed is False


@pytest.mark.parametrize("touch", ["touch_status", "update_rssi"])
def test_every_status_refresh_rechecks_the_heartbeat(touch: str):
    uav = _uav()
    _receive(uav, armed=False, outputs_enabled=True)
    _age_heartbeat(uav, ARMED_HEARTBEAT_MAX_AGE + 0.5)
    if touch == "update_rssi":
        uav.update_rssi(index=0, value=80)
    else:
        uav.touch_status()
    assert "armed" not in uav.status.json


@pytest.mark.parametrize(
    "system_status", [dialect.MAV_STATE_BOOT, dialect.MAV_STATE_FLIGHT_TERMINATION]
)
def test_a_heartbeat_from_a_sleeping_or_booting_fc_is_unknown(system_status: int):
    uav = _uav()
    _receive(uav, armed=False, outputs_enabled=True)
    uav.handle_message_heartbeat(_heartbeat(armed=False, system_status=system_status))
    assert "armed" not in uav.status.json


def test_a_status_without_armed_reads_as_unknown():
    # what an older server sends: no armed key
    status = UAVStatusInfo(id="17")
    assert "armed" not in status.json
    assert status.armed is None
    status.armed = True
    assert status.json["armed"] is True
    status.armed = None
    assert "armed" not in status.json


# --- the operator error codes keep their meaning ------------------------------


def test_a_disarmed_heartbeat_with_motor_outputs_enabled_is_on_ground():
    # D17 on 2026-11-24: armed=false in Stabilize, MOTOR_OUTPUTS present,
    # healthy and enabled (no safety switch engaged)
    errors = _receive(_uav(), armed=False, outputs_enabled=True)
    assert FlockwaveErrorCode.ON_GROUND.value in errors
    assert FlockwaveErrorCode.DISARMED.value not in errors


def test_an_armed_heartbeat_is_never_on_ground():
    uav = _uav()
    on_ground = _receive(uav, armed=True, outputs_enabled=True)
    assert FlockwaveErrorCode.ON_GROUND.value not in on_ground
    assert FlockwaveErrorCode.MOTORS_RUNNING_WHILE_ON_GROUND.value in on_ground
    assert FlockwaveErrorCode.DISARMED.value not in on_ground

    flying = _receive(
        uav, armed=True, outputs_enabled=True, system_status=dialect.MAV_STATE_ACTIVE
    )
    assert FlockwaveErrorCode.ON_GROUND.value not in flying
    assert FlockwaveErrorCode.DISARMED.value not in flying


@pytest.mark.parametrize("armed", [False, True])
def test_disarmed_stays_the_safety_switch(armed: bool):
    errors = _receive(_uav(), armed=armed, outputs_enabled=False)
    assert FlockwaveErrorCode.DISARMED.value in errors


def test_a_disarmed_failsafe_heartbeat_is_not_on_ground():
    errors = _receive(
        _uav(),
        armed=False,
        outputs_enabled=True,
        system_status=dialect.MAV_STATE_CRITICAL,
    )
    assert FlockwaveErrorCode.ON_GROUND.value not in errors
    assert FlockwaveErrorCode.DISARMED.value not in errors
