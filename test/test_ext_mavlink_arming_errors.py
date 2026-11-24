"""What an ArduPilot heartbeat and SYS_STATUS make of the arming error codes.

Control's armed-drone guard (RTLS cell Apply, anchor sleep and unlock
wording) reads them: ``DISARMED`` is the hardware safety switch
(``MOTOR_OUTPUTS`` not enabled), ``ON_GROUND`` and
``MOTORS_RUNNING_WHILE_ON_GROUND`` follow the heartbeat's armed flag.
"""

import pytest
from flockwave.protocols.mavlink.introspection import import_dialect
from flockwave.spec.errors import FlockwaveErrorCode

from flockwave.server.ext.mavlink.autopilots import ArduPilot
from flockwave.server.ext.mavlink.driver import MAVLinkDriver, MAVLinkUAV

dialect = import_dialect("ardupilotmega")
LINK = dialect.MAVLink(None, srcSystem=17, srcComponent=1)
MOTOR_OUTPUTS = dialect.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS
ARMED = dialect.MAV_MODE_FLAG_SAFETY_ARMED
CUSTOM_MODE = dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
STABILIZE = 0


def _uav() -> MAVLinkUAV:
    driver = MAVLinkDriver()
    driver.autopilot_factory = ArduPilot
    driver.assume_data_streams_configured = True
    driver.run_in_background = lambda *args, **kwds: None  # type: ignore[assignment]
    uav = MAVLinkUAV("17", driver=driver)
    uav.notify_updated = lambda: None  # type: ignore[assignment]
    return uav


def _receive(
    uav: MAVLinkUAV, *, armed: bool, outputs_enabled: bool, system_status: int
) -> set[int]:
    sys_status = dialect.MAVLink_sys_status_message(
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
    heartbeat = dialect.MAVLink_heartbeat_message(
        type=dialect.MAV_TYPE_QUADROTOR,
        autopilot=dialect.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=CUSTOM_MODE | (ARMED if armed else 0),
        custom_mode=STABILIZE,
        system_status=system_status,
        mavlink_version=3,
    )
    for message in (sys_status, heartbeat):
        message.pack(LINK)
    uav.handle_message_sys_status(sys_status)
    uav.handle_message_heartbeat(heartbeat)
    return set(uav.status.errors)


def test_a_disarmed_heartbeat_with_motor_outputs_enabled_is_on_ground():
    # D17 on 2026-11-24: armed=false in Stabilize, MOTOR_OUTPUTS present,
    # healthy and enabled (no safety switch engaged)
    errors = _receive(
        _uav(),
        armed=False,
        outputs_enabled=True,
        system_status=dialect.MAV_STATE_STANDBY,
    )
    assert FlockwaveErrorCode.ON_GROUND.value in errors
    assert FlockwaveErrorCode.DISARMED.value not in errors


def test_an_armed_heartbeat_is_never_on_ground():
    uav = _uav()
    on_ground = _receive(
        uav, armed=True, outputs_enabled=True, system_status=dialect.MAV_STATE_STANDBY
    )
    assert FlockwaveErrorCode.ON_GROUND.value not in on_ground
    assert FlockwaveErrorCode.MOTORS_RUNNING_WHILE_ON_GROUND.value in on_ground
    assert FlockwaveErrorCode.DISARMED.value not in on_ground

    flying = _receive(
        uav, armed=True, outputs_enabled=True, system_status=dialect.MAV_STATE_ACTIVE
    )
    assert FlockwaveErrorCode.ON_GROUND.value not in flying
    assert FlockwaveErrorCode.DISARMED.value not in flying


def test_arming_clears_on_ground_on_the_next_heartbeat():
    uav = _uav()
    standby = dialect.MAV_STATE_STANDBY
    assert FlockwaveErrorCode.ON_GROUND.value in _receive(
        uav, armed=False, outputs_enabled=True, system_status=standby
    )
    assert FlockwaveErrorCode.ON_GROUND.value not in _receive(
        uav, armed=True, outputs_enabled=True, system_status=standby
    )


@pytest.mark.parametrize("armed", [False, True])
def test_disarmed_stays_the_safety_switch(armed: bool):
    errors = _receive(
        _uav(),
        armed=armed,
        outputs_enabled=False,
        system_status=dialect.MAV_STATE_STANDBY,
    )
    assert FlockwaveErrorCode.DISARMED.value in errors


def test_a_disarmed_failsafe_heartbeat_is_not_on_ground():
    # a failsafe reports CRITICAL while disarmed: not on ground, so Control
    # keeps the drone as possibly armed
    errors = _receive(
        _uav(),
        armed=False,
        outputs_enabled=True,
        system_status=dialect.MAV_STATE_CRITICAL,
    )
    assert FlockwaveErrorCode.ON_GROUND.value not in errors
    assert FlockwaveErrorCode.DISARMED.value not in errors
