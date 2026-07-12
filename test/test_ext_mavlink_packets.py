from flockwave.server.ext.mavlink.packets import (
    DroneShowExecutionStage,
    DroneShowStatus,
    DroneShowStatusFlag,
    ShowStartState,
    authorization_scope_from_int,
    authorization_scope_to_int,
)
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.model.gps import GPSFixType


def test_authorization_scope_to_int():
    assert authorization_scope_to_int(AuthorizationScope.NONE) == 0
    assert authorization_scope_to_int(AuthorizationScope.LIVE) == 1
    assert authorization_scope_to_int(AuthorizationScope.REHEARSAL) == 2
    assert authorization_scope_to_int(AuthorizationScope.LIGHTS_ONLY) == 3
    assert authorization_scope_to_int("something_else") == 0  # type: ignore


def test_authorization_scope_from_int():
    assert authorization_scope_from_int(0) == AuthorizationScope.NONE
    assert authorization_scope_from_int(1) == AuthorizationScope.LIVE
    assert authorization_scope_from_int(2) == AuthorizationScope.REHEARSAL
    assert authorization_scope_from_int(3) == AuthorizationScope.LIGHTS_ONLY
    assert authorization_scope_from_int(4) == AuthorizationScope.NONE
    assert authorization_scope_from_int(-1) == AuthorizationScope.NONE
    assert authorization_scope_from_int("something_else") == AuthorizationScope.NONE  # type: ignore


def test_drone_show_status_from_bytes():
    # Legacy packet, length 9, no flags3 or elapsed_time field
    status = DroneShowStatus.from_bytes(b"\x01\x02\x03\x04\x05\x06\x07\x88\x19")

    assert status.start_time == 67305985
    assert status.light == 1541
    assert status.flags == (
        DroneShowStatusFlag.GEOFENCE_BREACHED
        | DroneShowStatusFlag.IS_GPS_TIME_BAD
        | DroneShowStatusFlag.HAS_AUTHORIZATION_TO_START
        | DroneShowStatusFlag.IS_MISPLACED_BEFORE_TAKEOFF
    )
    assert status.stage is DroneShowExecutionStage.LANDED
    assert status.gps_fix is GPSFixType.NO_FIX
    assert status.num_satellites == 3
    assert status.authorization_scope is AuthorizationScope.LIVE

    # Legacy packet, same as above, but the authorization flag is cleared
    # and the "misplaced before takeoff" flag is also cleared
    status = DroneShowStatus.from_bytes(b"\x01\x02\x03\x04\x05\x06\x03\x08\x19")
    assert status.flags == (
        DroneShowStatusFlag.GEOFENCE_BREACHED | DroneShowStatusFlag.IS_GPS_TIME_BAD
    )
    assert status.authorization_scope is AuthorizationScope.NONE

    # v2 packet, length 12
    status = DroneShowStatus.from_bytes(
        b"\x01\x02\x03\x04\x05\x06\x07\x88\x19\xcf\x0a\x0b"
    )

    assert status.start_time == 67305985
    assert status.light == 1541
    assert status.flags == (
        DroneShowStatusFlag.GEOFENCE_BREACHED
        | DroneShowStatusFlag.IS_GPS_TIME_BAD
        | DroneShowStatusFlag.HAS_AUTHORIZATION_TO_START
        | DroneShowStatusFlag.IS_MISPLACED_BEFORE_TAKEOFF
        | DroneShowStatusFlag.IS_FAR_FROM_EXPECTED_POSITION
        | DroneShowStatusFlag.HAS_HIGH_ESC_ERROR_RATE
    )
    assert status.stage is DroneShowExecutionStage.LANDED
    assert status.gps_fix is GPSFixType.NO_FIX
    assert status.num_satellites == 3
    assert status.authorization_scope is AuthorizationScope.LIGHTS_ONLY
    assert status.elapsed_time == 2826
    assert status.has_high_esc_error_rate


def test_drone_show_status_decodes_start_state():
    packet = bytearray(14)
    packet[9] = ShowStartState.UWB_LTC_COMMITTED << 4

    status = DroneShowStatus.from_bytes(bytes(packet))

    assert status.start_state is ShowStartState.UWB_LTC_COMMITTED


def test_show_sync_json_reports_committed_deadline():
    from flockwave.server.ext.mavlink.extension import _show_sync_json

    status = DroneShowStatus(
        elapsed_time=-12,
        flags=DroneShowStatusFlag.HAS_START_TIME,
        start_state=ShowStartState.UWB_LTC_COMMITTED,
    )

    assert _show_sync_json(status) == {
        "source": "uwb-ltc",
        "locked": True,
        "committed": True,
        "scheduled": True,
        "secondsToStart": 12,
    }
