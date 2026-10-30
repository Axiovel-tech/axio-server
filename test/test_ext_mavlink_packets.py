from struct import unpack

from flockwave.gps.time import unix_to_gps_time_of_week

from flockwave.server.ext.mavlink import packets
from flockwave.server.ext.mavlink.packets import (
    DroneShowExecutionStage,
    DroneShowStatus,
    DroneShowStatusFlag,
    authorization_scope_from_int,
    authorization_scope_to_int,
    create_start_time_configuration_packet,
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

    assert status.start_time == 0x30201
    assert status.start_time_msec == 0x40
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

    assert status.start_time == 0x30201
    assert status.start_time_msec == 0x40
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


def _unpack_start_config(spec):
    name, fields = spec
    assert name == "DATA16"
    assert fields["len"] == 12
    data = fields["data"]
    assert data[0] == 1
    return unpack("<iBiH", data[1:12])


def test_drone_show_status_unpacks_the_start_time_milliseconds():
    # Firmware: seconds of week in the low 20 bits, milliseconds above them
    packed = 414_531 | (750 << 20)
    status = DroneShowStatus.from_bytes(packed.to_bytes(4, "little", signed=True))
    assert status.start_time == 414_531
    assert status.start_time_msec == 750

    unset = DroneShowStatus.from_bytes((-1).to_bytes(4, "little", signed=True))
    assert unset.start_time == -1
    assert unset.start_time_msec == 0


def test_start_config_packet_carries_the_sub_second_start_time(monkeypatch):
    now = 1_760_000_000.25
    monkeypatch.setattr(packets, "time", lambda: now)

    start_time = 1_760_000_012.3456
    spec = create_start_time_configuration_packet(
        AuthorizationScope.REHEARSAL, start_time=start_time
    )

    tow, scope, countdown_msec, start_time_msec = _unpack_start_config(spec)
    assert tow == unix_to_gps_time_of_week(1_760_000_012)[1]
    assert scope == 2
    assert countdown_msec == 12096
    assert start_time_msec == 346


def test_start_config_packet_rounds_up_into_the_next_second(monkeypatch):
    monkeypatch.setattr(packets, "time", lambda: 1_760_000_000.0)

    spec = create_start_time_configuration_packet(
        AuthorizationScope.LIVE, start_time=1_760_000_012.9996
    )

    tow, _, countdown_msec, start_time_msec = _unpack_start_config(spec)
    assert tow == unix_to_gps_time_of_week(1_760_000_013)[1]
    assert countdown_msec == 13000
    assert start_time_msec == 0


def test_start_config_packet_clear_and_keep_have_no_milliseconds():
    cleared = _unpack_start_config(
        create_start_time_configuration_packet(AuthorizationScope.NONE, None)
    )
    assert cleared == (-0x80000000, 0, -0x80000000, 0)

    kept = _unpack_start_config(
        create_start_time_configuration_packet(
            AuthorizationScope.LIVE, 1_760_000_012.5, should_update_takeoff_time=False
        )
    )
    assert kept == (0x7FFFFFFF, 1, 0x7FFFFFFF, 0)
