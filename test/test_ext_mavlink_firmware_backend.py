"""Tests for ArduPilot firmware target configuration and discovery."""

from types import SimpleNamespace
from typing import Any, cast

import pytest

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.enums import (
    ConnectionState,
    MAVCommand,
    MAVMessageType,
    MAVState,
)
from flockwave.server.ext.mavlink.firmware.backend import (
    MAX_SAFETY_MESSAGE_AGE,
    ArduPilotUpdateBackend,
    FirmwareUpdateConfiguration,
    TargetState,
    UpdateOperationError,
    _board_id_from_version,
    _flash_failure,
    _git_hash_from_version,
    _interrupted_flash_failure,
    _reason_detail,
    _target_reason,
)
from flockwave.server.ext.show.config import AuthorizationScope


class FakeUAV:
    id = "1"
    driver: Any

    def __init__(
        self,
        board_id: int,
        *,
        armed: bool = False,
        battery_voltage: float | None = 16,
        connected: bool = True,
        landed_state: int | None = 1,
    ):
        self.is_connected = connected
        self.scheduled_takeoff_time = None
        self.scheduled_takeoff_authorization_scope = AuthorizationScope.NONE
        self.status = SimpleNamespace(
            battery=SimpleNamespace(voltage=battery_voltage, percentage=None)
        )
        self._messages = {
            MAVMessageType.HEARTBEAT: SimpleNamespace(
                base_mode=128 if armed else 0,
                system_status=MAVState.STANDBY.value,
            ),
            MAVMessageType.AUTOPILOT_VERSION: SimpleNamespace(
                board_version=board_id << 16,
                flight_custom_version=b"0123abcd",
                flight_sw_version=4 << 24 | 6 << 16 | 1 << 8 | 255,
            ),
            MAVMessageType.SYS_STATUS: SimpleNamespace(),
        }
        self._message_ages = {}
        if landed_state is not None:
            self._messages[MAVMessageType.EXTENDED_SYS_STATE] = SimpleNamespace(
                landed_state=landed_state
            )

    def get_last_message(self, message_type):
        return self._messages.get(message_type)

    def get_age_of_message(self, message_type):
        return self._message_ages.get(message_type, 0)


PROVISIONED = FirmwareUpdateConfiguration(
    provisioned_uav_ids=frozenset({"1"}), minimum_battery_voltage=14
)


def make_backend(uav: FakeUAV) -> ArduPilotUpdateBackend:
    return ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), PROVISIONED)


async def test_normal_stream_configuration_requests_landed_state() -> None:
    calls: list[tuple[MAVCommand, int, float]] = []

    class Driver:
        async def send_command_long(
            self, _uav, command: MAVCommand, *, param1: int, param2: float
        ) -> bool:
            calls.append((command, param1, param2))
            return True

    uav = SimpleNamespace(driver=Driver())
    await MAVLinkUAV._configure_data_streams_with_fine_grained_commands(
        cast(MAVLinkUAV, uav)
    )

    assert (
        MAVCommand.SET_MESSAGE_INTERVAL,
        MAVMessageType.EXTENDED_SYS_STATE,
        1_000_000,
    ) in calls


def test_fresh_sys_status_does_not_hide_missing_landed_state() -> None:
    requested = []
    ages = {
        MAVMessageType.HEARTBEAT: 0,
        MAVMessageType.SYS_STATUS: 0,
        MAVMessageType.EXTENDED_SYS_STATE: 6,
    }
    uav = SimpleNamespace(
        _mavlink_version=2,
        _connection_state=ConnectionState.CONNECTED,
        _autopilot=SimpleNamespace(is_duplicate_message=lambda _message: True),
        _last_autopilot_capabilities_requested_at=None,
        driver=SimpleNamespace(
            assume_data_streams_configured=False,
            autopilot_factory=True,
        ),
        get_age_of_message=lambda message_type: ages[message_type],
        _store_message=lambda _message: None,
        _configure_data_streams_soon=lambda: requested.append(True),
        touch_status=lambda: None,
        notify_updated=lambda: None,
    )
    heartbeat = SimpleNamespace(
        get_msgbuf=lambda: b"\xfd",
        system_status=MAVState.STANDBY.value,
    )

    MAVLinkUAV.handle_message_heartbeat(cast(MAVLinkUAV, uav), heartbeat)

    assert requested == [True]


def test_production_configuration_accepts_only_axiolight() -> None:
    state = make_backend(FakeUAV(1177)).target_state()
    assert state == TargetState(
        id="1",
        compatible=True,
        connected=True,
        disarmed=True,
        on_ground=True,
        power_sufficient=True,
        board_id=1177,
        current_hash="0123abcd",
        current_version="4.6.1",
        reason_code=None,
    )


def test_default_configuration_rejects_unprovisioned_bootloader() -> None:
    state = ArduPilotUpdateBackend(cast(MAVLinkUAV, FakeUAV(1177))).target_state()
    assert state.compatible is False
    assert state.reason_code == "bootloaderNotProvisioned"


def test_sitl_board_zero_requires_explicit_override() -> None:
    without_override = make_backend(FakeUAV(0)).target_state()
    assert not without_override.compatible
    assert without_override.reason_code == "unsupportedBoard"

    configuration = FirmwareUpdateConfiguration.from_json(
        {
            "provisioned_uav_ids": ["1"],
            "simulation_reported_board_id_overrides": {"0": 1177},
        }
    )
    with_override = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(0)), configuration
    ).target_state()
    assert with_override.compatible
    assert with_override.board_id == 1177
    assert with_override.reason_code is None


def test_sitl_landed_override_does_not_weaken_hardware_safety() -> None:
    configuration = FirmwareUpdateConfiguration.from_json(
        {
            "provisioned_uav_ids": ["1"],
            "simulation_reported_board_id_overrides": {"0": 1177},
        }
    )

    sitl = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(0, landed_state=None)), configuration
    ).target_state()
    hardware = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(1177, landed_state=None)), configuration
    ).target_state()

    assert sitl.on_ground is True
    assert sitl.reason_code is None
    assert hardware.on_ground is False
    assert hardware.reason_code == "notOnGround"


@pytest.mark.parametrize(
    "configuration",
    [
        [],
        {"provisioned_uav_ids": "1"},
        {"provisioned_uav_ids": [""]},
        {"provisioned_uav_ids": [7]},
        {"provisioned_uav_ids": ["1", "1"]},
        {"provisioned_uav_ids": ["x" * 129]},
        {"simulation_reported_board_id_overrides": {"42": 1177}},
        {"simulation_reported_board_id_overrides": {"0": 42}},
        {"simulation_reported_board_id_overrides": {0: 1177}},
        {"simulation_reported_board_id_overrides": []},
        {"minimum_battery_voltage": 0},
        {"minimum_battery_voltage": 100.1},
        {"minimum_battery_voltage": True},
        {"minimum_battery_voltage": "14"},
        {"result_timeout": 0},
        {"result_timeout": 0.09},
        {"result_timeout": 601},
        {"result_timeout": True},
        {"result_timeout": "15"},
    ],
)
def test_configuration_rejects_unsafe_values(configuration) -> None:
    with pytest.raises(ValueError):
        FirmwareUpdateConfiguration.from_json(configuration)


def test_bootloader_marker_states_are_not_confused() -> None:
    assert _flash_failure({"ardupilot-failed.abin"}) == (
        "imageRejected",
        "Bootloader rejected the update image",
    )
    assert _flash_failure({"ardupilot-verify.abin"}) is None
    assert _flash_failure({"ardupilot-flash.abin"}) is None
    assert _interrupted_flash_failure({"ardupilot-verify.abin"}) == (
        "verificationInterrupted",
        "Bootloader verification did not finish",
    )
    assert _interrupted_flash_failure({"ardupilot-flash.abin"}) == (
        "flashingInterrupted",
        "Bootloader flashing did not finish",
    )
    assert _interrupted_flash_failure({"ardupilot.abin"}) == (
        "updateUnsupported",
        "Bootloader did not process the staged update",
    )
    assert _interrupted_flash_failure(set()) == (
        "resultMissing",
        "Bootloader did not leave an update result marker",
    )


def test_configuration_parses_all_bounds_and_simulation_mapping() -> None:
    defaults = FirmwareUpdateConfiguration.from_json(None)
    assert defaults == FirmwareUpdateConfiguration()
    assert defaults.provisioned_uav_ids == frozenset()
    assert defaults.effective_board_id(None) is None
    assert defaults.effective_board_id(1177) == 1177
    assert FirmwareUpdateConfiguration.from_json(
        {"provisioned_uav_ids": ["x" * 128]}
    ).provisioned_uav_ids == frozenset({"x" * 128})

    configuration = FirmwareUpdateConfiguration.from_json(
        {
            "provisioned_uav_ids": ["1"],
            "simulation_reported_board_id_overrides": {"0": 1177},
            "minimum_battery_voltage": 13.2,
            "disconnect_timeout": 0.1,
            "reconnect_timeout": 600,
            "result_timeout": 3,
            "version_timeout": 4.5,
        }
    )
    assert configuration == FirmwareUpdateConfiguration(
        provisioned_uav_ids=frozenset({"1"}),
        simulation_reported_board_id_overrides=((0, 1177),),
        minimum_battery_voltage=13.2,
        disconnect_timeout=0.1,
        reconnect_timeout=600.0,
        result_timeout=3.0,
        version_timeout=4.5,
    )
    assert configuration.effective_board_id(0) == 1177
    for voltage in (0.1, 1.0, 100):
        parsed = FirmwareUpdateConfiguration.from_json(
            {"minimum_battery_voltage": voltage}
        )
        assert parsed.minimum_battery_voltage == voltage


@pytest.mark.parametrize(
    ("uav", "code"),
    [
        (FakeUAV(1177, connected=False), "disconnected"),
        (FakeUAV(1177, armed=True), "armed"),
        (FakeUAV(1177, landed_state=None), "notOnGround"),
        (FakeUAV(1177, battery_voltage=13.9), "batteryLow"),
        (FakeUAV(42), "unsupportedBoard"),
    ],
)
def test_target_safety_reasons_are_reported_in_priority_order(
    uav: FakeUAV, code: str
) -> None:
    state = make_backend(uav).target_state()
    assert state.reason_code == code


def test_unknown_board_and_unknown_battery_are_handled_explicitly() -> None:
    uav = FakeUAV(1177, battery_voltage=None)
    del uav._messages[MAVMessageType.AUTOPILOT_VERSION]
    state = make_backend(uav).target_state()
    assert state.power_sufficient is False
    assert state.board_id is None
    assert state.compatible is False
    assert state.current_hash is None
    assert state.current_version is None
    assert state.reason_code == "boardUnknown"


def test_unknown_hardware_battery_fails_closed() -> None:
    for voltage in (None, 0):
        backend = make_backend(FakeUAV(1177, battery_voltage=voltage))
        state = backend.target_state()
        assert state.power_sufficient is False
        assert state.reason_code == "batteryUnknown"
        with pytest.raises(UpdateOperationError) as raised:
            backend.check_safety(1177)
        assert raised.value.code == "batteryUnknown"
        assert str(raised.value) == (
            "UAV battery voltage is unavailable or its minimum is not configured"
        )

    low = make_backend(FakeUAV(1177, battery_voltage=0.5)).target_state()
    assert low.reason_code == "batteryLow"

    unconfigured = FirmwareUpdateConfiguration(provisioned_uav_ids=frozenset({"1"}))
    state = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(1177)), unconfigured
    ).target_state()
    assert state.power_sufficient is False
    assert state.reason_code == "batteryUnknown"


def test_battery_voltage_threshold_is_inclusive() -> None:
    state = make_backend(FakeUAV(1177, battery_voltage=14)).target_state()
    assert state.power_sufficient is True
    assert state.reason_code is None


@pytest.mark.parametrize(
    ("message_type", "code"),
    [
        (MAVMessageType.HEARTBEAT, "disconnected"),
        (MAVMessageType.EXTENDED_SYS_STATE, "notOnGround"),
        (MAVMessageType.SYS_STATUS, "batteryUnknown"),
    ],
)
def test_stale_safety_telemetry_fails_closed(message_type, code: str) -> None:
    uav = FakeUAV(1177)
    uav._message_ages[message_type] = MAX_SAFETY_MESSAGE_AGE + 0.01
    state = make_backend(uav).target_state()
    assert state.reason_code == code


def test_sleeping_flight_controller_is_not_connected_for_update() -> None:
    uav = FakeUAV(1177)
    uav._messages[MAVMessageType.HEARTBEAT].system_status = MAVState.BOOT.value
    state = make_backend(uav).target_state()
    assert state.connected is False
    assert state.reason_code == "disconnected"


def test_safety_telemetry_at_freshness_boundary_is_accepted() -> None:
    uav = FakeUAV(1177)
    uav._message_ages = {
        MAVMessageType.HEARTBEAT: MAX_SAFETY_MESSAGE_AGE,
        MAVMessageType.EXTENDED_SYS_STATE: MAX_SAFETY_MESSAGE_AGE,
        MAVMessageType.SYS_STATUS: MAX_SAFETY_MESSAGE_AGE,
    }
    assert make_backend(uav).target_state().reason_code is None


@pytest.mark.parametrize(
    ("code", "board_id", "detail"),
    [
        ("disconnected", 1177, "UAV is disconnected"),
        ("armed", 1177, "UAV is armed"),
        (
            "notOnGround",
            1177,
            "UAV does not report that it is on the ground",
        ),
        (
            "batteryUnknown",
            1177,
            "UAV battery voltage is unavailable or its minimum is not configured",
        ),
        (
            "batteryLow",
            1177,
            "UAV battery voltage is below the configured minimum",
        ),
        ("boardUnknown", None, "UAV board ID is not available"),
        ("unsupportedBoard", 42, "UAV board ID 42 is not supported"),
        (
            "bootloaderNotProvisioned",
            1177,
            "UAV is not provisioned with the OTA bootloader",
        ),
        ("other", 1177, "UAV is not ready for an update"),
    ],
)
def test_target_reason_details_are_stable(
    code: str, board_id: int | None, detail: str
) -> None:
    state = TargetState(
        id="1",
        compatible=False,
        connected=False,
        disarmed=False,
        on_ground=False,
        power_sufficient=False,
        board_id=board_id,
        current_hash=None,
        current_version=None,
        reason_code=code,
    )
    assert _reason_detail(state) == detail


def test_safety_gate_rechecks_board_and_show_state() -> None:
    uav = FakeUAV(1177)
    backend = make_backend(uav)
    backend.check_safety(1177)

    with pytest.raises(UpdateOperationError) as mismatch:
        backend.check_safety(42)
    assert mismatch.value.code == "boardMismatch"
    assert str(mismatch.value) == (
        "Firmware board ID 42 does not match UAV board ID 1177"
    )

    uav.scheduled_takeoff_time = 123
    with pytest.raises(UpdateOperationError) as scheduled:
        backend.check_safety(1177)
    assert scheduled.value.code == "showScheduled"
    assert str(scheduled.value) == "UAV has a scheduled takeoff"

    uav.scheduled_takeoff_time = None
    uav.scheduled_takeoff_authorization_scope = AuthorizationScope.LIVE
    with pytest.raises(UpdateOperationError) as authorized:
        backend.check_safety(1177)
    assert authorized.value.code == "showAuthorized"
    assert str(authorized.value) == "UAV is authorized for a show"


def test_version_helpers_preserve_reported_identity() -> None:
    version = SimpleNamespace(
        board_version=1177 << 16,
        flight_sw_version=4 << 24 | 6 << 16 | 1 << 8 | 255,
        flight_custom_version=b"AbCdEf1\x00\x00",
    )
    assert _board_id_from_version(None) is None
    assert _board_id_from_version(version) == 1177
    assert _board_id_from_version(SimpleNamespace()) is None
    assert _git_hash_from_version(None) is None
    assert _git_hash_from_version(version) == "abcdef1"
    version.flight_custom_version = b"\x00" * 8
    assert _git_hash_from_version(version) is None
    version.flight_custom_version = b"abcX\x00"
    assert _git_hash_from_version(version) == "abcx"
    version.flight_custom_version = b"\xff"
    assert _git_hash_from_version(version) == "�"


def test_target_reason_function_allows_only_ready_supported_target() -> None:
    assert _target_reason(True, False, True, True, True, 1177, True) is None
    assert (
        _target_reason(False, True, False, False, False, None, False) == "disconnected"
    )
    assert (
        _target_reason(True, False, True, True, True, 1177, False)
        == "bootloaderNotProvisioned"
    )


@pytest.mark.parametrize(
    ("configuration", "detail"),
    [
        (
            {"provisioned_uav_ids": {}},
            "firmware_update.provisioned_uav_ids must be a list",
        ),
        (
            {"provisioned_uav_ids": [""]},
            "firmware_update.provisioned_uav_ids contains an invalid UAV ID",
        ),
        (
            {"provisioned_uav_ids": ["1", "1"]},
            "firmware_update.provisioned_uav_ids contains a duplicate UAV ID",
        ),
        (
            {"simulation_reported_board_id_overrides": []},
            "firmware_update.simulation_reported_board_id_overrides must be an object",
        ),
        (
            {"simulation_reported_board_id_overrides": {"1": 1177}},
            "Only the simulation board override 0 -> 1177 is supported",
        ),
        (
            {"minimum_battery_voltage": True},
            "firmware_update.minimum_battery_voltage must be a number",
        ),
        (
            {"minimum_battery_voltage": 0},
            "firmware_update.minimum_battery_voltage must be between 0.1 and 100 volts",
        ),
        (
            {"result_timeout": True},
            "firmware_update.result_timeout must be a number",
        ),
        (
            {"result_timeout": 0},
            "firmware_update.result_timeout must be between 0.1 and 600 seconds",
        ),
    ],
)
def test_configuration_errors_have_stable_details(configuration, detail: str) -> None:
    with pytest.raises(ValueError) as raised:
        FirmwareUpdateConfiguration.from_json(configuration)
    assert str(raised.value) == detail
