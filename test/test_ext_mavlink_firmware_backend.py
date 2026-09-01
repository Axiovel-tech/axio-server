"""Tests for ArduPilot firmware target configuration and discovery."""

from contextlib import asynccontextmanager
from types import SimpleNamespace
from typing import cast

import pytest
import trio

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.enums import MAVMessageType
from flockwave.server.ext.mavlink.firmware.backend import (
    ArduPilotUpdateBackend,
    FirmwareUpdateConfiguration,
    TargetState,
    UpdateOperationError,
    UpdateResultIndeterminateError,
    _board_id_from_version,
    _flash_failure,
    _flight_version,
    _git_hash_from_version,
    _interrupted_flash_failure,
    _reason_detail,
    _target_reason,
)
from flockwave.server.ext.mavlink.ftp import MAVFTP
from flockwave.server.ext.show.config import AuthorizationScope


class FakeUAV:
    id = "1"

    def __init__(
        self,
        board_id: int,
        *,
        armed: bool = False,
        battery: int | None = 100,
        connected: bool = True,
        landed_state: int | None = 1,
    ):
        self.is_connected = connected
        self.scheduled_takeoff_time = None
        self.scheduled_takeoff_authorization_scope = AuthorizationScope.NONE
        self.status = SimpleNamespace(battery=SimpleNamespace(percentage=battery))
        self._messages = {
            MAVMessageType.HEARTBEAT: SimpleNamespace(base_mode=128 if armed else 0),
            MAVMessageType.AUTOPILOT_VERSION: SimpleNamespace(
                board_version=board_id << 16,
                flight_custom_version=b"0123abcd",
                flight_sw_version=4 << 24 | 6 << 16 | 1 << 8 | 255,
            ),
        }
        if landed_state is not None:
            self._messages[MAVMessageType.EXTENDED_SYS_STATE] = SimpleNamespace(
                landed_state=landed_state
            )

    def get_last_message(self, message_type):
        return self._messages.get(message_type)


def test_production_configuration_accepts_only_axiolight() -> None:
    state = ArduPilotUpdateBackend(cast(MAVLinkUAV, FakeUAV(1177))).target_state()
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


def test_sitl_board_zero_requires_explicit_override() -> None:
    without_override = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(0))
    ).target_state()
    assert not without_override.compatible
    assert without_override.reason_code == "unsupportedBoard"

    configuration = FirmwareUpdateConfiguration.from_json(
        {"simulation_reported_board_id_overrides": {"0": 1177}}
    )
    with_override = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(0)), configuration
    ).target_state()
    assert with_override.compatible
    assert with_override.board_id == 1177
    assert with_override.reason_code is None


@pytest.mark.parametrize(
    "configuration",
    [
        [],
        {"allowed_board_ids": []},
        {"allowed_board_ids": "1177"},
        {"allowed_board_ids": [True]},
        {"simulation_reported_board_id_overrides": {"42": 1177}},
        {"simulation_reported_board_id_overrides": {"0": 42}},
        {"simulation_reported_board_id_overrides": {0: 1177}},
        {"simulation_reported_board_id_overrides": []},
        {"allowed_board_ids": [0]},
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
    assert defaults.effective_board_id(None) is None
    assert defaults.effective_board_id(1177) == 1177

    configuration = FirmwareUpdateConfiguration.from_json(
        {
            "allowed_board_ids": [1177, 1177],
            "simulation_reported_board_id_overrides": {"0": 1177},
            "disconnect_timeout": 0.1,
            "reconnect_timeout": 600,
            "result_timeout": 3,
            "version_timeout": 4.5,
        }
    )
    assert configuration == FirmwareUpdateConfiguration(
        allowed_board_ids=frozenset({1177}),
        simulation_reported_board_id_overrides=((0, 1177),),
        disconnect_timeout=0.1,
        reconnect_timeout=600.0,
        result_timeout=3.0,
        version_timeout=4.5,
    )
    assert configuration.effective_board_id(0) == 1177


@pytest.mark.parametrize(
    ("uav", "code"),
    [
        (FakeUAV(1177, connected=False), "disconnected"),
        (FakeUAV(1177, armed=True), "armed"),
        (FakeUAV(1177, landed_state=None), "notOnGround"),
        (FakeUAV(1177, battery=29), "batteryLow"),
        (FakeUAV(42), "unsupportedBoard"),
    ],
)
def test_target_safety_reasons_are_reported_in_priority_order(
    uav: FakeUAV, code: str
) -> None:
    state = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav)).target_state()
    assert state.reason_code == code


def test_unknown_board_and_unknown_battery_are_handled_explicitly() -> None:
    uav = FakeUAV(1177, battery=None)
    del uav._messages[MAVMessageType.AUTOPILOT_VERSION]
    state = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav)).target_state()
    assert state.power_sufficient is True
    assert state.board_id is None
    assert state.compatible is False
    assert state.current_hash is None
    assert state.current_version is None
    assert state.reason_code == "boardUnknown"


def test_battery_threshold_is_inclusive() -> None:
    state = ArduPilotUpdateBackend(
        cast(MAVLinkUAV, FakeUAV(1177, battery=30))
    ).target_state()
    assert state.power_sufficient is True
    assert state.reason_code is None


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
        ("batteryLow", 1177, "UAV battery is below 30%"),
        ("boardUnknown", None, "UAV board ID is not available"),
        ("unsupportedBoard", 42, "UAV board ID 42 is not supported"),
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
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav))
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
    assert _flight_version(version.flight_sw_version) == "4.6.1"
    assert _git_hash_from_version(None) is None
    assert _git_hash_from_version(version) == "abcdef1"
    version.flight_custom_version = b"\x00" * 8
    assert _git_hash_from_version(version) is None
    version.flight_custom_version = b"abcX\x00"
    assert _git_hash_from_version(version) == "abcx"
    version.flight_custom_version = b"\xff"
    assert _git_hash_from_version(version) == "�"


def test_target_reason_function_allows_only_ready_supported_target() -> None:
    allowed = frozenset({1177})
    assert _target_reason(True, False, True, True, 1177, allowed) is None
    assert _target_reason(False, True, False, False, None, allowed) == "disconnected"


@pytest.mark.parametrize(
    ("configuration", "detail"),
    [
        (
            {"allowed_board_ids": []},
            "firmware_update.allowed_board_ids must be a non-empty list",
        ),
        (
            {"allowed_board_ids": [0]},
            "firmware_update.allowed_board_ids contains an unsupported board",
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


class MarkerFTP:
    def __init__(self, entries: set[str]):
        self.entries = entries
        self.removed: list[str] = []
        self.closed = False

    @asynccontextmanager
    async def ls(self, path: str):
        assert path == "/"

        async def generate():
            for name in self.entries:
                yield SimpleNamespace(name=name)

        yield generate()

    async def rm(self, path: str) -> None:
        self.removed.append(path)

    async def aclose(self) -> None:
        self.closed = True


async def test_flash_result_accepts_success_marker_and_removes_it(monkeypatch) -> None:
    uav = FakeUAV(1177)
    ftp = MarkerFTP({"ardupilot-flashed.abin"})

    def make_ftp(candidate):
        assert candidate is uav
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav))
    await backend.verify_flash_result()
    assert ftp.removed == ["/ardupilot-flashed.abin"]
    assert ftp.closed is True


async def test_flash_result_rejects_explicit_failure_marker(monkeypatch) -> None:
    ftp = MarkerFTP({"ardupilot-failed.abin"})
    monkeypatch.setattr(MAVFTP, "for_uav", lambda _uav: ftp)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, FakeUAV(1177)))
    with pytest.raises(UpdateOperationError) as raised:
        await backend.verify_flash_result()
    assert raised.value.code == "imageRejected"
    assert ftp.closed is True


async def test_flash_result_times_out_as_indeterminate(monkeypatch) -> None:
    ftp = MarkerFTP({"ardupilot-verify.abin"})
    monkeypatch.setattr(MAVFTP, "for_uav", lambda _uav: ftp)
    delays = []

    async def wait_for_next_poll(delay: float) -> None:
        delays.append(delay)
        await trio.sleep_forever()

    monkeypatch.setattr(trio, "sleep", wait_for_next_poll)
    configuration = FirmwareUpdateConfiguration(result_timeout=0.1)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, FakeUAV(1177)), configuration)
    with pytest.raises(UpdateResultIndeterminateError) as raised:
        await backend.verify_flash_result()
    assert raised.value.code == "verificationInterrupted"
    assert delays == [0.25]
