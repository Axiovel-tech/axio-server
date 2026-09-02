"""MAVLink and MAVFTP operations used by ArduPilot SD-card updates."""

from __future__ import annotations

from contextlib import aclosing
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, AsyncIterator, cast

import trio
from flockwave.concurrency import AdaptiveExponentialBackoffPolicy

from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.logger import log as base_log

from ..enums import MAVLandedState, MAVMessageType, MAVModeFlag
from ..ftp import MAVFTP, MAVFTPErrorCode, OperationNotAcknowledgedError
from ..utils import (
    can_communicate_infer_from_heartbeat,
    mavlink_version_number_to_semver,
)
from .apj import MAX_IMAGE_SIZE_BY_BOARD

if TYPE_CHECKING:
    from ..driver import MAVLinkUAV
    from .apj import FirmwareImage

log = base_log.getChild("ext.mavlink.firmware")

PART_PATH = "/ardupilot.abin.part"
READY_PATH = "/ardupilot.abin"
RESULT_PATHS = (
    "/ardupilot-verify.abin",
    "/ardupilot-flash.abin",
    "/ardupilot-flashed.abin",
    "/ardupilot-failed.abin",
)
MAX_SAFETY_MESSAGE_AGE = 3.0
OTA_MAVFTP_MAX_RETRIES = 20
OTA_MAVFTP_INITIAL_TIMEOUT = 1.0


@dataclass(frozen=True)
class TargetState:
    """Firmware-update-relevant state reported by one MAVLink UAV."""

    id: str
    compatible: bool
    connected: bool
    disarmed: bool
    on_ground: bool
    power_sufficient: bool
    board_id: int | None
    current_hash: str | None
    current_version: str | None
    reason_code: str | None


@dataclass(frozen=True)
class InstalledFirmware:
    board_id: int
    git_hash: str
    version: str


@dataclass(frozen=True)
class FirmwareUpdateConfiguration:
    """Validated MAVLink extension settings for application firmware OTA."""

    provisioned_uav_ids: frozenset[str] = frozenset()
    simulation_reported_board_id_overrides: tuple[tuple[int, int], ...] = ()
    minimum_battery_voltage: float | None = None
    disconnect_timeout: float = 15.0
    reconnect_timeout: float = 180.0
    result_timeout: float = 15.0
    version_timeout: float = 15.0

    @classmethod
    def from_json(cls, value: object) -> FirmwareUpdateConfiguration:
        if value is None:
            return cls()
        if not isinstance(value, dict):
            raise ValueError("firmware_update must be an object")
        configuration = cast(dict[str, Any], value)
        provisioned = _parse_uav_ids(configuration.get("provisioned_uav_ids", []))
        overrides = _parse_board_overrides(
            configuration.get("simulation_reported_board_id_overrides", {})
        )
        return cls(
            provisioned_uav_ids=frozenset(provisioned),
            simulation_reported_board_id_overrides=tuple(overrides.items()),
            minimum_battery_voltage=_parse_optional_voltage(
                configuration.get("minimum_battery_voltage")
            ),
            disconnect_timeout=_parse_timeout(
                configuration, "disconnect_timeout", 15.0
            ),
            reconnect_timeout=_parse_timeout(configuration, "reconnect_timeout", 180.0),
            result_timeout=_parse_timeout(configuration, "result_timeout", 15.0),
            version_timeout=_parse_timeout(configuration, "version_timeout", 15.0),
        )

    def effective_board_id(self, reported: int | None) -> int | None:
        if reported is None:
            return None
        return dict(self.simulation_reported_board_id_overrides).get(reported, reported)

    def is_simulated_board(self, reported: int | None) -> bool:
        """Whether an explicit simulator-only mapping owns this board ID."""
        return reported is not None and any(
            source == reported
            for source, _target in self.simulation_reported_board_id_overrides
        )


class ArduPilotUpdateBackend:
    """Runs an SD-staged update against one connected ArduPilot UAV."""

    def __init__(
        self,
        uav: MAVLinkUAV,
        configuration: FirmwareUpdateConfiguration | None = None,
    ):
        self._uav = uav
        self._configuration = configuration or FirmwareUpdateConfiguration()

    def target_state(self) -> TargetState:
        heartbeat = self._fresh_message(MAVMessageType.HEARTBEAT)
        version = self._uav.get_last_message(MAVMessageType.AUTOPILOT_VERSION)
        extended_state = self._fresh_message(MAVMessageType.EXTENDED_SYS_STATE)
        sys_status = self._fresh_message(MAVMessageType.SYS_STATUS)
        armed = bool(heartbeat and heartbeat.base_mode & MAVModeFlag.SAFETY_ARMED.value)
        reported_board_id = _board_id_from_version(version)
        board_id = self._configuration.effective_board_id(reported_board_id)
        simulated = self._configuration.is_simulated_board(reported_board_id)
        on_ground = simulated or bool(
            extended_state
            and extended_state.landed_state == MAVLandedState.ON_GROUND.value
        )
        voltage = self._uav.status.battery.voltage
        threshold = self._configuration.minimum_battery_voltage
        power_observed = simulated or bool(
            sys_status is not None
            and threshold is not None
            and voltage is not None
            and voltage > 0
        )
        power_sufficient = simulated or bool(
            sys_status is not None
            and threshold is not None
            and voltage is not None
            and voltage >= threshold
        )
        connected = self._uav.is_connected and can_communicate_infer_from_heartbeat(
            heartbeat
        )
        reason = _target_reason(
            connected,
            armed,
            on_ground,
            power_observed,
            power_sufficient,
            board_id,
            self._uav.id in self._configuration.provisioned_uav_ids,
        )
        return TargetState(
            id=self._uav.id,
            compatible=(
                board_id in MAX_IMAGE_SIZE_BY_BOARD
                and self._uav.id in self._configuration.provisioned_uav_ids
            ),
            connected=connected,
            disarmed=not armed,
            on_ground=on_ground,
            power_sufficient=power_sufficient,
            board_id=board_id,
            current_hash=_git_hash_from_version(version),
            current_version=(
                mavlink_version_number_to_semver(version.flight_sw_version)
                if version
                else None
            ),
            reason_code=reason,
        )

    def _fresh_message(self, message_type: MAVMessageType):
        if self._uav.get_age_of_message(message_type) > MAX_SAFETY_MESSAGE_AGE:
            return None
        return self._uav.get_last_message(message_type)

    def check_safety(self, board_id: int) -> None:
        state = self.target_state()
        if state.reason_code:
            raise UpdateOperationError(state.reason_code, _reason_detail(state))
        if state.board_id != board_id:
            raise UpdateOperationError(
                "boardMismatch",
                f"Firmware board ID {board_id} does not match UAV board ID {state.board_id}",
            )
        if self._uav.scheduled_takeoff_time is not None:
            raise UpdateOperationError("showScheduled", "UAV has a scheduled takeoff")
        if (
            self._uav.scheduled_takeoff_authorization_scope
            is not AuthorizationScope.NONE
        ):
            raise UpdateOperationError("showAuthorized", "UAV is authorized for a show")

    async def stage(self, image: FirmwareImage) -> AsyncIterator[int]:
        ftp = _make_update_ftp(self._uav)
        try:
            await _remove_if_present(ftp, PART_PATH)
            await _remove_if_present(ftp, READY_PATH)
            for path in RESULT_PATHS:
                await _remove_if_present(ftp, path)
            async with ftp.put_gen(image.abin, PART_PATH) as progress:
                async for item in progress:
                    percentage = item.percentage or 0
                    yield min(image.total_size, image.total_size * percentage // 100)
        finally:
            # A user cancellation arrives through the coordinator's cancel scope.
            # Reset the remote sessions before that cancellation can unwind the
            # upload, otherwise an immediate retry may inherit an open file handle.
            with trio.CancelScope(shield=True):
                await ftp.aclose()

    async def commit(self) -> None:
        async with aclosing(_make_update_ftp(self._uav)) as ftp:
            try:
                await ftp.rename(PART_PATH, READY_PATH)
            except OperationNotAcknowledgedError as ex:
                raise CommitRejectedError(
                    "commitRejected",
                    f"Flight controller rejected the staged image: {ex}",
                ) from ex

    async def reboot(self) -> None:
        await self._uav.reboot_after_update()

    async def wait_for_disconnect(self) -> None:
        with trio.fail_after(self._configuration.disconnect_timeout):
            while self._uav.is_connected and can_communicate_infer_from_heartbeat(
                self._uav.get_last_message(MAVMessageType.HEARTBEAT)
            ):
                await trio.sleep(0.2)

    async def wait_for_reconnect(self) -> None:
        with trio.fail_after(self._configuration.reconnect_timeout):
            await self._uav.wait_until_connected()

    async def refresh_version_info(self) -> None:
        """Discard any pre-reconnect identity and request it from the live FC."""
        with trio.fail_after(self._configuration.version_timeout):
            self._uav.invalidate_version_info()
            await self._uav.get_version_info()

    async def read_installed(self) -> InstalledFirmware:
        await self.refresh_version_info()
        message = self._uav.get_last_message(MAVMessageType.AUTOPILOT_VERSION)
        if message is None:
            raise RuntimeError("Autopilot version was not cached after requesting it")
        board_id = self._configuration.effective_board_id(
            _board_id_from_version(message)
        )
        if board_id is None:
            raise RuntimeError("Fresh autopilot version has no board identity")
        return InstalledFirmware(
            board_id=board_id,
            git_hash=_git_hash_from_version(message) or "",
            version=mavlink_version_number_to_semver(message.flight_sw_version),
        )

    async def verify_flash_result(self) -> None:
        with trio.move_on_after(self._configuration.result_timeout):
            while True:
                async with aclosing(_make_update_ftp(self._uav)) as ftp:
                    entries: set[str] = set()
                    async with ftp.ls("/") as listing:
                        async for entry in listing:
                            entries.add(entry.name.lower())
                    failure = _flash_failure(entries)
                    if failure:
                        raise UpdateOperationError(*failure)
                    if "ardupilot-flashed.abin" in entries:
                        try:
                            await _remove_if_present(ftp, "/ardupilot-flashed.abin")
                        except Exception:
                            log.warning(
                                "Failed to remove ArduPilot OTA success marker",
                                exc_info=True,
                                extra={"id": self._uav.id},
                            )
                        return
                await trio.sleep(0.25)
        raise UpdateResultIndeterminateError(*_interrupted_flash_failure(entries))


class UpdateOperationError(RuntimeError):
    """A firmware transaction failure with a stable wire error code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


class CommitRejectedError(UpdateOperationError):
    """A definite pre-commit rejection of the atomic MAVFTP rename."""


class UpdateResultIndeterminateError(UpdateOperationError):
    """A committed update whose terminal bootloader result is not observable."""


async def _remove_if_present(ftp: MAVFTP, path: str) -> None:
    try:
        await ftp.rm(path)
    except OperationNotAcknowledgedError as ex:
        if ex.code != MAVFTPErrorCode.FILE_NOT_FOUND:
            raise


def _make_update_ftp(uav: MAVLinkUAV) -> MAVFTP:
    """Create an OTA session without flooding a high-latency MAVLink link."""
    return MAVFTP.for_uav(
        uav,
        retry_policy=AdaptiveExponentialBackoffPolicy(
            max_retries=OTA_MAVFTP_MAX_RETRIES,
            base_timeout=OTA_MAVFTP_INITIAL_TIMEOUT,
            max_timeout=3,
        ),
    )


def _board_id_from_version(version) -> int | None:
    if version is None:
        return None
    value = getattr(version, "board_version", None)
    return value >> 16 if isinstance(value, int) else None


def _target_reason(
    connected: bool,
    armed: bool,
    on_ground: bool,
    power_observed: bool,
    power_sufficient: bool,
    board_id: int | None,
    bootloader_provisioned: bool,
) -> str | None:
    if not connected:
        return "disconnected"
    if armed:
        return "armed"
    if not on_ground:
        return "notOnGround"
    if board_id is None:
        return "boardUnknown"
    if board_id not in MAX_IMAGE_SIZE_BY_BOARD:
        return "unsupportedBoard"
    if not bootloader_provisioned:
        return "bootloaderNotProvisioned"
    if not power_observed:
        return "batteryUnknown"
    if not power_sufficient:
        return "batteryLow"
    return None


def _reason_detail(state: TargetState) -> str:
    details: dict[str | None, str] = {
        "disconnected": "UAV is disconnected",
        "armed": "UAV is armed",
        "notOnGround": "UAV does not report that it is on the ground",
        "batteryUnknown": (
            "UAV battery voltage is unavailable or its minimum is not configured"
        ),
        "batteryLow": "UAV battery voltage is below the configured minimum",
        "boardUnknown": "UAV board ID is not available",
        "unsupportedBoard": f"UAV board ID {state.board_id} is not supported",
        "bootloaderNotProvisioned": "UAV is not provisioned with the OTA bootloader",
    }
    return details.get(state.reason_code, "UAV is not ready for an update")


def _flash_failure(entries: set[str]) -> tuple[str, str] | None:
    if "ardupilot-failed.abin" in entries:
        return "imageRejected", "Bootloader rejected the update image"
    return None


def _interrupted_flash_failure(entries: set[str]) -> tuple[str, str]:
    if "ardupilot-verify.abin" in entries:
        return "verificationInterrupted", "Bootloader verification did not finish"
    if "ardupilot-flash.abin" in entries:
        return "flashingInterrupted", "Bootloader flashing did not finish"
    if "ardupilot.abin" in entries:
        return "updateUnsupported", "Bootloader did not process the staged update"
    return "resultMissing", "Bootloader did not leave an update result marker"


def _git_hash_from_version(version) -> str | None:
    if version is None:
        return None
    raw_hash = bytes(version.flight_custom_version).rstrip(b"\x00")
    text = raw_hash.decode(errors="replace")
    return text.lower() or None


def _parse_uav_ids(value: object) -> set[str]:
    if not isinstance(value, list):
        raise ValueError("firmware_update.provisioned_uav_ids must be a list")
    uav_ids: set[str] = set()
    for item in value:
        if not isinstance(item, str) or not item or len(item) > 128:
            raise ValueError(
                "firmware_update.provisioned_uav_ids contains an invalid UAV ID"
            )
        if item in uav_ids:
            raise ValueError(
                "firmware_update.provisioned_uav_ids contains a duplicate UAV ID"
            )
        uav_ids.add(item)
    return uav_ids


def _parse_board_overrides(value: object) -> dict[int, int]:
    if not isinstance(value, dict):
        raise ValueError(
            "firmware_update.simulation_reported_board_id_overrides must be an object"
        )
    overrides: dict[int, int] = {}
    for source, target in value.items():
        if source != "0" or target != 1177:
            raise ValueError(
                "Only the simulation board override 0 -> 1177 is supported"
            )
        overrides[0] = 1177
    return overrides


def _parse_optional_voltage(value: object) -> float | None:
    if value is None:
        return None
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError("firmware_update.minimum_battery_voltage must be a number")
    voltage = float(value)
    if not 0.1 <= voltage <= 100:
        raise ValueError(
            "firmware_update.minimum_battery_voltage must be between 0.1 and 100 volts"
        )
    return voltage


def _parse_timeout(value: dict[str, Any], key: str, default: float) -> float:
    timeout = value.get(key, default)
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
        raise ValueError(f"firmware_update.{key} must be a number")
    timeout = float(timeout)
    if not 0.1 <= timeout <= 600:
        raise ValueError(f"firmware_update.{key} must be between 0.1 and 600 seconds")
    return timeout
