"""MAVLink and MAVFTP operations used by ArduPilot SD-card updates."""

from __future__ import annotations

from contextlib import aclosing
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, AsyncIterator, Protocol, cast

import trio

from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.show.utils import crc32_mavftp

from ..enums import MAVLandedState, MAVMessageType, MAVModeFlag
from ..ftp import MAVFTP, MAVFTPErrorCode, OperationNotAcknowledgedError

if TYPE_CHECKING:
    from ..driver import MAVLinkUAV
    from .apj import FirmwareImage

PART_PATH = "/ardupilot.abin.part"
READY_PATH = "/ardupilot.abin"
RESULT_PATHS = (
    "/ardupilot-verify.abin",
    "/ardupilot-flash.abin",
    "/ardupilot-flashed.abin",
    "/ardupilot-failed.abin",
)


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

    allowed_board_ids: frozenset[int] = frozenset((1177,))
    simulation_reported_board_id_overrides: tuple[tuple[int, int], ...] = ()
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
        allowed = _parse_board_ids(configuration.get("allowed_board_ids", [1177]))
        overrides = _parse_board_overrides(
            configuration.get("simulation_reported_board_id_overrides", {})
        )
        return cls(
            allowed_board_ids=frozenset(allowed),
            simulation_reported_board_id_overrides=tuple(overrides.items()),
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


class UpdateBackend(Protocol):
    def check_safety(self, board_id: int) -> None: ...

    def stage(self, image: FirmwareImage) -> AsyncIterator[int]: ...

    async def verify_upload(self, image: FirmwareImage) -> None: ...

    async def commit(self) -> None: ...

    async def reboot(self) -> None: ...

    async def wait_for_disconnect(self) -> None: ...

    async def wait_for_reconnect(self) -> None: ...

    async def read_installed(self) -> InstalledFirmware: ...

    async def verify_flash_result(self) -> None: ...


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
        heartbeat = self._uav.get_last_message(MAVMessageType.HEARTBEAT)
        version = self._uav.get_last_message(MAVMessageType.AUTOPILOT_VERSION)
        extended_state = self._uav.get_last_message(MAVMessageType.EXTENDED_SYS_STATE)
        armed = bool(heartbeat and heartbeat.base_mode & MAVModeFlag.SAFETY_ARMED.value)
        reported_board_id = _board_id_from_version(version)
        board_id = self._configuration.effective_board_id(reported_board_id)
        on_ground = bool(
            extended_state
            and extended_state.landed_state == MAVLandedState.ON_GROUND.value
        )
        percentage = self._uav.status.battery.percentage
        power_sufficient = percentage is None or percentage >= 30
        reason = _target_reason(
            self._uav.is_connected,
            armed,
            on_ground,
            power_sufficient,
            board_id,
            self._configuration.allowed_board_ids,
        )
        return TargetState(
            id=self._uav.id,
            compatible=board_id in self._configuration.allowed_board_ids,
            connected=self._uav.is_connected,
            disarmed=not armed,
            on_ground=on_ground,
            power_sufficient=power_sufficient,
            board_id=board_id,
            current_hash=_git_hash_from_version(version),
            current_version=(
                _flight_version(version.flight_sw_version) if version else None
            ),
            reason_code=reason,
        )

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
        async with aclosing(MAVFTP.for_uav(self._uav)) as ftp:
            await _remove_if_present(ftp, PART_PATH)
            async with ftp.put_gen(image.abin, PART_PATH) as progress:
                async for item in progress:
                    percentage = item.percentage or 0
                    yield min(image.total_size, image.total_size * percentage // 100)

    async def verify_upload(self, image: FirmwareImage) -> None:
        expected = crc32_mavftp(image.abin)
        async with aclosing(MAVFTP.for_uav(self._uav)) as ftp:
            observed = await ftp.crc32(PART_PATH)
        if observed != expected:
            raise UpdateOperationError(
                "uploadHashMismatch",
                f"Remote CRC32 {observed:08x} does not match {expected:08x}",
            )

    async def commit(self) -> None:
        async with aclosing(MAVFTP.for_uav(self._uav)) as ftp:
            await _remove_if_present(ftp, READY_PATH)
            for path in RESULT_PATHS:
                await _remove_if_present(ftp, path)
            await ftp.rename(PART_PATH, READY_PATH)

    async def reboot(self) -> None:
        self._uav._clear_autopilot_capabilities()
        await self._uav.reboot_after_update()

    async def wait_for_disconnect(self) -> None:
        with trio.fail_after(self._configuration.disconnect_timeout):
            while self._uav.is_connected:
                await trio.sleep(0.2)

    async def wait_for_reconnect(self) -> None:
        with trio.fail_after(self._configuration.reconnect_timeout):
            while not self._uav.is_connected:
                await trio.sleep(0.5)

    async def read_installed(self) -> InstalledFirmware:
        with trio.fail_after(self._configuration.version_timeout):
            await self._uav.get_version_info()
            while True:
                message = self._uav.get_last_message(MAVMessageType.AUTOPILOT_VERSION)
                if message is not None:
                    return InstalledFirmware(
                        board_id=self._configuration.effective_board_id(
                            _board_id_from_version(message)
                        )
                        or 0,
                        git_hash=bytes(message.flight_custom_version)
                        .rstrip(b"\x00")
                        .decode("ascii", errors="replace")
                        .lower(),
                        version=_flight_version(message.flight_sw_version),
                    )
                await trio.sleep(0.2)

    async def verify_flash_result(self) -> None:
        with trio.move_on_after(self._configuration.result_timeout):
            while True:
                async with aclosing(MAVFTP.for_uav(self._uav)) as ftp:
                    entries: set[str] = set()
                    async with ftp.ls("/") as listing:
                        async for entry in listing:
                            entries.add(entry.name.lower())
                    failure = _flash_failure(entries)
                    if failure:
                        raise UpdateOperationError(*failure)
                    if "ardupilot-flashed.abin" in entries:
                        await _remove_if_present(ftp, "/ardupilot-flashed.abin")
                        return
                await trio.sleep(0.25)
        raise UpdateResultIndeterminateError(*_interrupted_flash_failure(entries))


class UpdateOperationError(RuntimeError):
    """A firmware transaction failure with a stable wire error code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


class UpdateResultIndeterminateError(RuntimeError):
    """A committed update whose terminal bootloader result is not observable."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


async def _remove_if_present(ftp: MAVFTP, path: str) -> None:
    try:
        await ftp.rm(path)
    except OperationNotAcknowledgedError as ex:
        if ex.code != MAVFTPErrorCode.FILE_NOT_FOUND:
            raise


def _board_id_from_version(version) -> int | None:
    if version is None:
        return None
    value = getattr(version, "board_version", None)
    return value >> 16 if isinstance(value, int) else None


def _target_reason(
    connected: bool,
    armed: bool,
    on_ground: bool,
    power_sufficient: bool,
    board_id: int | None,
    allowed_board_ids: frozenset[int],
) -> str | None:
    if not connected:
        return "disconnected"
    if armed:
        return "armed"
    if not on_ground:
        return "notOnGround"
    if not power_sufficient:
        return "batteryLow"
    if board_id is None:
        return "boardUnknown"
    if board_id not in allowed_board_ids:
        return "unsupportedBoard"
    return None


def _reason_detail(state: TargetState) -> str:
    details = {
        "disconnected": "UAV is disconnected",
        "armed": "UAV is armed",
        "notOnGround": "UAV does not report that it is on the ground",
        "batteryLow": "UAV battery is below 30%",
        "boardUnknown": "UAV board ID is not available",
        "unsupportedBoard": f"UAV board ID {state.board_id} is not supported",
    }
    if state.reason_code is None:
        return "UAV is not ready for an update"
    return details.get(state.reason_code, "UAV is not ready for an update")


def _flight_version(value: int) -> str:
    return ".".join(str((value >> shift) & 0xFF) for shift in (24, 16, 8))


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


def _parse_board_ids(value: object) -> set[int]:
    if not isinstance(value, list) or not value:
        raise ValueError("firmware_update.allowed_board_ids must be a non-empty list")
    board_ids: set[int] = set()
    for item in value:
        if type(item) is not int or item != 1177:
            raise ValueError(
                "firmware_update.allowed_board_ids contains an unsupported board"
            )
        board_ids.add(item)
    return board_ids


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


def _parse_timeout(value: dict[str, Any], key: str, default: float) -> float:
    timeout = value.get(key, default)
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
        raise ValueError(f"firmware_update.{key} must be a number")
    timeout = float(timeout)
    if not 0.1 <= timeout <= 600:
        raise ValueError(f"firmware_update.{key} must be between 0.1 and 600 seconds")
    return timeout
