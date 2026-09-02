"""Direct tests for the ArduPilot OTA MAVFTP and reboot transport."""

from contextlib import asynccontextmanager
from types import SimpleNamespace
from typing import cast
from unittest.mock import AsyncMock

import pytest
import trio
from test_ext_mavlink_firmware_backend import FakeUAV, make_backend

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.enums import MAVCommand, MAVMessageType
from flockwave.server.ext.mavlink.firmware.apj import FirmwareImage
from flockwave.server.ext.mavlink.firmware.backend import (
    PART_PATH,
    READY_PATH,
    RESULT_PATHS,
    ArduPilotUpdateBackend,
    FirmwareUpdateConfiguration,
    UpdateOperationError,
    UpdateResultIndeterminateError,
    _remove_if_present,
    log,
)
from flockwave.server.ext.mavlink.ftp import (
    MAVFTP,
    MAVFTPErrorCode,
    OperationNotAcknowledgedError,
)


async def test_sitl_stage_quiets_truth_then_uses_crc_upload(monkeypatch) -> None:
    abin = b"x" * 200
    events = []

    class FTP:
        def __init__(self):
            self.removed = []
            self.upload = None
            self.closed = False

        async def rm(self, path: str) -> None:
            events.append(("rm", path))
            self.removed.append(path)

        @asynccontextmanager
        async def put_gen(self, data: bytes, path: str):
            self.upload = (data, path)

            async def progress():
                yield SimpleNamespace(percentage=None)
                yield SimpleNamespace(percentage=50)
                yield SimpleNamespace(percentage=100)

            yield progress()

        async def aclose(self) -> None:
            self.closed = True

    ftp = FTP()
    uav = FakeUAV(0)

    class Driver:
        async def send_command_long(
            self, candidate, command, *, param1: int, param2: float
        ) -> bool:
            events.append(("command", candidate, command, param1, param2))
            return True

    uav.driver = Driver()

    def make_ftp(candidate, *, retry_policy):
        assert candidate is uav
        assert retry_policy.base_timeout == 1
        assert retry_policy.max_retries == 20
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    image = SimpleNamespace(abin=abin, total_size=len(abin))

    configuration = FirmwareUpdateConfiguration.from_json(
        {
            "provisioned_uav_ids": ["1"],
            "simulation_reported_board_id_overrides": {"0": 1177},
        }
    )
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    transferred = [amount async for amount in backend.stage(cast(FirmwareImage, image))]

    assert events[0] == (
        "command",
        uav,
        MAVCommand.SET_MESSAGE_INTERVAL,
        MAVMessageType.SIM_STATE,
        -1,
    )
    assert ftp.removed == [PART_PATH, READY_PATH, *RESULT_PATHS]
    assert ftp.upload == (abin, PART_PATH)
    assert transferred == [0, len(abin) // 2, len(abin)]
    assert all(type(amount) is int for amount in transferred)
    assert ftp.closed is True


async def test_hardware_stage_setup_leaves_sim_truth_unchanged() -> None:
    uav = FakeUAV(1177)
    uav.driver = SimpleNamespace(send_command_long=AsyncMock(return_value=True))

    await make_backend(uav)._suppress_simulation_truth_stream()

    uav.driver.send_command_long.assert_not_awaited()


async def test_sitl_stage_fails_if_truth_stream_cannot_be_stopped() -> None:
    uav = FakeUAV(0)
    uav.driver = SimpleNamespace(send_command_long=AsyncMock(return_value=False))
    configuration = FirmwareUpdateConfiguration.from_json(
        {
            "provisioned_uav_ids": ["1"],
            "simulation_reported_board_id_overrides": {"0": 1177},
        }
    )
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)

    with pytest.raises(UpdateOperationError) as ex:
        await backend._suppress_simulation_truth_stream()

    assert ex.value.code == "simulationSetupFailed"


async def test_commit_atomically_renames_the_staged_image(monkeypatch) -> None:
    events = []

    class FTP:
        async def rename(self, source: str, destination: str) -> None:
            events.append(("rename", source, destination))

        async def aclose(self) -> None:
            events.append(("close",))

    ftp = FTP()
    uav = FakeUAV(1177)
    backend = make_backend(uav)

    def make_ftp(candidate, *, retry_policy):
        del retry_policy
        assert candidate is uav
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    await backend.commit()
    assert events == [
        ("rename", PART_PATH, READY_PATH),
        ("close",),
    ]


async def test_reboot_uses_the_uav_update_command_without_early_invalidation() -> None:
    uav = FakeUAV(1177)
    uav.reboot_after_update = AsyncMock()  # type: ignore[attr-defined]

    await make_backend(uav).reboot()

    uav.reboot_after_update.assert_awaited_once_with()  # type: ignore[attr-defined]


async def test_wait_for_disconnect_observes_connection_loss(monkeypatch) -> None:
    uav = FakeUAV(1177)
    delays = []

    async def disconnect(delay: float) -> None:
        delays.append(delay)
        uav.is_connected = False

    monkeypatch.setattr(trio, "sleep", disconnect)
    await make_backend(uav).wait_for_disconnect()
    assert delays == [0.2]


async def test_wait_for_reconnect_uses_uav_connection_primitive() -> None:
    uav = FakeUAV(1177, connected=False)
    uav.wait_until_connected = AsyncMock()  # type: ignore[attr-defined]

    await make_backend(uav).wait_for_reconnect()

    uav.wait_until_connected.assert_awaited_once_with()  # type: ignore[attr-defined]


async def test_installed_version_invalidates_cache_after_reconnect() -> None:
    events = []

    class VersionUAV(FakeUAV):
        def invalidate_version_info(self) -> None:
            events.append("invalidate")
            self._messages.pop(MAVMessageType.AUTOPILOT_VERSION, None)

        async def get_version_info(self) -> dict:
            events.append("request")
            self._messages[MAVMessageType.AUTOPILOT_VERSION] = SimpleNamespace(
                board_version=1177 << 16,
                flight_custom_version=b"89ABCDEF",
                flight_sw_version=4 << 24 | 7 << 16 | 0 << 8 | 255,
            )
            return {}

    installed = await make_backend(VersionUAV(1177)).read_installed()

    assert events == ["invalidate", "request"]
    assert installed.board_id == 1177
    assert installed.git_hash == "89abcdef"
    assert installed.version == "4.7.0"


def test_invalidate_version_info_clears_only_the_cached_version() -> None:
    heartbeat = object()
    uav = SimpleNamespace(
        _last_messages={
            MAVMessageType.HEARTBEAT: heartbeat,
            MAVMessageType.AUTOPILOT_VERSION: object(),
        }
    )
    MAVLinkUAV.invalidate_version_info(cast(MAVLinkUAV, uav))
    assert uav._last_messages == {MAVMessageType.HEARTBEAT: heartbeat}


async def test_installed_version_requires_a_fresh_version_message() -> None:
    class MissingVersionUAV(FakeUAV):
        def invalidate_version_info(self) -> None:
            self._messages.pop(MAVMessageType.AUTOPILOT_VERSION, None)

        async def get_version_info(self) -> dict:
            return {}

    with pytest.raises(RuntimeError) as raised:
        await make_backend(MissingVersionUAV(1177)).read_installed()
    assert str(raised.value) == "Autopilot version was not cached after requesting it"


async def test_installed_version_requires_a_fresh_board_identity() -> None:
    class BoardlessVersionUAV(FakeUAV):
        def invalidate_version_info(self) -> None:
            self._messages.pop(MAVMessageType.AUTOPILOT_VERSION, None)

        async def get_version_info(self) -> dict:
            self._messages[MAVMessageType.AUTOPILOT_VERSION] = SimpleNamespace(
                flight_custom_version=b"89ABCDEF",
                flight_sw_version=4 << 24 | 7 << 16 | 0 << 8 | 255,
            )
            return {}

    with pytest.raises(RuntimeError) as raised:
        await make_backend(BoardlessVersionUAV(1177)).read_installed()
    assert str(raised.value) == "Fresh autopilot version has no board identity"


async def test_installed_version_preserves_an_unknown_git_hash() -> None:
    class UnknownHashVersionUAV(FakeUAV):
        def invalidate_version_info(self) -> None:
            self._messages.pop(MAVMessageType.AUTOPILOT_VERSION, None)

        async def get_version_info(self) -> dict:
            self._messages[MAVMessageType.AUTOPILOT_VERSION] = SimpleNamespace(
                board_version=1177 << 16,
                flight_custom_version=b"\x00" * 8,
                flight_sw_version=4 << 24 | 7 << 16 | 0 << 8 | 255,
            )
            return {}

    installed = await make_backend(UnknownHashVersionUAV(1177)).read_installed()
    assert installed.git_hash == ""


async def test_marker_cleanup_ignores_only_missing_files() -> None:
    missing = SimpleNamespace(
        rm=AsyncMock(
            side_effect=OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_NOT_FOUND)
        )
    )
    await _remove_if_present(cast(MAVFTP, missing), PART_PATH)

    denied = SimpleNamespace(
        rm=AsyncMock(side_effect=OperationNotAcknowledgedError(MAVFTPErrorCode.FAIL))
    )
    with pytest.raises(OperationNotAcknowledgedError):
        await _remove_if_present(cast(MAVFTP, denied), PART_PATH)


class MarkerFTP:
    def __init__(self, entries: set[str], remove_error: Exception | None = None):
        self.entries = entries
        self.remove_error = remove_error
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
        if self.remove_error:
            raise self.remove_error

    async def aclose(self) -> None:
        self.closed = True


async def test_flash_result_accepts_success_marker_and_removes_it(monkeypatch) -> None:
    uav = FakeUAV(1177)
    ftp = MarkerFTP({"ardupilot-flashed.abin"})

    def make_ftp(candidate, *, retry_policy):
        del retry_policy
        assert candidate is uav
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    await ArduPilotUpdateBackend(cast(MAVLinkUAV, uav)).verify_flash_result()
    assert ftp.removed == ["/ardupilot-flashed.abin"]
    assert ftp.closed is True


async def test_flash_result_ignores_success_marker_cleanup_failure(monkeypatch) -> None:
    uav = FakeUAV(1177)
    ftp = MarkerFTP({"ardupilot-flashed.abin"}, OSError("read-only SD card"))
    warnings = []

    def make_ftp(candidate, *, retry_policy):
        del retry_policy
        assert candidate is uav
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    monkeypatch.setattr(
        log,
        "warning",
        lambda message, **kwargs: warnings.append((message, kwargs)),
    )
    await make_backend(uav).verify_flash_result()
    assert ftp.removed == ["/ardupilot-flashed.abin"]
    assert ftp.closed is True
    assert warnings == [
        (
            "Failed to remove ArduPilot OTA success marker",
            {"exc_info": True, "extra": {"id": "1"}},
        )
    ]


async def test_flash_result_rejects_explicit_failure_marker(monkeypatch) -> None:
    uav = FakeUAV(1177)
    ftp = MarkerFTP({"ardupilot-failed.abin"})

    def make_ftp(candidate, *, retry_policy):
        del retry_policy
        assert candidate is uav
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav))
    with pytest.raises(UpdateOperationError) as raised:
        await backend.verify_flash_result()
    assert raised.value.code == "imageRejected"
    assert ftp.closed is True


async def test_flash_result_times_out_as_indeterminate(monkeypatch) -> None:
    uav = FakeUAV(1177)
    ftp = MarkerFTP({"ardupilot-verify.abin"})

    def make_ftp(candidate, *, retry_policy):
        del retry_policy
        assert candidate is uav
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    delays = []

    async def wait_for_next_poll(delay: float) -> None:
        delays.append(delay)
        await trio.sleep_forever()

    monkeypatch.setattr(trio, "sleep", wait_for_next_poll)
    configuration = FirmwareUpdateConfiguration(result_timeout=0.1)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    with pytest.raises(UpdateResultIndeterminateError) as raised:
        await backend.verify_flash_result()
    assert raised.value.code == "verificationInterrupted"
    assert delays == [0.25]
