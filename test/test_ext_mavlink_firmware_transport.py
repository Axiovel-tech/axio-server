"""Direct tests for the ArduPilot OTA MAVFTP and reboot transport."""

from contextlib import asynccontextmanager
from types import SimpleNamespace
from typing import cast
from unittest.mock import AsyncMock

import pytest
import trio
from test_ext_mavlink_firmware_backend import FakeUAV, make_backend

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.enums import MAVMessageType, MAVState
from flockwave.server.ext.mavlink.firmware.apj import FirmwareImage
from flockwave.server.ext.mavlink.firmware.backend import (
    PART_PATH,
    READY_PATH,
    RESULT_PATHS,
    ArduPilotUpdateBackend,
    CommitRejectedError,
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


async def test_sitl_stage_uses_crc_upload(monkeypatch) -> None:
    abin = b"x" * 200

    class FTP:
        def __init__(self):
            self.removed = []
            self.upload = None
            self.closed = False

        async def rm(self, path: str) -> None:
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

    def make_ftp(candidate, *, retry_policy):
        assert candidate is uav
        assert retry_policy.base_timeout == 1
        assert retry_policy.max_retries == 20
        assert retry_policy.max_timeout == 3
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

    assert ftp.removed == [PART_PATH, READY_PATH, *RESULT_PATHS]
    assert ftp.upload == (abin, PART_PATH)
    assert transferred == [0, len(abin) // 2, len(abin)]
    assert all(type(amount) is int for amount in transferred)
    assert ftp.closed is True


async def test_stage_shields_session_reset_from_user_cancellation(monkeypatch) -> None:
    remote = SimpleNamespace(open=False, attempts=0, resets=0)

    class FTP:
        closed = False

        async def rm(self, _path: str) -> None:
            pass

        @asynccontextmanager
        async def put_gen(self, _data: bytes, _path: str):
            if remote.open:
                raise RuntimeError("previous MAVFTP session remains open")
            remote.open = True
            remote.attempts += 1

            async def progress():
                if remote.attempts == 1:
                    yield SimpleNamespace(percentage=0)
                    await trio.sleep_forever()
                yield SimpleNamespace(percentage=100)

            yield progress()

        async def aclose(self) -> None:
            await trio.lowlevel.checkpoint()
            remote.open = False
            remote.resets += 1
            self.closed = True

    ftps = []
    uav = FakeUAV(1177)

    def make_ftp(*_args, **_kwargs):
        ftp = FTP()
        ftps.append(ftp)
        return ftp

    monkeypatch.setattr(MAVFTP, "for_uav", make_ftp)
    image = cast(FirmwareImage, SimpleNamespace(abin=b"x", total_size=1))
    started = trio.Event()
    finished = trio.Event()
    stage_scope: trio.CancelScope | None = None

    async def consume_stage() -> None:
        nonlocal stage_scope
        with trio.CancelScope() as scope:
            stage_scope = scope
            try:
                async for _transferred in make_backend(uav).stage(image):
                    started.set()
            finally:
                finished.set()

    async with trio.open_nursery() as nursery:
        nursery.start_soon(consume_stage)
        await started.wait()
        assert stage_scope is not None
        stage_scope.cancel()
        await finished.wait()
        nursery.cancel_scope.cancel()

    assert ftps[0].closed is True
    assert [amount async for amount in make_backend(uav).stage(image)] == [1]
    assert remote.attempts == remote.resets == 2


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
    monkeypatch.setattr(backend, "refresh_version_info", AsyncMock())

    await backend.commit(1177, lambda: events.append(("committed",)))
    assert events == [
        ("committed",),
        ("rename", PART_PATH, READY_PATH),
        ("close",),
    ]


async def test_commit_translates_an_explicit_rename_rejection(monkeypatch) -> None:
    class FTP:
        async def rename(self, _source: str, _destination: str) -> None:
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_PROTECTED)

        async def aclose(self) -> None:
            pass

    monkeypatch.setattr(MAVFTP, "for_uav", lambda *_args, **_kwargs: FTP())

    with pytest.raises(CommitRejectedError) as raised:
        backend = make_backend(FakeUAV(1177))
        monkeypatch.setattr(backend, "refresh_version_info", AsyncMock())
        await backend.commit(1177, lambda: None)

    assert raised.value.code == "commitRejected"
    assert str(raised.value) == (
        "Flight controller rejected the staged image: "
        "File or directory is write protected"
    )


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
        await trio.lowlevel.checkpoint()

    monkeypatch.setattr(trio, "sleep", disconnect)
    configuration = FirmwareUpdateConfiguration(disconnect_timeout=0.05)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    await backend.wait_for_disconnect()
    assert delays == [0.2]


async def test_wait_for_disconnect_accepts_proxy_boot_heartbeat() -> None:
    uav = FakeUAV(1177)
    uav._messages[MAVMessageType.HEARTBEAT].system_status = MAVState.BOOT.value

    configuration = FirmwareUpdateConfiguration(disconnect_timeout=0.05)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    await backend.wait_for_disconnect()

    assert uav.is_connected is True


async def test_wait_for_reconnect_requires_a_new_communicable_heartbeat(
    monkeypatch,
) -> None:
    uav = FakeUAV(1177)
    previous = uav._messages[MAVMessageType.HEARTBEAT]
    previous.system_status = MAVState.BOOT.value
    boot = SimpleNamespace(system_status=MAVState.BOOT.value)
    replacement = SimpleNamespace(system_status=MAVState.STANDBY.value)
    delays = []

    async def receive_heartbeat(delay: float) -> None:
        delays.append(delay)
        if len(delays) == 1:
            previous.system_status = MAVState.STANDBY.value
        elif len(delays) == 2:
            uav._messages[MAVMessageType.HEARTBEAT] = boot
        elif len(delays) == 3:
            uav._messages[MAVMessageType.HEARTBEAT] = replacement
        else:
            raise AssertionError("reconnect did not accept a fresh heartbeat")
        await trio.lowlevel.checkpoint()

    monkeypatch.setattr(trio, "sleep", receive_heartbeat)
    await make_backend(uav).wait_for_reconnect()

    assert delays == [0.2, 0.2, 0.2]
    assert uav.get_last_message(MAVMessageType.HEARTBEAT) is replacement


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
