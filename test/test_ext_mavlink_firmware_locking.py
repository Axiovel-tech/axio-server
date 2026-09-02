"""Concurrency regressions for ArduPilot firmware MAVFTP sessions."""

from contextlib import asynccontextmanager
from types import SimpleNamespace
from typing import cast
from unittest.mock import AsyncMock

import pytest
import trio
from test_ext_mavlink_firmware_backend import FakeUAV, make_backend

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.enums import MAVMessageType
from flockwave.server.ext.mavlink.firmware.backend import (
    ArduPilotUpdateBackend,
    FirmwareUpdateConfiguration,
    UpdateOperationError,
    UpdateResultIndeterminateError,
)
from flockwave.server.ext.mavlink.ftp import MAVFTP


class CommitFTP:
    closed = False

    async def rename(self, _source: str, _destination: str) -> None:
        raise AssertionError("unsafe image was renamed")

    async def aclose(self) -> None:
        self.closed = True


class ResultFTP:
    removed: list[str]

    def __init__(
        self,
        *,
        block_listing: bool = False,
        listing_delay: float = 0,
        remove_delay: float = 0,
    ) -> None:
        self.removed = []
        self.block_listing = block_listing
        self.listing_delay = listing_delay
        self.remove_delay = remove_delay
        self.closed = False

    @asynccontextmanager
    async def ls(self, _path: str):
        async def entries():
            if self.block_listing:
                await trio.sleep_forever()
            await trio.sleep(self.listing_delay)
            yield SimpleNamespace(name="ardupilot-flashed.abin")

        yield entries()

    async def rm(self, path: str) -> None:
        await trio.sleep(self.remove_delay)
        self.removed.append(path)

    async def aclose(self) -> None:
        self.closed = True


async def _wait_until_lock_is_contended(lock: trio.Lock) -> None:
    with trio.fail_after(1):
        while lock.statistics().tasks_waiting != 1:
            await trio.lowlevel.checkpoint()


async def test_commit_rechecks_safety_after_waiting_for_mavftp_lock(
    monkeypatch,
) -> None:
    ftp = CommitFTP()
    uav = FakeUAV(1177)
    backend = make_backend(uav)
    monkeypatch.setattr(backend, "refresh_version_info", AsyncMock())
    monkeypatch.setattr(MAVFTP, "for_uav", lambda *_args, **_kwargs: ftp)
    committed = []
    errors: list[UpdateOperationError] = []
    finished = trio.Event()

    async def commit() -> None:
        try:
            await backend.commit(1177, lambda: committed.append(True))
        except UpdateOperationError as ex:
            errors.append(ex)
        finally:
            finished.set()

    await uav.mavftp_lock.acquire()
    async with trio.open_nursery() as nursery:
        nursery.start_soon(commit)
        await _wait_until_lock_is_contended(uav.mavftp_lock)
        uav._messages[MAVMessageType.HEARTBEAT].base_mode = 128
        uav.mavftp_lock.release()
        await finished.wait()
        nursery.cancel_scope.cancel()

    assert errors and errors[0].code == "armed"
    assert committed == []
    assert ftp.closed is True


async def test_flash_result_timeout_starts_after_mavftp_lock_acquisition(
    monkeypatch,
) -> None:
    uav = FakeUAV(1177)
    ftp = ResultFTP()
    configuration = FirmwareUpdateConfiguration(result_timeout=0.1)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    monkeypatch.setattr(MAVFTP, "for_uav", lambda *_args, **_kwargs: ftp)
    errors: list[Exception] = []
    finished = trio.Event()

    async def verify() -> None:
        try:
            await backend.verify_flash_result()
        except Exception as ex:  # noqa: BLE001
            errors.append(ex)
        finally:
            finished.set()

    await uav.mavftp_lock.acquire()
    async with trio.open_nursery() as nursery:
        nursery.start_soon(verify)
        await _wait_until_lock_is_contended(uav.mavftp_lock)
        await trio.sleep(0.15)
        assert not finished.is_set()
        uav.mavftp_lock.release()
        await finished.wait()
        nursery.cancel_scope.cancel()

    assert errors == []
    assert ftp.removed == ["/ardupilot-flashed.abin"]


async def test_flash_result_timeout_during_first_listing_is_indeterminate(
    monkeypatch,
) -> None:
    uav = FakeUAV(1177)
    ftp = ResultFTP(block_listing=True)
    configuration = FirmwareUpdateConfiguration(result_timeout=0.1)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    monkeypatch.setattr(MAVFTP, "for_uav", lambda *_args, **_kwargs: ftp)

    with pytest.raises(UpdateResultIndeterminateError) as raised:
        await backend.verify_flash_result()

    assert raised.value.code == "resultMissing"


async def test_success_marker_cleanup_finishes_outside_result_deadline(
    monkeypatch,
) -> None:
    uav = FakeUAV(1177)
    ftp = ResultFTP(listing_delay=0.08, remove_delay=0.05)
    configuration = FirmwareUpdateConfiguration(result_timeout=0.1)
    backend = ArduPilotUpdateBackend(cast(MAVLinkUAV, uav), configuration)
    monkeypatch.setattr(MAVFTP, "for_uav", lambda *_args, **_kwargs: ftp)

    await backend.verify_flash_result()

    assert ftp.removed == ["/ardupilot-flashed.abin"]
    assert ftp.closed is True
