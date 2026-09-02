"""Tests for the ArduPilot firmware update transaction coordinator."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import cast

import pytest
import trio
from test_ext_mavlink_firmware_apj import make_apj

from flockwave.server.ext.mavlink.firmware.backend import (
    ArduPilotUpdateBackend,
    CommitRejectedError,
    InstalledFirmware,
    UpdateOperationError,
    UpdateResultIndeterminateError,
)
from flockwave.server.ext.mavlink.firmware.transaction import (
    CancellationRejectedError,
    FirmwareUpdateCoordinator,
    UpdateBusyError,
)


@dataclass
class FakeBackend:
    """Deterministic backend with gates for cancellation and timeout tests."""

    git_hash: str = "0123abcd"
    calls: list[str] = field(default_factory=list)
    hold_initial_refresh: trio.Event | None = None
    hold_staging: trio.Event | None = None
    hold_reboot: trio.Event | None = None
    reboot_ack_lost: bool = False
    disconnect_timeout: bool = False
    marker_error: Exception | None = None
    installed_board_id: int = 1177
    stage_error: Exception | None = None
    commit_error: Exception | None = None
    final_safety_error: UpdateOperationError | None = None
    reconnect_timeout: bool = False

    def check_safety(self, board_id: int) -> None:
        assert board_id == 1177
        self.calls.append("safety")
        if self.final_safety_error is not None and "stage" in self.calls:
            raise self.final_safety_error

    async def refresh_version_info(self) -> None:
        self.calls.append("refresh")
        if self.hold_initial_refresh is not None and "stage" not in self.calls:
            await self.hold_initial_refresh.wait()

    async def stage(self, image):
        assert image.board_id == 1177
        self.calls.append("stage")
        if self.stage_error is not None:
            raise self.stage_error
        yield image.total_size // 2
        if self.hold_staging is not None:
            await self.hold_staging.wait()
        yield image.total_size

    async def commit(self, board_id: int, mark_committed) -> None:
        await self.refresh_version_info()
        self.check_safety(board_id)
        self.calls.append("commit")
        mark_committed()
        if self.commit_error is not None:
            raise self.commit_error

    async def reboot(self) -> None:
        self.calls.append("reboot")
        if self.hold_reboot is not None:
            await self.hold_reboot.wait()
        if self.reboot_ack_lost:
            raise TimeoutError("ACK lost")

    async def wait_for_disconnect(self) -> None:
        self.calls.append("disconnect")
        if self.disconnect_timeout:
            raise trio.TooSlowError

    async def wait_for_reconnect(self) -> None:
        self.calls.append("reconnect")
        if self.reconnect_timeout:
            raise trio.TooSlowError

    async def verify_flash_result(self) -> None:
        self.calls.append("marker")
        if self.marker_error is not None:
            raise self.marker_error

    async def read_installed(self) -> InstalledFirmware:
        self.calls.append("installed")
        return InstalledFirmware(self.installed_board_id, self.git_hash, "4.6.0")


async def start_job(
    nursery, backend: FakeBackend, notifications: list[dict], notify_hook=None
):
    payload = make_apj()

    async def notify(job) -> None:
        notifications.append(job.json())
        if notify_hook is not None:
            notify_hook(job)

    def make_backend(uav_id: str) -> ArduPilotUpdateBackend:
        assert uav_id == "1"
        return cast(ArduPilotUpdateBackend, backend)

    coordinator = FirmwareUpdateCoordinator(nursery, make_backend, notify)
    job = coordinator.start(
        uav_id="1",
        name="arducopter.apj",
        payload=payload,
        sha256=hashlib.sha256(payload).hexdigest(),
    )
    return coordinator, job


async def wait_finished(job) -> None:
    with trio.fail_after(2):
        while job.status == "running":
            await trio.sleep(0)


async def test_full_update_survives_lost_reboot_ack() -> None:
    backend = FakeBackend(reboot_ack_lost=True)
    notifications: list[dict] = []
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, notifications)
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "success"
    assert job.committed
    assert job.transferred_bytes == job.total_bytes == len(parse_abin())
    assert job.expected == {
        "gitHash": "0123abcd",
        "version": "4.6.1",
    }
    assert job.observed == {
        "gitHash": "0123abcd",
        "version": "4.6.0",
    }
    assert backend.calls == [
        "refresh",
        "safety",
        "stage",
        "refresh",
        "safety",
        "commit",
        "reboot",
        "disconnect",
        "reconnect",
        "marker",
        "installed",
    ]
    phases = [item["phase"] for item in notifications]
    assert phases[0] == "validating"
    assert phases[-1] == "complete"
    assert phases.index("staging") < phases.index("committing")
    assert phases.index("committing") < phases.index("rebooting")
    assert phases.index("rebooting") < phases.index("reconnecting")
    assert phases.index("reconnecting") < phases.index("verifyingInstalled")


async def test_cancel_during_staging_never_commits() -> None:
    backend = FakeBackend(hold_staging=trio.Event())
    async with trio.open_nursery() as nursery:
        coordinator, job = await start_job(nursery, backend, [])
        with trio.fail_after(2):
            while job.transferred_bytes is None:
                await trio.sleep(0)
        cancelled = coordinator.cancel(job.operation_id)
        assert cancelled is job
        assert job.cancel_requested.is_set()
        assert job.cancellable is False
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "cancelled"
    assert not job.committed
    assert "commit" not in backend.calls
    assert coordinator._precommit_cancel_scopes == {}


async def test_cancel_during_initial_version_refresh_finishes_cancelled() -> None:
    backend = FakeBackend(hold_initial_refresh=trio.Event())
    async with trio.open_nursery() as nursery:
        coordinator, job = await start_job(nursery, backend, [])
        with trio.fail_after(2):
            while backend.calls != ["refresh"]:
                await trio.sleep(0)
        coordinator.cancel(job.operation_id)
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "cancelled"
    assert job.committed is False
    assert backend.calls == ["refresh"]
    assert coordinator._precommit_cancel_scopes == {}


async def test_cancel_interrupts_the_initial_notification() -> None:
    backend = FakeBackend()
    notification_started = trio.Event()
    notifications: list[dict] = []

    async def notify(job) -> None:
        if not notification_started.is_set():
            notification_started.set()
            await trio.sleep_forever()
        notifications.append(job.json())

    def make_backend(_uav_id: str) -> ArduPilotUpdateBackend:
        return cast(ArduPilotUpdateBackend, backend)

    async with trio.open_nursery() as nursery:
        coordinator = FirmwareUpdateCoordinator(nursery, make_backend, notify)
        payload = make_apj()
        job = coordinator.start(
            uav_id="1",
            name="arducopter.apj",
            payload=payload,
            sha256=hashlib.sha256(payload).hexdigest(),
        )
        await notification_started.wait()
        coordinator.cancel(job.operation_id)
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "cancelled"
    assert backend.calls == []
    assert notifications[-1]["status"] == "cancelled"
    assert coordinator._precommit_cancel_scopes == {}


async def test_cancel_after_commit_is_rejected() -> None:
    release = trio.Event()
    backend = FakeBackend(hold_reboot=release)
    async with trio.open_nursery() as nursery:
        coordinator, job = await start_job(nursery, backend, [])
        with trio.fail_after(2):
            while not job.committed:
                await trio.sleep(0)
        with pytest.raises(CancellationRejectedError) as raised:
            coordinator.cancel(job.operation_id)
        assert str(raised.value) == "The update has passed its cancellation point"
        release.set()
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "success"


async def test_disconnect_timeout_after_commit_is_indeterminate() -> None:
    backend = FakeBackend(disconnect_timeout=True)
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "indeterminate"
    assert job.error == {
        "code": "indeterminate",
        "detail": "Reboot was not observed after the image was committed",
    }


async def test_reconnect_timeout_after_commit_is_indeterminate() -> None:
    backend = FakeBackend(reconnect_timeout=True)
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "indeterminate"
    assert job.committed is True
    assert job.error == {
        "code": "timeout",
        "detail": "Firmware update timed out",
    }
    assert "marker" not in backend.calls


async def test_timeout_before_commit_is_failed() -> None:
    backend = FakeBackend(stage_error=trio.TooSlowError())
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "failed"
    assert job.committed is False
    assert job.error == {
        "code": "timeout",
        "detail": "Firmware update timed out",
    }


async def test_stage_transport_loss_fails_before_commit_without_reboot() -> None:
    backend = FakeBackend(stage_error=TimeoutError("MAVFTP packet retry exhausted"))
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "failed"
    assert job.committed is False
    assert job.error == {
        "code": "internalError",
        "detail": "MAVFTP packet retry exhausted",
    }
    assert backend.calls == ["refresh", "safety", "stage"]


async def test_put_gen_crc_failure_prevents_commit_rename_and_reboot() -> None:
    backend = FakeBackend(stage_error=RuntimeError("CRC mismatch after MAVFTP upload"))
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "failed"
    assert job.committed is False
    assert job.error == {
        "code": "internalError",
        "detail": "CRC mismatch after MAVFTP upload",
    }
    assert backend.calls == ["refresh", "safety", "stage"]


async def test_lost_rename_ack_is_indeterminate_and_past_cancellation() -> None:
    backend = FakeBackend(commit_error=TimeoutError("rename ACK lost"))
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "indeterminate"
    assert job.committed is True
    assert job.cancellable is False
    assert job.error == {"code": "internalError", "detail": "rename ACK lost"}
    assert backend.calls == [
        "refresh",
        "safety",
        "stage",
        "refresh",
        "safety",
        "commit",
    ]


async def test_explicit_rename_rejection_fails_without_claiming_commit() -> None:
    backend = FakeBackend(
        commit_error=CommitRejectedError(
            "commitRejected", "Flight controller rejected the staged image"
        )
    )
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "failed"
    assert job.committed is False
    assert job.cancellable is False
    assert job.error == {
        "code": "commitRejected",
        "detail": "Flight controller rejected the staged image",
    }
    assert "reboot" not in backend.calls


async def test_final_safety_rejection_is_failed_before_commit() -> None:
    backend = FakeBackend()

    def arm_during_commit_notification(job) -> None:
        if job.phase == "committing":
            backend.final_safety_error = UpdateOperationError(
                "armed", "UAV became armed while its image was staged"
            )

    async with trio.open_nursery() as nursery:
        _, job = await start_job(
            nursery, backend, [], notify_hook=arm_during_commit_notification
        )
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "failed"
    assert job.committed is False
    assert job.cancellable is False
    assert job.error == {
        "code": "armed",
        "detail": "UAV became armed while its image was staged",
    }
    assert backend.calls == ["refresh", "safety", "stage", "refresh", "safety"]


async def test_marker_timeout_after_commit_is_indeterminate_without_retry() -> None:
    backend = FakeBackend(
        marker_error=UpdateResultIndeterminateError(
            "flashingInterrupted", "Bootloader flashing did not finish"
        )
    )
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "indeterminate"
    assert job.error == {
        "code": "flashingInterrupted",
        "detail": "Bootloader flashing did not finish",
    }
    assert backend.calls.count("marker") == 1
    assert "installed" not in backend.calls


async def test_explicit_bootloader_rejection_after_commit_is_failed() -> None:
    backend = FakeBackend(
        marker_error=UpdateOperationError(
            "imageRejected", "Bootloader rejected the update image"
        )
    )
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "failed"
    assert job.error and job.error["code"] == "imageRejected"


async def test_unexpected_error_after_commit_is_indeterminate() -> None:
    backend = FakeBackend(marker_error=RuntimeError("transport vanished"))
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.status == "indeterminate"
    assert job.error == {
        "code": "internalError",
        "detail": "transport vanished",
    }


async def test_global_concurrency_limit_cannot_be_bypassed() -> None:
    release = trio.Event()
    backend = FakeBackend(hold_staging=release)
    async with trio.open_nursery() as nursery:
        coordinator, first = await start_job(nursery, backend, [])
        with pytest.raises(UpdateBusyError) as raised:
            coordinator.start(
                uav_id="2",
                name="arducopter.apj",
                payload=make_apj(),
                sha256=hashlib.sha256(make_apj()).hexdigest(),
            )
        assert str(raised.value) == (
            "Another flight-controller update is already running"
        )
        coordinator.cancel(first.operation_id)
        release.set()
        await wait_finished(first)
        nursery.cancel_scope.cancel()


async def test_unknown_operation_cannot_be_cancelled() -> None:
    backend = FakeBackend()
    async with trio.open_nursery() as nursery:
        coordinator, job = await start_job(nursery, backend, [])
        with pytest.raises(KeyError) as raised:
            coordinator.cancel("not-the-operation")
        assert raised.value.args == ("not-the-operation",)
        await wait_finished(job)
        nursery.cancel_scope.cancel()


@pytest.mark.parametrize(
    ("backend", "code", "detail"),
    [
        (
            FakeBackend(git_hash="89abcdef"),
            "installedHashMismatch",
            "Running git hash 89abcdef does not match 0123abcd",
        ),
        (
            FakeBackend(git_hash=""),
            "installedHashMismatch",
            "Running git hash unknown does not match 0123abcd",
        ),
        (
            FakeBackend(installed_board_id=42),
            "installedBoardMismatch",
            "Running board ID 42 does not match 1177",
        ),
    ],
)
async def test_observed_installed_identity_mismatch_is_failed(
    backend: FakeBackend, code: str, detail: str
) -> None:
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "failed"
    assert job.error and job.error["code"] == code
    assert job.error["detail"] == detail


async def test_lost_reboot_ack_is_included_when_disconnect_is_not_observed() -> None:
    backend = FakeBackend(reboot_ack_lost=True, disconnect_timeout=True)
    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        await wait_finished(job)
        nursery.cancel_scope.cancel()
    assert job.error == {
        "code": "indeterminate",
        "detail": ("Reboot was not observed after the image was committed: ACK lost"),
    }


async def test_completed_job_releases_global_lock() -> None:
    backend = FakeBackend()
    async with trio.open_nursery() as nursery:
        coordinator, first = await start_job(nursery, backend, [])
        await wait_finished(first)
        second = coordinator.start(
            uav_id="1",
            name="arducopter.apj",
            payload=make_apj(),
            sha256=hashlib.sha256(make_apj()).hexdigest(),
        )
        await wait_finished(second)
        nursery.cancel_scope.cancel()
    assert first.status == second.status == "success"


def parse_abin() -> bytes:
    from flockwave.server.ext.mavlink.firmware.apj import parse_apj

    payload = make_apj()
    return parse_apj(
        payload,
        expected_sha256=hashlib.sha256(payload).hexdigest(),
        name="arducopter.apj",
    ).abin
