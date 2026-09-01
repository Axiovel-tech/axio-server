"""Transaction coordinator for one-at-a-time ArduPilot firmware updates."""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from typing import Collection, Protocol
from uuid import uuid4

import trio

from .apj import APJValidationError, FirmwareImage, parse_apj
from .backend import (
    InstalledFirmware,
    UpdateBackend,
    UpdateOperationError,
    UpdateResultIndeterminateError,
)
from .model import OTAError, OTAJob


class JobNotifier(Protocol):
    def __call__(self, job: OTAJob) -> Awaitable[None]: ...


class UpdateBusyError(RuntimeError):
    """Raised when another flight-controller update already runs."""


class CancellationRejectedError(RuntimeError):
    """Raised when a client tries to cancel after the commit boundary."""


class UserCancelled(RuntimeError):
    """Internal control-flow exception for a requested cancellation."""


class IndeterminateUpdate(RuntimeError):
    """Raised when the server cannot determine a committed update's result."""


class FirmwareUpdateCoordinator:
    """Owns update jobs and enforces a global concurrency limit of one."""

    def __init__(
        self,
        nursery: trio.Nursery,
        backend_factory: Callable[[str], UpdateBackend],
        notifier: JobNotifier,
        allowed_board_ids: Collection[int] = (1177,),
    ):
        self._nursery = nursery
        self._backend_factory = backend_factory
        self._notifier = notifier
        self._allowed_board_ids = frozenset(allowed_board_ids)
        self._jobs: dict[str, OTAJob] = {}
        self._active_operation_id: str | None = None
        self._precommit_cancel_scopes: dict[str, trio.CancelScope] = {}

    def get(self, uav_id: str) -> OTAJob | None:
        return self._jobs.get(uav_id)

    def get_by_operation_id(self, operation_id: str) -> OTAJob | None:
        return next(
            (job for job in self._jobs.values() if job.operation_id == operation_id),
            None,
        )

    def start(
        self,
        *,
        uav_id: str,
        name: str,
        payload: bytes,
        sha256: str,
    ) -> OTAJob:
        if self._active_operation_id is not None:
            raise UpdateBusyError("Another flight-controller update is already running")
        job = OTAJob(operation_id=uuid4().hex, uav_id=uav_id, name=name)
        self._jobs[uav_id] = job
        self._active_operation_id = job.operation_id
        self._nursery.start_soon(self._run, job, payload, sha256)
        return job

    def cancel(self, operation_id: str) -> OTAJob:
        job = self.get_by_operation_id(operation_id)
        if job is None:
            raise KeyError(operation_id)
        if job.status != "running" or not job.cancellable:
            raise CancellationRejectedError(
                "The update has passed its cancellation point"
            )
        job.cancel_requested.set()
        job.cancellable = False
        scope = self._precommit_cancel_scopes.get(operation_id)
        if scope is not None:
            scope.cancel()
        return job

    async def _run(self, job: OTAJob, payload: bytes, sha256: str) -> None:
        try:
            image = parse_apj(
                payload,
                expected_sha256=sha256,
                name=job.name,
                allowed_board_ids=self._allowed_board_ids,
            )
            job.expected.update(
                boardId=image.board_id,
                gitHash=image.git_hash,
                version=image.version,
            )
            job.total_bytes = image.total_size
            await self._notify(job)
            backend = self._backend_factory(job.uav_id)
            backend.check_safety(image.board_id)
            with trio.CancelScope() as scope:
                self._precommit_cancel_scopes[job.operation_id] = scope
                try:
                    self._raise_if_cancelled(job)
                    await self._stage(job, backend, image)
                    await self._verify_upload(job, backend, image)
                finally:
                    del self._precommit_cancel_scopes[job.operation_id]
            self._raise_if_cancelled(job)
            await self._commit(job, backend, image)
            await self._reboot_and_reconnect(job, backend)
            await self._verify_installed(job, backend, image)
            job.transferred_bytes = job.total_bytes
            job.finish("success")
        except UserCancelled:
            job.finish("cancelled")
        except IndeterminateUpdate as ex:
            job.finish("indeterminate", _error("indeterminate", str(ex)))
        except UpdateResultIndeterminateError as ex:
            job.finish("indeterminate", _error(ex.code, str(ex)))
        except (APJValidationError, UpdateOperationError) as ex:
            job.finish("failed", _error(ex.code, str(ex)))
        except trio.TooSlowError as ex:
            status = "indeterminate" if job.committed else "failed"
            job.finish(
                status, _error("timeout", str(ex) or "Firmware update timed out")
            )
        except Exception as ex:  # noqa: BLE001
            status = "indeterminate" if job.committed else "failed"
            job.finish(status, _error("internalError", str(ex)))
        finally:
            if self._active_operation_id == job.operation_id:
                self._active_operation_id = None
            await self._notify(job)

    async def _stage(
        self, job: OTAJob, backend: UpdateBackend, image: FirmwareImage
    ) -> None:
        job.set_phase("staging")
        await self._notify(job)
        async for transferred in backend.stage(image):
            self._raise_if_cancelled(job)
            if transferred != job.transferred_bytes:
                job.transferred_bytes = transferred
                await self._notify(job)

    async def _verify_upload(
        self, job: OTAJob, backend: UpdateBackend, image: FirmwareImage
    ) -> None:
        self._raise_if_cancelled(job)
        job.set_phase("verifyingUpload")
        await self._notify(job)
        await backend.verify_upload(image)
        self._raise_if_cancelled(job)
        backend.check_safety(image.board_id)

    async def _commit(
        self, job: OTAJob, backend: UpdateBackend, image: FirmwareImage
    ) -> None:
        self._raise_if_cancelled(job)
        job.enter_commit()
        await self._notify(job)
        await backend.commit(image.board_id)
        job.mark_committed()
        await self._notify(job)

    async def _reboot_and_reconnect(self, job: OTAJob, backend: UpdateBackend) -> None:
        job.set_phase("rebooting")
        await self._notify(job)
        reboot_error: Exception | None = None
        try:
            await backend.reboot()
        except Exception as ex:  # noqa: BLE001
            reboot_error = ex

        job.set_phase("reconnecting")
        await self._notify(job)
        try:
            await backend.wait_for_disconnect()
        except trio.TooSlowError:
            detail = "Reboot was not observed after the image was committed"
            if reboot_error:
                detail += f": {reboot_error}"
            raise IndeterminateUpdate(detail) from reboot_error
        await backend.wait_for_reconnect()

    async def _verify_installed(
        self, job: OTAJob, backend: UpdateBackend, image: FirmwareImage
    ) -> None:
        job.set_phase("verifyingInstalled")
        await self._notify(job)
        await backend.verify_flash_result()
        installed = await backend.read_installed()
        job.observed.update(
            boardId=installed.board_id,
            gitHash=installed.git_hash,
            version=installed.version,
        )
        await self._notify(job)
        _check_installed(image, installed)

    def _raise_if_cancelled(self, job: OTAJob) -> None:
        if job.cancel_requested.is_set():
            raise UserCancelled

    async def _notify(self, job: OTAJob) -> None:
        await self._notifier(job)


def _check_installed(image: FirmwareImage, installed: InstalledFirmware) -> None:
    if installed.board_id != image.board_id:
        raise UpdateOperationError(
            "installedBoardMismatch",
            f"Running board ID {installed.board_id} does not match {image.board_id}",
        )
    if installed.git_hash != image.git_hash:
        raise UpdateOperationError(
            "installedHashMismatch",
            f"Running git hash {installed.git_hash or 'unknown'} does not match {image.git_hash}",
        )


def _error(code: str, detail: str) -> OTAError:
    return {"code": code, "detail": detail}
