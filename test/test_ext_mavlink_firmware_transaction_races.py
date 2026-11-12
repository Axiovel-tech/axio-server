"""Race regressions for the firmware update transaction coordinator."""

import hashlib
from typing import cast

import pytest
import trio
from test_ext_mavlink_firmware_apj import make_apj
from test_ext_mavlink_firmware_transaction import (
    FakeBackend,
    start_job,
    wait_finished,
)

from flockwave.server.ext.mavlink.firmware.backend import ArduPilotUpdateBackend
from flockwave.server.ext.mavlink.firmware.transaction import FirmwareUpdateCoordinator


@pytest.mark.parametrize("wait_point", ["ftpLock", "versionRefresh"])
async def test_cancel_remains_interruptible_until_rename(wait_point: str) -> None:
    gate = trio.Event()
    backend = FakeBackend(
        hold_commit_lock=gate if wait_point == "ftpLock" else None,
        hold_final_refresh=gate if wait_point == "versionRefresh" else None,
    )

    async with trio.open_nursery() as nursery:
        coordinator, job = await start_job(nursery, backend, [])
        with trio.fail_after(2):
            while not _reached_wait_point(backend, wait_point):
                await trio.lowlevel.checkpoint()

        assert job.phase == "committing"
        assert job.cancellable is True
        coordinator.cancel(job.operation_id)
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "cancelled"
    assert job.committed is False
    assert "commit" not in backend.calls
    assert coordinator._precommit_cancel_scopes == {}


def _reached_wait_point(backend: FakeBackend, wait_point: str) -> bool:
    if wait_point == "ftpLock":
        return "commitLock" in backend.calls
    return backend.calls.count("refresh") == 2


class FastRebootCycleBackend(FakeBackend):
    """Completes a reboot cycle while the reboot command still awaits its ACK."""

    def __init__(self) -> None:
        super().__init__()
        self.observer_waiting = trio.Event()
        self.disconnected = trio.Event()
        self.cycle_complete = trio.Event()
        self.release_reboot_ack = trio.Event()

    async def wait_for_disconnect(self) -> None:
        self.calls.append("disconnect")
        self.observer_waiting.set()
        await self.disconnected.wait()

    async def reboot(self) -> None:
        self.calls.append("reboot")
        await self.observer_waiting.wait()
        self.disconnected.set()
        await trio.lowlevel.checkpoint()
        self.cycle_complete.set()
        await self.release_reboot_ack.wait()

    async def wait_for_reconnect(self) -> None:
        self.calls.append("reconnect")
        assert self.cycle_complete.is_set()


async def test_observes_reboot_cycle_while_reboot_command_is_awaiting() -> None:
    backend = FastRebootCycleBackend()

    async with trio.open_nursery() as nursery:
        _, job = await start_job(nursery, backend, [])
        with trio.fail_after(2):
            await backend.cycle_complete.wait()

        assert job.status == "running"
        assert job.phase == "rebooting"
        assert "reconnect" not in backend.calls
        backend.release_reboot_ack.set()
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "success"
    assert backend.calls.index("disconnect") < backend.calls.index("reboot")
    assert backend.calls.index("reboot") < backend.calls.index("reconnect")


async def test_saturated_notifier_cannot_delay_committed_reboot() -> None:
    release_reboot = trio.Event()
    backend = FakeBackend(hold_reboot=release_reboot)
    blocked_phases: list[str] = []

    async def saturated_notifier(job) -> None:
        if job.committed:
            blocked_phases.append(job.phase)
            await trio.sleep_forever()

    def make_backend(_uav_id: str) -> ArduPilotUpdateBackend:
        return cast(ArduPilotUpdateBackend, backend)

    async with trio.open_nursery() as nursery:
        coordinator = FirmwareUpdateCoordinator(
            nursery, make_backend, saturated_notifier
        )
        payload = make_apj()
        job = coordinator.start(
            uav_id="1",
            name="arducopter.apj",
            payload=payload,
            sha256=hashlib.sha256(payload).hexdigest(),
        )
        with trio.fail_after(1):
            while "reboot" not in backend.calls:
                await trio.lowlevel.checkpoint()

        assert blocked_phases == ["rebooting"]
        release_reboot.set()
        await wait_finished(job)
        nursery.cancel_scope.cancel()

    assert job.status == "success"
    assert blocked_phases == [
        "rebooting",
        "reconnecting",
        "verifyingInstalled",
        "verifyingInstalled",
        "complete",
    ]
