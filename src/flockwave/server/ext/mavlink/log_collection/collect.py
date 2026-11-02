"""Safe collection of flight-controller logs: guard, close, fetch, restore."""

from __future__ import annotations

import fcntl
import json
import os
import pwd
import re
from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
from dataclasses import asdict, dataclass, field
from pathlib import Path

import trio

from flockwave.server.errors import FlockwaveError

from .download import VerificationError, fetch_file, write_json
from .fc import ARMED, heartbeat, read_parameter, write_parameter
from .ftp_client import FTPClient, FTPError, ListingEntry
from .status import Status, now_utc, provenance
from .transport import FCTransport

__all__ = ("CollectRequest", "Refused", "collect", "exclusive")

LOG_NAME = re.compile(r"^\d{8}\.BIN$", re.IGNORECASE)

_UNKNOWN_VOLTAGE = 0xFFFF
_RESTORE_TIMEOUT = 20
_CLOSE_TIMEOUT = 60
_STABLE_LISTINGS = 2

_FAILURES = (
    FTPError,
    VerificationError,
    trio.TooSlowError,
    OSError,
    RuntimeError,
    ValueError,
)


class Refused(FlockwaveError):
    """A guard failed before the flight controller was changed."""


@dataclass(frozen=True)
class CollectRequest:
    directory: Path
    """Output directory of this vehicle."""

    log_dir: str = "/APM/LOGS"
    latest: int = 1
    files: tuple[str, ...] = ()
    min_battery_voltage: float | None = None

    def __post_init__(self) -> None:
        if not 1 <= self.latest <= 100:
            raise ValueError("latest must be between 1 and 100")
        if not all(LOG_NAME.match(name) for name in self.files):
            raise ValueError("file names must look like 00000042.BIN")
        if not self.log_dir or len(self.log_dir) > 200:
            raise ValueError("invalid log directory")

    def json(self) -> dict[str, object]:
        return {**asdict(self), "directory": str(self.directory)}


def _lock(path: Path, message: str, stack: ExitStack) -> None:
    lock = stack.enter_context(path.open("w"))
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise Refused(message) from None


LOCK_DIRECTORY = Path(pwd.getpwuid(os.getuid()).pw_dir) / ".cache" / "axio-fc-logs"
"""Where the per-system locks live: the server and the direct-link CLI must
find the same file whatever their environment (XDG_RUNTIME_DIR, TMPDIR)."""


def recovery_state(system_id: int) -> Path:
    """Where a run keeps the original LOG_DISARMED of a flight controller
    until it is restored: next to the system's lock, so that the next run
    finds it whichever output directory or link it uses."""
    return LOCK_DIRECTORY / f"sysid-{system_id}.state.json"


@contextmanager
def exclusive(directory: Path, system_id: int) -> Iterator[None]:
    """One collection per output directory and per system ID on this host,
    across processes, so two runs never interleave LOG_DISARMED changes.
    """
    directory.mkdir(parents=True, exist_ok=True)
    LOCK_DIRECTORY.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        _lock(
            LOCK_DIRECTORY / f"sysid-{system_id}.lock",
            f"another collection is running for system {system_id}",
            stack,
        )
        _lock(
            directory / ".collect.lock",
            "another collection is writing to this directory",
            stack,
        )
        yield


async def check_show_start(transport: FCTransport) -> dict[str, float]:
    auth, _ = await read_parameter(transport, "SHOW_START_AUTH")
    start, _ = await read_parameter(transport, "SHOW_START_TIME")
    if auth != 0:
        raise Refused(f"a show start is authorized (SHOW_START_AUTH={auth:g})")
    if start >= 0:
        raise Refused(f"a show start is scheduled (SHOW_START_TIME={start:g})")
    return {"SHOW_START_AUTH": auth, "SHOW_START_TIME": start}


async def check_guards(
    transport: FCTransport, request: CollectRequest
) -> dict[str, object]:
    beat = await heartbeat(transport)
    if beat.base_mode & ARMED:
        raise Refused("the flight controller is armed")
    result: dict[str, object] = {
        "system_id": transport.system_id,
        "armed": False,
        **await check_show_start(transport),
    }
    minimum = request.min_battery_voltage
    if minimum is not None:
        voltage = None
        with transport.subscribe("SYS_STATUS") as health:
            with trio.move_on_after(5):
                reported = (await health.receive()).voltage_battery
                if reported != _UNKNOWN_VOLTAGE:
                    voltage = reported / 1000
        if voltage is None or voltage < minimum:
            raise Refused(
                f"battery {'unreported' if voltage is None else f'{voltage} V'} "
                f"is below {minimum} V"
            )
        result["battery_v"] = voltage
    return result


class LoggingSwitch:
    """Turns LOG_DISARMED off so the current log stops growing.

    The original value is persisted before the change, so a run that dies
    before restoring hands the true original to the next run.
    """

    NAME = "LOG_DISARMED"

    def __init__(self, transport: FCTransport, state: Path):
        self._transport = transport
        self._state = state
        self._original: float | None = None
        self._type = 0

    async def disable(self) -> dict[str, object]:
        pending = json.loads(self._state.read_text()) if self._state.exists() else {}
        current, self._type = await read_parameter(self._transport, self.NAME)
        original = pending.get("log_disarmed_original", current)
        write_json(self._state, {"log_disarmed_original": original, "since": now_utc()})
        self._original = original
        if current != 0:
            await write_parameter(self._transport, self.NAME, 0, self._type)
        return {
            "original": original,
            "found": current,
            "recovered_from_interrupted_run": "log_disarmed_original" in pending,
        }

    async def restore(self) -> dict[str, object]:
        if self._original is None:
            return {}
        await write_parameter(self._transport, self.NAME, self._original, self._type)
        self._state.unlink(missing_ok=True)
        return {"restored": self._original, "restore_verified": True}


async def wait_until_closed(ftp: FTPClient, log_dir: str) -> None:
    """Returns once the log listing has stayed the same for two seconds.

    ArduPilot keeps logging for a while after a disarm and then flushes its
    write buffer, so a single unchanged listing is not enough.
    """
    unchanged, previous = 0, None
    with trio.fail_after(_CLOSE_TIMEOUT):
        while unchanged < _STABLE_LISTINGS:
            sizes = {entry.name: entry.size for entry in await ftp.listdir(log_dir)}
            unchanged = unchanged + 1 if sizes == previous else 0
            previous = sizes
            if unchanged < _STABLE_LISTINGS:
                await trio.sleep(1)


async def newest_log_number(ftp: FTPClient, log_dir: str) -> int | None:
    """ArduPilot's LASTLOG.TXT; log numbers wrap, so the highest is not the newest."""
    try:
        await ftp.open_read(f"{log_dir.rstrip('/')}/LASTLOG.TXT")
        try:
            text = (await ftp.read(0, 16)).strip()
        finally:
            with trio.CancelScope(shield=True), trio.move_on_after(5):
                await ftp.terminate()
    except (FTPError, trio.TooSlowError):
        return None
    return int(text) if text.isdigit() else None


def newest_first(entries: list[ListingEntry], newest: int | None) -> list[ListingEntry]:
    def key(entry: ListingEntry) -> tuple[bool, int]:
        number = int(entry.name[:8])
        return (newest is None or number <= newest, number)

    return sorted(entries, key=key, reverse=True)


async def select_logs(ftp: FTPClient, request: CollectRequest) -> list[ListingEntry]:
    logs = [
        entry
        for entry in await ftp.listdir(request.log_dir)
        if entry.is_file and LOG_NAME.match(entry.name)
    ]
    if request.files:
        by_name = {entry.name.upper(): entry for entry in logs}
        missing = [name for name in request.files if name.upper() not in by_name]
        if missing:
            raise Refused(f"not on the flight controller: {', '.join(missing)}")
        return [by_name[name.upper()] for name in request.files]
    if not logs:
        raise Refused(f"no logs in {request.log_dir}")
    newest = await newest_log_number(ftp, request.log_dir)
    return newest_first(logs, newest)[: request.latest]


async def _watch_armed(
    transport: FCTransport, stop: list[str], scope: trio.CancelScope
):
    with transport.subscribe("HEARTBEAT") as beats:
        async for beat in beats:
            if beat.base_mode & ARMED:
                stop.append("the flight controller is armed")
                scope.cancel()
                return


async def _watch_battery(
    transport: FCTransport,
    status: Status,
    minimum: float | None,
    stop: list[str],
    scope: trio.CancelScope,
):
    with transport.subscribe("SYS_STATUS") as health:
        async for message in health:
            if message.voltage_battery == _UNKNOWN_VOLTAGE:
                continue
            status.battery_voltage = message.voltage_battery / 1000
            if minimum is not None and status.battery_voltage < minimum:
                stop.append(f"battery {status.battery_voltage} V is below {minimum} V")
                scope.cancel()
                return


async def transfer(
    transport: FCTransport,
    ftp: FTPClient,
    request: CollectRequest,
    selected: list[ListingEntry],
    status: Status,
    results: list[dict[str, object]],
) -> str | None:
    """Fetches the selected files in order; returns why it stopped early."""
    for entry in selected:
        try:
            await check_show_start(transport)
            receipt = await fetch_file(
                ftp,
                f"{request.log_dir.rstrip('/')}/{entry.name}",
                request.directory / entry.name,
                status,
            )
        except (Refused, *_FAILURES) as error:
            results.append(
                {"file": entry.name, "status": "failed", "error": str(error)}
            )
            return str(error)
        results.append({**receipt, "status": "verified"})
        status.end_file(entry.size)
    return None


def _size(path: Path) -> int | None:
    return path.stat().st_size if path.is_file() else None


def write_manifest(directory: Path, status: Status, system_id: int) -> None:
    receipts = [
        json.loads(path.read_text()) for path in sorted(directory.glob("*.BIN.json"))
    ]
    write_json(
        directory / "manifest.json",
        {
            "uav": status.uav,
            "system_id": system_id,
            "updated_at": now_utc(),
            "files": [
                receipt
                for receipt in receipts
                if _size(directory / str(receipt.get("file"))) == receipt.get("bytes")
            ],
        },
    )


@dataclass
class RunRecord:
    """The parts of a run receipt that the run fills in as it goes."""

    guards: dict[str, object] = field(default_factory=dict)
    log_disarmed: dict[str, object] = field(default_factory=dict)
    selected: list[dict[str, object]] = field(default_factory=list)
    files: list[dict[str, object]] = field(default_factory=list)
    error: str | None = None

    @property
    def complete(self) -> bool:
        return bool(
            self.error is None
            and self.selected
            and len(self.files) == len(self.selected)
            and all(item.get("status") == "verified" for item in self.files)
            and self.log_disarmed.get("restore_verified")
        )


async def _restore(switch: LoggingSwitch, record: RunRecord) -> None:
    with trio.CancelScope(shield=True):
        with trio.move_on_after(_RESTORE_TIMEOUT) as deadline:
            try:
                record.log_disarmed.update(await switch.restore())
            except _FAILURES as error:
                record.log_disarmed["restore_error"] = str(error)
        if deadline.cancelled_caught:
            record.log_disarmed["restore_error"] = "LOG_DISARMED restore timed out"


async def _run(
    transport: FCTransport,
    request: CollectRequest,
    status: Status,
    record: RunRecord,
    switch: LoggingSwitch,
) -> None:
    record.guards = await check_guards(transport, request)
    status.phase = "closing"
    record.log_disarmed = await switch.disable()
    with transport.subscribe("FILE_TRANSFER_PROTOCOL") as replies:
        ftp = FTPClient(transport, replies)
        await wait_until_closed(ftp, request.log_dir)
        status.phase = "listing"
        selected = await select_logs(ftp, request)
        record.selected = [{"file": e.name, "bytes": e.size} for e in selected]
        status.files_total = len(selected)
        status.bytes_total = sum(entry.size for entry in selected)
        record.error = await transfer(
            transport, ftp, request, selected, status, record.files
        )


async def collect(
    transport: FCTransport, request: CollectRequest, status: Status
) -> dict[str, object]:
    """Runs one vehicle's collection and returns its run receipt.

    Nothing on the flight controller changes until the guards pass, and
    LOG_DISARMED is restored even when the run fails or is cancelled. The
    caller holds the vehicle directory's `exclusive` lock.
    """
    started_at = now_utc()
    header: dict[str, object] = {
        "uav": status.uav,
        "system_id": transport.system_id,
        "transport": transport.description,
        "request": request.json(),
        "started_at": started_at,
        "provenance": await trio.to_thread.run_sync(provenance),
    }
    record = RunRecord()
    switch = LoggingSwitch(transport, recovery_state(transport.system_id))
    path = request.directory / f"run-{started_at}.json".replace(":", "")
    stop: list[str] = []
    try:
        async with trio.open_nursery() as nursery:
            scope = nursery.cancel_scope
            nursery.start_soon(_watch_armed, transport, stop, scope)
            nursery.start_soon(
                _watch_battery,
                transport,
                status,
                request.min_battery_voltage,
                stop,
                scope,
            )
            try:
                await _run(transport, request, status, record, switch)
            except (Refused, *_FAILURES) as error:
                record.error = str(error)
            scope.cancel()
        if stop:
            record.error = stop[0]
    finally:
        status.phase = "restoring"
        await _restore(switch, record)
        write_manifest(request.directory, status, transport.system_id)
        status.error = record.error
        status.phase = "done" if record.complete else "failed"
        receipt: dict[str, object] = {
            **header,
            **asdict(record),
            "complete": record.complete,
            "finished_at": now_utc(),
        }
        write_json(path, receipt)
    receipt["receipt_path"] = str(path)
    return receipt
