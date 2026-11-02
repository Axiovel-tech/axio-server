"""Resumable download of one flight-controller file with whole-file checks."""

from __future__ import annotations

import hashlib
import json
import os
import zlib
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic
from typing import Protocol

import trio

from .ftp_client import CHUNK_SIZE, FTPClient

__all__ = (
    "Coverage",
    "FileProgress",
    "PartialFile",
    "VerificationError",
    "digest",
    "fetch_file",
    "mavftp_crc32",
    "write_json",
)

_STALLED_PASSES = 8
"""Consecutive passes without new bytes before a transfer counts as stalled."""

_CRC_BYTES_PER_SECOND = 128 * 1024
"""Conservative SD-card rate for the flight controller's whole-file CRC."""

_SMALL_GAP = 4 * CHUNK_SIZE


class VerificationError(RuntimeError):
    pass


class FileProgress(Protocol):
    def begin_file(self, name: str, covered: int) -> None: ...

    def received(self, covered: int) -> None: ...

    def verifying(self) -> None: ...


def mavftp_crc32(data: bytes, crc: int = 0) -> int:
    """CRC32 as ArduPilot's MAVFTP computes it: zero seed, no final inversion."""
    return zlib.crc32(data, crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF


def digest(path: Path) -> tuple[int, str]:
    """MAVFTP CRC32 and SHA-256 of a local file, in one pass."""
    crc = 0
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1 << 20):
            crc = mavftp_crc32(chunk, crc)
            sha.update(chunk)
    return crc, sha.hexdigest()


def write_json(path: Path, data: object) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    os.replace(temporary, path)


class Coverage:
    """Sorted, merged, half-open byte ranges received so far."""

    def __init__(self, spans: list[list[int]] | None = None):
        self.spans: list[list[int]] = []
        for start, end in spans or []:
            self.add(start, end)

    def add(self, start: int, end: int) -> None:
        if end <= start:
            return
        merged: list[list[int]] = []
        for span in self.spans:
            if span[1] < start or span[0] > end:
                merged.append(span)
            else:
                start, end = min(start, span[0]), max(end, span[1])
        merged.append([start, end])
        self.spans = sorted(merged)

    @property
    def covered(self) -> int:
        return sum(end - start for start, end in self.spans)

    def gaps(self, size: int) -> list[tuple[int, int]]:
        result = []
        position = 0
        for start, end in self.spans:
            if start > position:
                result.append((position, min(start, size)))
            position = max(position, end)
        if position < size:
            result.append((position, size))
        return result


class PartialFile:
    """Bytes in ``<name>.part``; received ranges in ``<name>.part.json``.

    The range record is written only after the data is synced, so a crash can
    lose received bytes but never mark unreceived bytes as present.
    """

    def __init__(self, path: Path, size: int):
        self.path = path
        self.size = size
        self._record = path.with_name(path.name + ".json")
        self.coverage = Coverage()
        self.resumed_bytes = 0
        if self.path.exists() and self._record.exists():
            record = json.loads(self._record.read_text())
            if record.get("size") == size:
                self.coverage = Coverage(record.get("ranges"))
                self.resumed_bytes = self.coverage.covered
        self._fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        if not self.resumed_bytes:
            os.ftruncate(self._fd, 0)
        os.ftruncate(self._fd, size)

    def write(self, offset: int, data: bytes) -> None:
        if offset < 0 or offset + len(data) > self.size:
            raise VerificationError(
                f"{self.path.stem} grew past its {self.size} bytes while being "
                "read; it is still being written"
            )
        os.pwrite(self._fd, data, offset)
        self.coverage.add(offset, offset + len(data))

    def checkpoint(self) -> None:
        os.fsync(self._fd)
        write_json(self._record, {"size": self.size, "ranges": self.coverage.spans})

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def discard(self) -> None:
        self.close()
        self.path.unlink(missing_ok=True)
        self._record.unlink(missing_ok=True)

    def publish(self, target: Path) -> None:
        self.close()
        os.replace(self.path, target)
        self._record.unlink(missing_ok=True)


def reuse_verified(local: Path, remote_size: int) -> dict[str, object] | None:
    """Receipt of an existing verified copy, or None when there is none.

    Never overwrites a local file: an unverified or changed copy is an error.
    """
    receipt_path = local.with_name(local.name + ".json")
    if not local.exists():
        return None
    if not receipt_path.exists():
        raise VerificationError(f"{local.name} exists without a receipt")
    receipt = json.loads(receipt_path.read_text())
    if receipt.get("bytes") != remote_size:
        raise VerificationError(
            f"{local.name} changed on the flight controller since it was "
            f"verified ({receipt.get('bytes')} -> {remote_size} bytes)"
        )
    _, sha256 = digest(local)
    if sha256 != receipt.get("sha256"):
        raise VerificationError(f"{local.name} no longer matches its receipt")
    return {**receipt, "reused": True}


async def _fill(ftp: FTPClient, part: PartialFile, progress: FileProgress) -> None:
    stalls = 0
    while gaps := part.coverage.gaps(part.size):
        before = part.coverage.covered
        start, end = gaps[0]
        if end - start <= _SMALL_GAP:
            for offset in range(start, end, CHUNK_SIZE):
                data = await ftp.read(offset, min(CHUNK_SIZE, end - offset))
                if not data:
                    raise VerificationError(f"file ended early at byte {offset}")
                part.write(offset, data)
        else:
            await ftp.burst(start, part.write)
        await trio.to_thread.run_sync(part.checkpoint)
        progress.received(part.coverage.covered)
        stalls = 0 if part.coverage.covered > before else stalls + 1
        if stalls >= _STALLED_PASSES:
            raise trio.TooSlowError(f"transfer stalled at byte {start}")


async def fetch_file(
    ftp: FTPClient,
    remote: str,
    local: Path,
    progress: FileProgress,
) -> dict[str, object]:
    """Downloads one closed file and publishes it only after both checks pass.

    The remote whole-file CRC32 must equal the local one; the receipt records
    the SHA-256 of the published bytes.
    """
    size = await ftp.open_read(remote)
    reused = await trio.to_thread.run_sync(reuse_verified, local, size)
    if reused is not None:
        with trio.CancelScope(shield=True), trio.move_on_after(5):
            await ftp.terminate()
        progress.begin_file(local.name, size)
        return reused

    started = monotonic()
    part = PartialFile(local.with_name(local.name + ".part"), size)
    progress.begin_file(local.name, part.coverage.covered)
    try:
        try:
            await _fill(ftp, part, progress)
        finally:
            with trio.CancelScope(shield=True):
                await trio.to_thread.run_sync(part.checkpoint)
                with trio.move_on_after(5):
                    await ftp.terminate()
        progress.verifying()
        remote_crc = await ftp.crc32(remote, 60 + size / _CRC_BYTES_PER_SECOND)
        local_crc, sha256 = await trio.to_thread.run_sync(digest, part.path)
        if local_crc != remote_crc:
            part.discard()
            raise VerificationError(
                f"{local.name}: CRC32 {local_crc:08x} differs from the flight "
                f"controller's {remote_crc:08x}; partial copy discarded"
            )
        receipt: dict[str, object] = {
            "file": local.name,
            "remote_path": remote,
            "bytes": size,
            "sha256": sha256,
            "remote_crc32": f"{remote_crc:08x}",
            "resumed_from_bytes": part.resumed_bytes,
            "transfer_s": round(monotonic() - started, 3),
            "verified_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
        write_json(local.with_name(local.name + ".json"), receipt)
        part.publish(local)
    finally:
        part.close()
    return receipt
