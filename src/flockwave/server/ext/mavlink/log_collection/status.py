"""Bounded progress status and run provenance for log collection."""

from __future__ import annotations

import hashlib
import platform
import subprocess
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from time import monotonic

__all__ = ("Status", "now_utc", "provenance")

_RATE_SAMPLES = 12
"""Status snapshots that the transfer rate averages over."""


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass
class Status:
    """Live state of one vehicle's collection; one snapshot has fixed fields."""

    uav: str
    phase: str = "guarding"
    file: str | None = None
    files_done: int = 0
    files_total: int = 0
    bytes_total: int = 0
    battery_voltage: float | None = None
    error: str | None = None
    _bytes_finished: int = 0
    _bytes_current: int = 0
    _bytes_received: int = 0
    _samples: deque[tuple[float, int]] = field(
        default_factory=lambda: deque(maxlen=_RATE_SAMPLES)
    )

    def begin_file(self, name: str, covered: int) -> None:
        self.file = name
        self.phase = "transferring"
        self._bytes_current = covered

    def received(self, covered: int) -> None:
        self._bytes_received += max(0, covered - self._bytes_current)
        self._bytes_current = covered

    def verifying(self) -> None:
        self.phase = "verifying"

    def end_file(self, size: int) -> None:
        self.files_done += 1
        self._bytes_finished += size
        self._bytes_current = 0
        self.file = None

    def snapshot(self) -> dict[str, object]:
        now = monotonic()
        self._samples.append((now, self._bytes_received))
        then, received_then = self._samples[0]
        rate = (
            (self._bytes_received - received_then) / (now - then)
            if now > then
            else None
        )
        done = self._bytes_finished + self._bytes_current
        eta = (
            (self.bytes_total - done) / rate
            if rate and self.phase == "transferring"
            else None
        )
        return {
            "uav": self.uav,
            "phase": self.phase,
            "file": self.file,
            "files_done": self.files_done,
            "files_total": self.files_total,
            "bytes_done": done,
            "bytes_total": self.bytes_total,
            "rate_Bps": None if rate is None else round(rate),
            "eta_s": None if eta is None else round(eta),
            "battery_v": self.battery_voltage,
            "error": self.error,
            "updated_at": now_utc(),
        }


def _package_version(name: str) -> str | None:
    try:
        return version(name)
    except PackageNotFoundError:
        return None


def _source_revision() -> dict[str, object] | None:
    source = Path(__file__).resolve().parent

    def git(*args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", "-C", str(source), *args],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )

    try:
        head = git("rev-parse", "HEAD")
        if head.returncode != 0:
            return None
        changes = git("status", "--porcelain", "--untracked-files=no").stdout
        diff = git("diff", "HEAD").stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return {
        "revision": head.stdout.strip(),
        "dirty": bool(changes.strip()),
        "diff_sha256": hashlib.sha256(diff.encode()).hexdigest(),
    }


def provenance() -> dict[str, object]:
    """What executed this collection, for the run receipt."""
    return {
        "tool": "axio-fc-logs",
        "flockwave_server": _package_version("flockwave-server"),
        "pymavlink": _package_version("pymavlink"),
        "python": platform.python_version(),
        "host": platform.node(),
        "source": _source_revision(),
    }
