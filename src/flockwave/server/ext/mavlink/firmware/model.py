"""State model for one ArduPilot firmware update operation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal, TypedDict

import trio

OTAStatus = Literal["running", "success", "failed", "cancelled", "indeterminate"]
OTAPhase = Literal[
    "validating",
    "staging",
    "committing",
    "rebooting",
    "reconnecting",
    "verifyingInstalled",
    "complete",
]


class FirmwareIdentity(TypedDict):
    boardId: int | None
    gitHash: str | None
    version: str | None


class OTAError(TypedDict):
    code: str
    detail: str


@dataclass
class OTAJob:
    """Mutable server-side record exposed to Control as a JSON snapshot."""

    operation_id: str
    uav_id: str
    name: str
    status: OTAStatus = "running"
    phase: OTAPhase = "validating"
    transferred_bytes: int | None = None
    total_bytes: int | None = None
    committed: bool = False
    cancellable: bool = True
    expected: FirmwareIdentity = field(
        default_factory=lambda: {"boardId": None, "gitHash": None, "version": None}
    )
    observed: FirmwareIdentity = field(
        default_factory=lambda: {"boardId": None, "gitHash": None, "version": None}
    )
    error: OTAError | None = None
    cancel_requested: trio.Event = field(default_factory=trio.Event, repr=False)

    def json(self) -> dict[str, Any]:
        return {
            "id": self.uav_id,
            "operationId": self.operation_id,
            "status": self.status,
            "phase": self.phase,
            "bytesTransferred": self.transferred_bytes,
            "bytesTotal": self.total_bytes,
            "committed": self.committed,
            "cancellable": self.cancellable,
            "expectedHash": self.expected["gitHash"],
            "expectedVersion": self.expected["version"],
            "observedHash": self.observed["gitHash"],
            "observedVersion": self.observed["version"],
            "error": dict(self.error) if self.error else None,
        }

    def enter_commit(self) -> None:
        self.phase = "committing"
        self.cancellable = False

    def finish(self, status: OTAStatus, error: OTAError | None = None) -> None:
        self.status = status
        self.phase = "complete"
        self.cancellable = False
        self.error = error
