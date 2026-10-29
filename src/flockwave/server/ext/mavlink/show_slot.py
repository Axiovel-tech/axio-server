"""The drone's show slot: the one file that holds the show a drone flies,
whatever its format.

Firmware with the Axio show slot (`SHOW_SLOT_VER = 1`) reads `/axio/show.bin`
only, tells `.skyb` from `.gdmb` by the first bytes and requires a valid CRC
for both. Both show uploads (`__show_upload` and `__gdm_upload`) write the slot
the same way (AxioStack#89, decision 1):

1. refuse if the drone is armed, and check `SHOW_SLOT_VER`;
2. remove `/axio/show.bin` and the legacy `/collmot/show.skyb`;
3. put `/axio/show.bin`, with the CRC read-back;
4. configure the show origin, then reload the show.

Removing the legacy file as well means that a drone that missed the reflash
ends up with no show, never a stale one.
"""

from __future__ import annotations

from typing import Protocol

from .ftp import FTPPath, MAVFTPErrorCode, OperationNotAcknowledgedError

__all__ = (
    "LEGACY_SHOW_PATH",
    "SHOW_SLOT_PATH",
    "SHOW_SLOT_VERSION",
    "SHOW_SLOT_VERSION_PARAMETER",
    "ShowFileTransfer",
    "ShowUploadRefusedError",
    "check_show_slot_version",
    "write_show_slot",
)

SHOW_SLOT_DIRECTORY = "/axio"

SHOW_SLOT_PATH = "/axio/show.bin"
"""The show slot on the vehicle."""

LEGACY_SHOW_PATH = "/collmot/show.skyb"
"""Where `.skyb` shows lived before the show slot; removed by every upload."""

SHOW_SLOT_VERSION_PARAMETER = "SHOW_SLOT_VER"
"""Parameter with which the firmware advertises the show slot."""

SHOW_SLOT_VERSION = 1
"""The show slot version this server writes."""


class ShowUploadRefusedError(RuntimeError):
    """A show upload refused for a reason that retrying cannot fix.

    The message starts with a stable code (`E_ARMED`,
    `E_SHOW_SLOT_UNSUPPORTED`), the same convention as the `.gdmb` validation
    errors. Axio Control treats a server error that starts with an `E_` code
    as permanent and does not retry the upload automatically.
    """

    code: str

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code


class ShowFileTransfer(Protocol):
    """The MAVFTP operations that `write_show_slot()` needs; `MAVFTP`
    provides them.
    """

    async def mkdir(
        self, path: FTPPath, parents: bool = False, exist_ok: bool = False
    ) -> None: ...

    async def put(self, data: bytes, remote_path: FTPPath) -> None: ...

    async def rm(self, path: FTPPath) -> None: ...


def check_show_slot_version(version: float | None) -> None:
    """Refuses an upload to a drone whose firmware does not implement the show
    slot this server writes.

    Parameters:
        version: the value of `SHOW_SLOT_VER`, or `None` if the firmware does
            not have the parameter

    Raises:
        ShowUploadRefusedError: if the firmware does not support the slot.
            Uploading anyway would leave the drone with a show it never loads.
    """
    if version is None:
        raise ShowUploadRefusedError(
            "E_SHOW_SLOT_UNSUPPORTED",
            f"This drone's firmware is too old: it has no "
            f"{SHOW_SLOT_VERSION_PARAMETER} parameter and does not load shows "
            f"from {SHOW_SLOT_PATH}; update the firmware.",
        )
    if version != SHOW_SLOT_VERSION:
        raise ShowUploadRefusedError(
            "E_SHOW_SLOT_UNSUPPORTED",
            f"This drone's firmware implements show slot version {version:g} "
            f"({SHOW_SLOT_VERSION_PARAMETER}); this server writes version "
            f"{SHOW_SLOT_VERSION}. Update the firmware or the server.",
        )


async def _remove_if_present(ftp: ShowFileTransfer, path: str) -> None:
    try:
        await ftp.rm(path)
    except OperationNotAcknowledgedError as ex:
        if ex.code != MAVFTPErrorCode.FILE_NOT_FOUND:
            raise


async def write_show_slot(ftp: ShowFileTransfer, data: bytes) -> None:
    """Writes a show file (`.skyb` or `.gdmb`) to the drone's show slot.

    Removes the slot and the legacy `.skyb` first (a missing file is fine),
    then writes the slot. `MAVFTP.put()` compares the vehicle's whole-file CRC
    with the CRC of the bytes sent. Both formats carry their CRC in the
    header, so a match proves that the stored file is intact *and* is this
    show, not a leftover.

    The caller must have checked that the drone is disarmed and supports the
    slot. An interrupted write leaves a partial slot, which the firmware
    rejects by its CRC: the drone then has no show, never a wrong one.
    """
    await _remove_if_present(ftp, SHOW_SLOT_PATH)
    await _remove_if_present(ftp, LEGACY_SHOW_PATH)
    await ftp.mkdir(SHOW_SLOT_DIRECTORY, exist_ok=True)
    await ftp.put(data, SHOW_SLOT_PATH)
