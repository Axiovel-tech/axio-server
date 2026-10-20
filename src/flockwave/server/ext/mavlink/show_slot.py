"""The drone's show slot: the one file that holds the show a drone flies,
whatever its format.

Firmware with the Axio show slot reads `/axio/show.bin` only, tells `.skyb`
from `.gdmb` by the first bytes, and ignores the legacy `/collmot/show.skyb`.
It advertises the slot with the `SHOW_SLOT_VER` parameter. Both show uploads
(`__show_upload` and `__gdm_upload`) write the slot the same way: to a
temporary name, verified, then renamed (ADR 0002 decision D6, AxioStack
`contracts/gdm-format.md` §9.2).
"""

from __future__ import annotations

from typing import Protocol

from flockwave.server.show.utils import crc32_mavftp

from .ftp import FTPPath, MAVFTPErrorCode, OperationNotAcknowledgedError

__all__ = (
    "SHOW_SLOT_PATH",
    "SHOW_SLOT_TEMP_PATH",
    "SHOW_SLOT_VERSION",
    "SHOW_SLOT_VERSION_PARAMETER",
    "ShowFileTransfer",
    "check_show_slot_version",
    "write_show_slot",
)

SHOW_SLOT_DIRECTORY = "/axio"

SHOW_SLOT_PATH = "/axio/show.bin"
"""The show slot on the vehicle."""

SHOW_SLOT_TEMP_PATH = "/axio/show.bin.part"
"""Temporary path that a show is written to and verified at before it
replaces the slot.
"""

SHOW_SLOT_VERSION_PARAMETER = "SHOW_SLOT_VER"
"""Parameter with which the firmware advertises the show slot."""

SHOW_SLOT_VERSION = 1
"""The show slot version this server writes."""


class ShowFileTransfer(Protocol):
    """The MAVFTP operations that `write_show_slot()` needs; `MAVFTP`
    provides them.
    """

    async def crc32(self, path: FTPPath) -> int: ...

    async def mkdir(
        self, path: FTPPath, parents: bool = False, exist_ok: bool = False
    ) -> None: ...

    async def put(self, data: bytes, remote_path: FTPPath) -> None: ...

    async def rename(self, old_path: FTPPath, new_path: FTPPath) -> None: ...

    async def rm(self, path: FTPPath) -> None: ...


def check_show_slot_version(version: float | None) -> None:
    """Refuses an upload to a drone whose firmware does not advertise the
    show slot this server writes.

    Parameters:
        version: the value of `SHOW_SLOT_VER`, or `None` if the drone did not
            report it

    Raises:
        RuntimeError: if the firmware does not support the slot. Uploading
            anyway would leave the drone flying its previous show.
    """
    if version is None or version != SHOW_SLOT_VERSION:
        found = "missing" if version is None else f"{version:g}"
        raise RuntimeError(
            "This drone's firmware does not support the /axio show slot; "
            f"update the firmware ({SHOW_SLOT_VERSION_PARAMETER} is {found}, "
            f"expected {SHOW_SLOT_VERSION})."
        )


async def write_show_slot(ftp: ShowFileTransfer, data: bytes) -> None:
    """Writes a show file (`.skyb` or `.gdmb`) to the drone's show slot with
    an atomic replace.

    1. Write the file to `SHOW_SLOT_TEMP_PATH`. `MAVFTP.put()` truncates the
       file first and compares the vehicle's whole-file CRC with the CRC of
       the bytes sent. Both formats carry their CRC in the header, so a match
       proves that the stored file is intact *and* is this show.
    2. Remove the live slot. ArduPilot's FATFS backend does not rename onto
       an existing file. (The upload has already asked the drone to remove its
       show, so normally there is nothing to remove.)
    3. Rename the temporary file over the slot, then check its CRC again.

    A failure between steps 2 and 3 leaves an empty slot and a verified
    temporary file on the vehicle, never a partially written slot.
    """
    expected_crc = crc32_mavftp(data)

    await ftp.mkdir(SHOW_SLOT_DIRECTORY, exist_ok=True)
    await ftp.put(data, SHOW_SLOT_TEMP_PATH)

    try:
        await ftp.rm(SHOW_SLOT_PATH)
    except OperationNotAcknowledgedError as ex:
        if ex.code != MAVFTPErrorCode.FILE_NOT_FOUND:
            raise

    await ftp.rename(SHOW_SLOT_TEMP_PATH, SHOW_SLOT_PATH)

    observed_crc = await ftp.crc32(SHOW_SLOT_PATH)
    if observed_crc != expected_crc:
        raise RuntimeError(
            f"CRC mismatch after renaming the show file, expected "
            f"{expected_crc:08X}, got {observed_crc:08X}"
        )
