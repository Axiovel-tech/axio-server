"""Ground side of the `.gdm` show upload (`OBJ-CMD __gdm_upload`).

Axio Control sends each UAV its precompiled, certified `.gdmb` together with
the show hash of the `.gdm` it came from. Axio Server never re-encodes the
payload (ADR 0002 decision D2): it validates it against AxioStack
`contracts/gdm-format.md` §7.1, then writes it to the vehicle with an atomic
replace (decision D6, spec §9.2).
"""

from __future__ import annotations

from base64 import b64decode
from binascii import Error as BinasciiError
from typing import Protocol

from flockwave.server.show.gdmb import (
    GDMB_MAX_SIZE,
    GdmbSummary,
    GdmbValidationError,
    validate_gdmb,
)
from flockwave.server.show.utils import crc32_mavftp

from .ftp import FTPPath, MAVFTPErrorCode, OperationNotAcknowledgedError

__all__ = (
    "GDMB_LIVE_PATH",
    "GDMB_TEMP_PATH",
    "decode_gdm_upload_request",
    "upload_gdmb",
)

GDMB_LIVE_PATH = "/axio/show.gdmb"
"""Path of the live `.gdmb` on the vehicle (spec §9.3)."""

GDMB_TEMP_PATH = "/axio/show.gdmb.part"
"""Temporary path that the payload is written to and verified at before it
replaces the live file (spec §9.3).
"""

_MAX_BASE64_LENGTH = 4 * ((GDMB_MAX_SIZE + 2) // 3)
"""Length of the longest base64 string that can decode to a valid `.gdmb`."""


class GdmbFileTransfer(Protocol):
    """The MAVFTP operations that `upload_gdmb()` needs; `MAVFTP` provides
    them.
    """

    async def crc32(self, path: FTPPath) -> int: ...

    async def mkdir(
        self, path: FTPPath, parents: bool = False, exist_ok: bool = False
    ) -> None: ...

    async def put(self, data: bytes, remote_path: FTPPath) -> None: ...

    async def rename(self, old_path: FTPPath, new_path: FTPPath) -> None: ...

    async def rm(self, path: FTPPath) -> None: ...


def decode_gdm_upload_request(gdmb: str, show_hash: str) -> tuple[bytes, GdmbSummary]:
    """Decodes and validates the arguments of an `__gdm_upload` request.

    Parameters:
        gdmb: the `.gdmb` payload, standard base64 with padding
        show_hash: the show hash of the `.gdm`, as 64 hexadecimal digits

    Returns:
        the raw `.gdmb` bytes and the validator's summary of them

    Raises:
        GdmbValidationError: if the payload breaks a rule of spec §7.1,
            including a header show hash that differs from `show_hash`
        ValueError: if an argument is not well-formed
    """
    if not isinstance(show_hash, str) or len(show_hash) != 64:
        raise ValueError("show hash must be 64 hexadecimal digits")
    try:
        expected_hash = bytes.fromhex(show_hash)
    except ValueError:
        raise ValueError("show hash must be 64 hexadecimal digits") from None

    if not isinstance(gdmb, str):
        raise ValueError(".gdmb payload must be a base64-encoded string")
    if len(gdmb) > _MAX_BASE64_LENGTH:
        # Reject before decoding so an oversized request costs nothing
        raise GdmbValidationError(
            "E_SIZE", f".gdmb payload is larger than {GDMB_MAX_SIZE} bytes"
        )
    try:
        data = b64decode(gdmb, validate=True)
    except (BinasciiError, ValueError):
        raise ValueError(".gdmb payload is not valid base64") from None

    return data, validate_gdmb(data, show_hash=expected_hash)


async def upload_gdmb(ftp: GdmbFileTransfer, data: bytes) -> None:
    """Writes a validated `.gdmb` to the vehicle with an atomic replace.

    1. Write the payload to `GDMB_TEMP_PATH`. `MAVFTP.put()` truncates the
       file first and compares the vehicle's whole-file CRC with the CRC of
       the bytes sent. The `.gdmb` CRC lives in its header, so a match proves
       that the stored file is intact *and* is this show (spec §4.9, §9.2).
    2. Remove the live file. ArduPilot's FATFS backend does not rename onto
       an existing file.
    3. Rename the temporary file over the live path, then check the CRC of
       the live file once more.

    A failure between steps 2 and 3 leaves no live `.gdmb` and a verified
    temporary file on the vehicle, never a partially written live file.
    """
    expected_crc = crc32_mavftp(data)

    await ftp.mkdir("/axio", exist_ok=True)
    await ftp.put(data, GDMB_TEMP_PATH)

    try:
        await ftp.rm(GDMB_LIVE_PATH)
    except OperationNotAcknowledgedError as ex:
        if ex.code != MAVFTPErrorCode.FILE_NOT_FOUND:
            raise

    await ftp.rename(GDMB_TEMP_PATH, GDMB_LIVE_PATH)

    observed_crc = await ftp.crc32(GDMB_LIVE_PATH)
    if observed_crc != expected_crc:
        raise RuntimeError(
            f"CRC mismatch after renaming the show file, expected "
            f"{expected_crc:08X}, got {observed_crc:08X}"
        )
