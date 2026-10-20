"""Tests for the ground side of `OBJ-CMD __gdm_upload`: request decoding and
the atomic MAVFTP replace of AxioStack `contracts/gdm-format.md` §9.2.
"""

import json
from base64 import b64encode
from pathlib import Path

from pytest import raises

from flockwave.server.ext.mavlink.ftp import (
    MAVFTP,
    MAVFTPErrorCode,
    MAVFTPMessage,
    MAVFTPOpCode,
    OperationNotAcknowledgedError,
)
from flockwave.server.ext.mavlink.gdm_upload import (
    GDMB_LIVE_PATH,
    GDMB_TEMP_PATH,
    decode_gdm_upload_request,
    upload_gdmb,
)
from flockwave.server.show.gdmb import GDMB_MAX_SIZE, GdmbValidationError
from flockwave.server.show.utils import crc32_mavftp

VECTORS = Path(__file__).parent / "fixtures" / "gdm-vectors"
FLAT = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
FLAT_HASH = json.loads((VECTORS / "flat-single" / "expected.json").read_text())[
    "show_hash"
]
MANEUVER_HASH = json.loads((VECTORS / "maneuver-single" / "expected.json").read_text())[
    "show_hash"
]


class FakeFTP:
    """Records MAVFTP operations against an in-memory file system that
    behaves like ArduPilot's FATFS backend (no rename onto an existing file).
    """

    def __init__(self, files: dict[str, bytes] | None = None):
        self.files = dict(files or {})
        self.dirs: set[str] = set()
        self.calls: list[tuple] = []
        self.corrupt_on_rename = False

    async def mkdir(self, path, parents=False, exist_ok=False):
        self.calls.append(("mkdir", path))
        self.dirs.add(path)

    async def put(self, data, remote_path, *, parents=False, skip_crc_check=False):
        self.calls.append(("put", remote_path))
        self.files[remote_path] = bytes(data)

    async def rm(self, path):
        self.calls.append(("rm", path))
        if path not in self.files:
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_NOT_FOUND)
        del self.files[path]

    async def rename(self, old_path, new_path):
        self.calls.append(("rename", old_path, new_path))
        if new_path in self.files:
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_EXISTS)
        data = self.files.pop(old_path)
        self.files[new_path] = data[:-1] if self.corrupt_on_rename else data

    async def crc32(self, path):
        self.calls.append(("crc32", path))
        return crc32_mavftp(self.files[path])


def test_decode_valid_request():
    data, summary = decode_gdm_upload_request(b64encode(FLAT).decode(), FLAT_HASH)
    assert data == FLAT
    assert summary.show_hash.hex() == FLAT_HASH


def test_decode_rejects_show_hash_mismatch():
    with raises(GdmbValidationError) as info:
        decode_gdm_upload_request(b64encode(FLAT).decode(), MANEUVER_HASH)
    assert info.value.code == "E_SHOW_HASH"


def test_decode_rejects_corrupt_payload():
    corrupt = bytearray(FLAT)
    corrupt[100] ^= 0x01
    with raises(GdmbValidationError) as info:
        decode_gdm_upload_request(b64encode(bytes(corrupt)).decode(), FLAT_HASH)
    assert info.value.code == "E_CRC"


def test_decode_rejects_oversize_payload_before_decoding():
    too_long = "A" * (4 * ((GDMB_MAX_SIZE + 2) // 3) + 4)
    with raises(GdmbValidationError) as info:
        decode_gdm_upload_request(too_long, FLAT_HASH)
    assert info.value.code == "E_SIZE"


def test_decode_rejects_malformed_arguments():
    with raises(ValueError, match="base64"):
        decode_gdm_upload_request("not base64!", FLAT_HASH)
    with raises(ValueError, match="show hash"):
        decode_gdm_upload_request(b64encode(FLAT).decode(), FLAT_HASH[:-2])
    with raises(ValueError, match="show hash"):
        decode_gdm_upload_request(b64encode(FLAT).decode(), "z" * 64)


async def test_upload_writes_temporary_file_then_renames():
    ftp = FakeFTP({GDMB_LIVE_PATH: b"old show"})
    await upload_gdmb(ftp, FLAT)

    assert ftp.files == {GDMB_LIVE_PATH: FLAT}
    assert ftp.calls == [
        ("mkdir", "/axio"),
        ("put", GDMB_TEMP_PATH),
        ("rm", GDMB_LIVE_PATH),
        ("rename", GDMB_TEMP_PATH, GDMB_LIVE_PATH),
        ("crc32", GDMB_LIVE_PATH),
    ]


async def test_upload_without_previous_show():
    ftp = FakeFTP()
    await upload_gdmb(ftp, FLAT)
    assert ftp.files == {GDMB_LIVE_PATH: FLAT}


async def test_upload_detects_crc_mismatch_after_rename():
    ftp = FakeFTP()
    ftp.corrupt_on_rename = True
    with raises(RuntimeError, match="CRC mismatch"):
        await upload_gdmb(ftp, FLAT)


async def test_upload_propagates_unexpected_rm_errors():
    class ProtectedFTP(FakeFTP):
        async def rm(self, path):
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_PROTECTED)

    ftp = ProtectedFTP({GDMB_LIVE_PATH: b"old show"})
    with raises(OperationNotAcknowledgedError):
        await upload_gdmb(ftp, FLAT)
    # The verified temporary file stays; the live file is untouched
    assert ftp.files[GDMB_LIVE_PATH] == b"old show"
    assert ftp.files[GDMB_TEMP_PATH] == FLAT


async def test_mavftp_rename_message():
    sent: list[MAVFTPMessage] = []

    async def send_and_wait(message, *, allow_nak=False):
        sent.append(message)
        return MAVFTPMessage(MAVFTPOpCode.ACK)

    ftp = MAVFTP(sender=None)  # type: ignore
    ftp._send_and_wait = send_and_wait  # type: ignore
    await ftp.rename(GDMB_TEMP_PATH, GDMB_LIVE_PATH)

    (message,) = sent
    assert message.opcode == MAVFTPOpCode.RENAME
    # Leading slashes are stripped like every other MAVFTP path; ArduPilot
    # expects the two names separated by a single NUL, size = len1 + len2 + 1
    assert message.data == b"axio/show.gdmb.part\x00axio/show.gdmb"
    assert message.encode(0)[4] == len(message.data)  # size byte
