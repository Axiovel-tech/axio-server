"""Tests for the drone's show slot: the firmware capability check and the
atomic MAVFTP replace of `/axio/show.bin` (ADR 0002 D6, AxioStack
`contracts/gdm-format.md` §9.2).
"""

from pathlib import Path

from pytest import mark, raises

from flockwave.server.ext.mavlink.ftp import (
    MAVFTP,
    MAVFTPErrorCode,
    MAVFTPMessage,
    MAVFTPOpCode,
    OperationNotAcknowledgedError,
)
from flockwave.server.ext.mavlink.show_slot import (
    SHOW_SLOT_PATH,
    SHOW_SLOT_TEMP_PATH,
    check_show_slot_version,
    write_show_slot,
)
from flockwave.server.show.utils import crc32_mavftp

VECTORS = Path(__file__).parent / "fixtures" / "gdm-vectors"
FLAT = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()


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


async def test_write_writes_temporary_file_then_renames():
    ftp = FakeFTP({SHOW_SLOT_PATH: b"old show"})
    await write_show_slot(ftp, FLAT)

    assert ftp.files == {SHOW_SLOT_PATH: FLAT}
    assert ftp.calls == [
        ("mkdir", "/axio"),
        ("put", SHOW_SLOT_TEMP_PATH),
        ("rm", SHOW_SLOT_PATH),
        ("rename", SHOW_SLOT_TEMP_PATH, SHOW_SLOT_PATH),
        ("crc32", SHOW_SLOT_PATH),
    ]


async def test_write_without_previous_show():
    ftp = FakeFTP()
    await write_show_slot(ftp, FLAT)
    assert ftp.files == {SHOW_SLOT_PATH: FLAT}


async def test_write_detects_crc_mismatch_after_rename():
    ftp = FakeFTP()
    ftp.corrupt_on_rename = True
    with raises(RuntimeError, match="CRC mismatch"):
        await write_show_slot(ftp, FLAT)


async def test_write_propagates_unexpected_rm_errors():
    class ProtectedFTP(FakeFTP):
        async def rm(self, path):
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_PROTECTED)

    ftp = ProtectedFTP({SHOW_SLOT_PATH: b"old show"})
    with raises(OperationNotAcknowledgedError):
        await write_show_slot(ftp, FLAT)
    # The verified temporary file stays; the live file is untouched
    assert ftp.files[SHOW_SLOT_PATH] == b"old show"
    assert ftp.files[SHOW_SLOT_TEMP_PATH] == FLAT


async def test_mavftp_rename_message():
    sent: list[MAVFTPMessage] = []

    async def send_and_wait(message, *, allow_nak=False):
        sent.append(message)
        return MAVFTPMessage(MAVFTPOpCode.ACK)

    ftp = MAVFTP(sender=None)  # type: ignore
    ftp._send_and_wait = send_and_wait  # type: ignore
    await ftp.rename(SHOW_SLOT_TEMP_PATH, SHOW_SLOT_PATH)

    (message,) = sent
    assert message.opcode == MAVFTPOpCode.RENAME
    # Leading slashes are stripped like every other MAVFTP path; ArduPilot
    # expects the two names separated by a single NUL, size = len1 + len2 + 1
    assert message.data == b"axio/show.bin.part\x00axio/show.bin"
    assert message.encode(0)[4] == len(message.data)  # size byte


def test_show_slot_version_1_is_accepted():
    check_show_slot_version(1)
    check_show_slot_version(1.0)


@mark.parametrize("version", [None, 0, 2, 1.5])
def test_other_show_slot_versions_are_refused(version):
    with raises(RuntimeError, match="does not support the /axio show slot"):
        check_show_slot_version(version)
