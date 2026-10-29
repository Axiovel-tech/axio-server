"""Tests for the drone's show slot: the MAVFTP write of `/axio/show.bin` and
the firmware version check (AxioStack#89, decision 1).
"""

from pathlib import Path

from pytest import mark, raises

from flockwave.server.ext.mavlink.ftp import (
    MAVFTPErrorCode,
    OperationNotAcknowledgedError,
)
from flockwave.server.ext.mavlink.show_slot import (
    LEGACY_SHOW_PATH,
    SHOW_SLOT_PATH,
    ShowUploadRefusedError,
    check_show_slot_version,
    write_show_slot,
)

VECTORS = Path(__file__).parent / "fixtures" / "gdm-vectors"
FLAT = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()


class FakeFTP:
    """Records MAVFTP operations against an in-memory file system."""

    def __init__(self, files: dict[str, bytes] | None = None):
        self.files = dict(files or {})
        self.calls: list[tuple] = []

    async def mkdir(self, path, parents=False, exist_ok=False):
        self.calls.append(("mkdir", path))

    async def put(self, data, remote_path, *, parents=False, skip_crc_check=False):
        self.calls.append(("put", remote_path))
        self.files[remote_path] = bytes(data)

    async def rm(self, path):
        self.calls.append(("rm", path))
        if path not in self.files:
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_NOT_FOUND)
        del self.files[path]


async def test_write_removes_slot_and_legacy_file_then_puts_slot():
    ftp = FakeFTP({SHOW_SLOT_PATH: b"old show", LEGACY_SHOW_PATH: b"older show"})
    await write_show_slot(ftp, FLAT)

    assert ftp.files == {SHOW_SLOT_PATH: FLAT}
    assert ftp.calls == [
        ("rm", SHOW_SLOT_PATH),
        ("rm", LEGACY_SHOW_PATH),
        ("mkdir", "/axio"),
        ("put", SHOW_SLOT_PATH),
    ]


async def test_write_without_previous_show():
    ftp = FakeFTP()
    await write_show_slot(ftp, FLAT)
    assert ftp.files == {SHOW_SLOT_PATH: FLAT}


async def test_write_propagates_unexpected_rm_errors_before_writing():
    class ProtectedFTP(FakeFTP):
        async def rm(self, path):
            self.calls.append(("rm", path))
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_PROTECTED)

    ftp = ProtectedFTP({SHOW_SLOT_PATH: b"old show"})
    with raises(OperationNotAcknowledgedError):
        await write_show_slot(ftp, FLAT)

    assert ftp.calls == [("rm", SHOW_SLOT_PATH)]


async def test_write_propagates_crc_failure_of_put():
    class CorruptingFTP(FakeFTP):
        async def put(self, data, remote_path, *, parents=False, skip_crc_check=False):
            raise RuntimeError("CRC mismatch, expected 0000ABCD, got 00001234")

    with raises(RuntimeError, match="CRC mismatch"):
        await write_show_slot(CorruptingFTP(), FLAT)


def test_show_slot_version_1_is_accepted():
    check_show_slot_version(1)
    check_show_slot_version(1.0)


def test_missing_show_slot_version_means_firmware_too_old():
    with raises(ShowUploadRefusedError, match="firmware is too old") as info:
        check_show_slot_version(None)
    assert info.value.code == "E_SHOW_SLOT_UNSUPPORTED"
    assert str(info.value).startswith("E_SHOW_SLOT_UNSUPPORTED: ")


@mark.parametrize("version", [0, 2, 1.5])
def test_other_show_slot_versions_are_refused(version):
    with raises(ShowUploadRefusedError, match="show slot version") as info:
        check_show_slot_version(version)
    assert info.value.code == "E_SHOW_SLOT_UNSUPPORTED"
