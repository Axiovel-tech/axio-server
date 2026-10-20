"""Tests for the `.gdmb` ground validator against the golden vectors of
AxioStack `contracts/gdm-vectors/` (vendored in `test/fixtures/gdm-vectors`).
"""

import json
import os
import struct
from pathlib import Path
from zipfile import ZipFile

from pytest import mark, raises

from flockwave.server.show.gdmb import (
    GDMB_MAX_SIZE,
    GdmbValidationError,
    validate_gdmb,
)
from flockwave.server.show.utils import crc32_mavftp

VECTORS = Path(
    os.environ.get("GDM_VECTORS_DIR")
    or Path(__file__).parent / "fixtures" / "gdm-vectors"
)
EDGE_INDEX = json.loads((VECTORS / "edge" / "index.json").read_text())
SHOW_HASHES = {
    name: bytes.fromhex(value) for name, value in EDGE_INDEX["show_hash"].items()
}
FULL_VECTORS = ("flat-single", "maneuver-single")


def _expected(vector: str) -> dict:
    return json.loads((VECTORS / vector / "expected.json").read_text())


def _hex_file(name: str) -> bytes:
    return bytes.fromhex((VECTORS / "edge" / name).read_text())


def _show_hash_of_base(data: bytes) -> bytes:
    """Every edge vector is a mutation of one of the two full vectors that
    leaves the header show hash alone. Returns the hash of that base vector,
    which is what the ground path would expect for the file.
    """
    (show_hash,) = [h for h in SHOW_HASHES.values() if data[12:44] == h]
    return show_hash


def _recrc(data: bytearray) -> bytes:
    data[8:12] = b"\x00\x00\x00\x00"
    struct.pack_into("<I", data, 8, crc32_mavftp(bytes(data)))
    return bytes(data)


@mark.parametrize("vector", FULL_VECTORS)
def test_accepts_vector(vector: str):
    data = (VECTORS / vector / "d01.gdmb").read_bytes()
    expected = _expected(vector)
    show_hash = bytes.fromhex(expected["show_hash"])

    summary = validate_gdmb(data, show_hash=show_hash)

    gdmb = expected["gdmb"]["d01"]
    assert summary.size == gdmb["size"] == len(data)
    assert summary.crc32 == int(gdmb["crc32"], 16)
    assert crc32_mavftp(data) == int(gdmb["mavftp_file_crc32"], 16)
    assert summary.show_hash == show_hash


@mark.parametrize("vector", FULL_VECTORS)
def test_accepts_gdmb_embedded_in_gdm(vector: str):
    with ZipFile(VECTORS / vector / "show.gdm") as archive:
        manifest = json.loads(archive.read("manifest.json"))
        show = json.loads(archive.read("show.json"))
        payloads = {
            drone["id"]: archive.read(f"drones/{drone['id']}.gdmb")
            for drone in show["drones"]
        }

    show_hash = bytes.fromhex(manifest["show_hash"])
    for data in payloads.values():
        assert validate_gdmb(data, show_hash=show_hash).show_hash == show_hash


def test_flat_vector_summary():
    data = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
    summary = validate_gdmb(data)
    assert summary.duration_us == 18_000_000
    assert summary.num_maneuvers == 0
    assert summary.rth_branch_times_us == (4_000_000, 7_000_000, 10_000_000)
    assert summary.features == 0x01
    assert summary.capabilities == 0


def test_maneuver_vector_summary():
    data = (VECTORS / "maneuver-single" / "d01.gdmb").read_bytes()
    summary = validate_gdmb(data)
    assert summary.num_maneuvers == 1
    assert summary.features == 0x03
    assert summary.capabilities == 0x01
    assert summary.rth_branch_times_us == ()


@mark.parametrize(
    "entry", EDGE_INDEX["accept"], ids=[e["file"] for e in EDGE_INDEX["accept"]]
)
def test_accepts_edge_vector(entry: dict):
    data = _hex_file(entry["file"])
    validate_gdmb(data, show_hash=_show_hash_of_base(data))


_REJECT_FILES = [e for e in EDGE_INDEX["reject"] if e.get("file")]


@mark.parametrize("entry", _REJECT_FILES, ids=[e["file"] for e in _REJECT_FILES])
def test_rejects_edge_vector(entry: dict):
    data = _hex_file(entry["file"])
    with raises(GdmbValidationError) as info:
        validate_gdmb(data, show_hash=_show_hash_of_base(data))
    assert info.value.code == entry["code"]


def test_rejects_oversize_file():
    # Recipe from edge/index.json: flat-single plus a skippable COMMENT padded
    # with spaces so that the file is one byte over the limit.
    (recipe,) = [e for e in EDGE_INDEX["reject"] if not e.get("file")]
    assert recipe["code"] == "E_SIZE"

    data = bytearray((VECTORS / "flat-single" / "d01.gdmb").read_bytes())
    pad = GDMB_MAX_SIZE + 1 - len(data) - 6
    data += struct.pack("<BBI", 0x03, 0x01, pad) + b" " * pad
    data = _recrc(data)
    assert len(data) == GDMB_MAX_SIZE + 1

    with raises(GdmbValidationError) as info:
        validate_gdmb(data)
    assert info.value.code == "E_SIZE"


def test_rejects_too_short_file():
    with raises(GdmbValidationError) as info:
        validate_gdmb(b"GDMB")
    assert info.value.code == "E_SIZE"


def test_rejects_show_hash_of_another_show():
    data = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
    with raises(GdmbValidationError) as info:
        validate_gdmb(data, show_hash=SHOW_HASHES["maneuver-single"])
    assert info.value.code == "E_SHOW_HASH"


def test_rejects_truncated_file():
    data = bytearray((VECTORS / "flat-single" / "d01.gdmb").read_bytes())
    with raises(GdmbValidationError) as info:
        validate_gdmb(_recrc(data[:-1]))
    assert info.value.code == "E_FRAMING"


def test_every_reject_code_is_exercised():
    # Guards against a vector set that silently shrinks: the spec's golden
    # set has 27 reject files plus the oversize recipe, and one accept file.
    assert len(_REJECT_FILES) == 27
    assert len(EDGE_INDEX["accept"]) == 1
