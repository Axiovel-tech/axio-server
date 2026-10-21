"""Tests for the `.gdmb` ground validator against the golden vectors of
AxioStack `contracts/gdm-vectors/` (vendored in `test/fixtures/gdm-vectors`).

The v1 conformance gate is `flat-single/` and `edge/`. The `provisional/`
vectors exercise layouts outside v1 (spec §4.11): a v1 reader must refuse
them, and the validator's provisional mode checks them for completeness.
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
PROVISIONAL_EDGE_INDEX = json.loads(
    (VECTORS / "provisional" / "edge" / "index.json").read_text()
)
V1_VECTORS = ("flat-single",)
PROVISIONAL_VECTORS = ("provisional/flat-rth", "provisional/maneuver-single")


def _expected(vector: str) -> dict:
    return json.loads((VECTORS / vector / "expected.json").read_text())


SHOW_HASHES = {
    vector: bytes.fromhex(_expected(vector)["show_hash"])
    for vector in V1_VECTORS + PROVISIONAL_VECTORS
}


def _hex_file(path: Path) -> bytes:
    return bytes.fromhex(path.read_text())


def _show_hash_of_base(data: bytes) -> bytes:
    """Every edge vector is a mutation of a full vector that leaves the header
    show hash alone. Returns the hash of that base vector, which is what the
    ground path would expect for the file.
    """
    (show_hash,) = [h for h in SHOW_HASHES.values() if data[12:44] == h]
    return show_hash


def _recrc(data: bytearray) -> bytes:
    data[8:12] = b"\x00\x00\x00\x00"
    struct.pack_into("<I", data, 8, crc32_mavftp(bytes(data)))
    return bytes(data)


def _gdmb_files_of(vector: str) -> tuple[bytes, dict[str, bytes]]:
    with ZipFile(VECTORS / vector / "show.gdm") as archive:
        manifest = json.loads(archive.read("manifest.json"))
        show = json.loads(archive.read("show.json"))
        payloads = {
            drone["id"]: archive.read(f"drones/{drone['id']}.gdmb")
            for drone in show["drones"]
        }
    return bytes.fromhex(manifest["show_hash"]), payloads


# ---------------------------------------------------------------------------
# v1 conformance: flat-single and edge/
# ---------------------------------------------------------------------------


def test_edge_index_is_the_v1_set():
    # Guards against a vector set that silently shrinks: the spec's v1 gate
    # has 21 reject vectors (20 files and the oversize recipe) and 2 accepts.
    assert EDGE_INDEX["mode"] == "v1"
    assert len(EDGE_INDEX["reject"]) == 21
    assert len([e for e in EDGE_INDEX["reject"] if e.get("file")]) == 20
    assert len(EDGE_INDEX["accept"]) == 2


@mark.parametrize("vector", V1_VECTORS)
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


@mark.parametrize("vector", V1_VECTORS)
def test_accepts_gdmb_embedded_in_gdm(vector: str):
    show_hash, payloads = _gdmb_files_of(vector)
    for data in payloads.values():
        assert validate_gdmb(data, show_hash=show_hash).show_hash == show_hash


def test_flat_vector_summary():
    data = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
    summary = validate_gdmb(data)
    assert summary.size == 542
    assert summary.duration_us == 18_000_000
    assert summary.num_maneuvers == 0
    assert summary.rth_branch_times_us == ()
    assert summary.features == 0x01
    assert summary.capabilities == 0


@mark.parametrize(
    "entry", EDGE_INDEX["accept"], ids=[e["file"] for e in EDGE_INDEX["accept"]]
)
def test_accepts_edge_vector(entry: dict):
    data = _hex_file(VECTORS / "edge" / entry["file"])
    validate_gdmb(data, show_hash=_show_hash_of_base(data))


_REJECT_FILES = [e for e in EDGE_INDEX["reject"] if e.get("file")]


@mark.parametrize("entry", _REJECT_FILES, ids=[e["file"] for e in _REJECT_FILES])
def test_rejects_edge_vector(entry: dict):
    data = _hex_file(VECTORS / "edge" / entry["file"])
    with raises(GdmbValidationError) as info:
        validate_gdmb(data, show_hash=_show_hash_of_base(data))
    assert info.value.code == entry["code"]


def test_header_extension_is_skipped():
    # header-extension carries 4 extra header bytes that a v1 reader skips
    data = _hex_file(VECTORS / "edge" / "header-extension.gdmb.hex")
    assert struct.unpack_from("<H", data, 6)[0] == 52
    summary = validate_gdmb(data)
    assert summary.duration_us == 18_000_000


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


def test_rejects_header_longer_than_file():
    data = bytearray((VECTORS / "flat-single" / "d01.gdmb").read_bytes())
    struct.pack_into("<H", data, 6, len(data) + 1)
    with raises(GdmbValidationError) as info:
        validate_gdmb(_recrc(data))
    assert info.value.code == "E_HEADER"


def test_rejects_show_hash_of_another_show():
    data = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
    with raises(GdmbValidationError) as info:
        validate_gdmb(data, show_hash=SHOW_HASHES["provisional/flat-rth"])
    assert info.value.code == "E_SHOW_HASH"


def test_rejects_truncated_file():
    data = bytearray((VECTORS / "flat-single" / "d01.gdmb").read_bytes())
    with raises(GdmbValidationError) as info:
        validate_gdmb(_recrc(data[:-1]))
    assert info.value.code == "E_FRAMING"


# ---------------------------------------------------------------------------
# Provisional layouts (spec §4.11): outside the v1 gate
# ---------------------------------------------------------------------------


@mark.parametrize("vector", PROVISIONAL_VECTORS)
def test_v1_reader_refuses_provisional_vector(vector: str):
    data = (VECTORS / vector / "d01.gdmb").read_bytes()
    with raises(GdmbValidationError) as info:
        validate_gdmb(data, show_hash=SHOW_HASHES[vector])
    assert info.value.code in ("E_BLOCK_TYPE", "E_RESERVED", "E_CAPS")


@mark.parametrize("vector", PROVISIONAL_VECTORS)
def test_provisional_mode_accepts_provisional_vector(vector: str):
    show_hash, payloads = _gdmb_files_of(vector)
    for data in payloads.values():
        validate_gdmb(data, show_hash=show_hash, provisional=True)
    assert (VECTORS / vector / "d01.gdmb").read_bytes() in payloads.values()


def test_provisional_vector_summaries():
    rth = validate_gdmb(
        (VECTORS / "provisional" / "flat-rth" / "d01.gdmb").read_bytes(),
        provisional=True,
    )
    assert rth.rth_branch_times_us == (4_000_000, 7_000_000, 10_000_000)

    maneuver = validate_gdmb(
        (VECTORS / "provisional" / "maneuver-single" / "d01.gdmb").read_bytes(),
        provisional=True,
    )
    assert maneuver.num_maneuvers == 1
    assert maneuver.features == 0x03
    assert maneuver.capabilities == 0x01


_PROVISIONAL_REJECTS = PROVISIONAL_EDGE_INDEX["reject"]


@mark.parametrize(
    "entry", _PROVISIONAL_REJECTS, ids=[e["file"] for e in _PROVISIONAL_REJECTS]
)
def test_provisional_mode_rejects_provisional_edge_vector(entry: dict):
    data = _hex_file(VECTORS / "provisional" / "edge" / entry["file"])
    with raises(GdmbValidationError) as info:
        validate_gdmb(data, show_hash=_show_hash_of_base(data), provisional=True)
    assert info.value.code == entry["code"]
