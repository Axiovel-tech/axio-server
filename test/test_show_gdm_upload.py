"""Tests for decoding and validating `OBJ-CMD __gdm_upload` requests."""

import json
from base64 import b64encode
from pathlib import Path

from pytest import raises

from flockwave.server.show.gdm_upload import decode_gdm_upload_request
from flockwave.server.show.gdmb import GDMB_MAX_SIZE, GdmbValidationError

VECTORS = Path(__file__).parent / "fixtures" / "gdm-vectors"
FLAT = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
FLAT_HASH = json.loads((VECTORS / "flat-single" / "expected.json").read_text())[
    "show_hash"
]
MANEUVER_HASH = json.loads((VECTORS / "maneuver-single" / "expected.json").read_text())[
    "show_hash"
]


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
