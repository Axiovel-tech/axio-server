"""Tests for bounded ArduPilot APJ parsing."""

import base64
import hashlib
import json
import struct
import zlib

import pytest

from flockwave.server.ext.mavlink.firmware.apj import (
    APP_DESCRIPTOR_SIGNATURE,
    MAX_APJ_SIZE,
    MAX_IMAGE_SIZE_BY_BOARD,
    APJValidationError,
    _decode_document,
    _decode_image,
    _reject_external_flash,
    _require_git_hash,
    _require_int,
    _validate_app_descriptor,
    _validate_envelope,
    parse_apj,
)

IMAGE_HEAD = b"vectors!"
DESCRIPTOR_START = len(IMAGE_HEAD)
NO_DESCRIPTOR = "Firmware image has no ArduPilot application descriptor"
CORRUPT = "Firmware image is corrupt: its checksum does not match its descriptor"


def crc32_small(data: bytes) -> int:
    """Bit-by-bit port of ArduPilot's ``crc32_small`` with a zero seed."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def make_image(
    body: bytes = b"firmware",
    *,
    board_id: int = 1177,
    described_size: int | None = None,
) -> bytes:
    """Builds an application image as ArduPilot's ``set_app_descriptor`` does."""
    head = IMAGE_HEAD + APP_DESCRIPTOR_SIGNATURE
    tail = struct.pack("<BBH8x", 4, 7, board_id) + body
    size = len(head) + 16 + len(tail)
    descriptor = struct.pack(
        "<IIII",
        crc32_small(head),
        crc32_small(tail),
        size if described_size is None else described_size,
        0x0123ABCD,
    )
    return head + descriptor + tail


def make_apj(
    image: bytes | None = None,
    *,
    board_id: int = 1177,
    git_hash: str = "0123abcd",
    declared_size: int | None = None,
) -> bytes:
    if image is None:
        image = make_image()
    document = {
        "magic": "APJFWv1",
        "board_id": board_id,
        "git_identity": git_hash,
        "image_size": len(image) if declared_size is None else declared_size,
        "image": base64.b64encode(zlib.compress(image)).decode("ascii"),
        "extf_image_size": 0,
        "signed_firmware": True,
        "version": "4.6.1",
    }
    return json.dumps(document).encode("utf-8")


def parse(payload: bytes):
    return parse_apj(
        payload,
        expected_sha256=hashlib.sha256(payload).hexdigest(),
        name="arducopter.apj",
    )


def encode_document(**changes) -> bytes:
    document = json.loads(make_apj())
    document.update(changes)
    return json.dumps(document).encode()


def assert_validation_error(call, code: str, detail: str) -> None:
    with pytest.raises(APJValidationError) as raised:
        call()
    assert raised.value.code == code
    assert str(raised.value) == detail


def test_apj_is_converted_to_md5_checked_abin() -> None:
    firmware = make_image()
    image = parse(make_apj(firmware))

    assert image.board_id == 1177
    assert image.git_hash == "0123abcd"
    assert image.version == "4.6.1"
    assert image.total_size == len(image.abin)
    assert image.abin == (
        b"git version: 0123abcd\nMD5: 99dfa3a69aff84bf9f58ec40e09aef3e\n--\n" + firmware
    )


def test_abin_md5_is_explicitly_marked_non_security(monkeypatch) -> None:
    calls = []

    class Digest:
        def hexdigest(self) -> str:
            return "digest"

    def md5(data, *, usedforsecurity):
        calls.append((data, usedforsecurity))
        return Digest()

    monkeypatch.setattr(hashlib, "md5", md5)
    firmware = make_image()
    image = parse(make_apj(firmware))
    assert calls == [(firmware, False)]
    assert image.abin == b"git version: 0123abcd\nMD5: digest\n--\n" + firmware


@pytest.mark.parametrize(
    ("payload", "sha256", "code", "detail"),
    [
        (
            make_apj(),
            "0" * 64,
            "hashMismatch",
            "APJ SHA-256 does not match the request",
        ),
        (
            make_apj(board_id=42),
            None,
            "unsupportedBoard",
            "Unsupported ArduPilot board ID: 42",
        ),
        (
            make_apj(declared_size=999),
            None,
            "sizeMismatch",
            "APJ declares 999 image bytes but contains 52",
        ),
        (
            b"not-json",
            None,
            "invalidContainer",
            "APJ is not valid UTF-8 JSON",
        ),
    ],
)
def test_apj_rejects_bad_integrity_or_metadata(
    payload: bytes, sha256: str | None, code: str, detail: str
) -> None:
    with pytest.raises(APJValidationError) as raised:
        parse_apj(
            payload,
            expected_sha256=sha256 or hashlib.sha256(payload).hexdigest(),
            name="arducopter.apj",
        )
    assert raised.value.code == code
    assert str(raised.value) == detail


def test_apj_decompression_is_bounded() -> None:
    payload = make_apj(b"x" * (MAX_IMAGE_SIZE_BY_BOARD[1177] + 1))
    with pytest.raises(APJValidationError) as raised:
        parse(payload)
    assert raised.value.code == "imageTooLarge"
    assert str(raised.value) == (
        f"Firmware image exceeds the {MAX_IMAGE_SIZE_BY_BOARD[1177]}-byte board limit"
    )


@pytest.mark.parametrize("version", [None, 7, "", "x" * 65])
def test_apj_rejects_invalid_version(version) -> None:
    payload = encode_document(version=version)
    assert_validation_error(
        lambda: parse(payload),
        "invalidMetadata",
        "version must contain 1 to 64 characters",
    )


def test_apj_accepts_metadata_boundaries_and_normalizes_hash() -> None:
    payload = encode_document(version="x" * 64, git_identity="ABCDEF12")
    image = parse(payload)
    assert image.version == "x" * 64
    assert image.git_hash == "abcdef12"


@pytest.mark.parametrize("name", [None, "", "x" * 129, "firmware.bin"])
def test_apj_rejects_invalid_names(name) -> None:
    payload = make_apj()
    expected_hash = hashlib.sha256(payload).hexdigest()
    detail = (
        "Firmware name must end in .apj"
        if name == "firmware.bin"
        else "Firmware name must contain 1 to 128 characters"
    )
    assert_validation_error(
        lambda: _validate_envelope(payload, expected_hash, name),
        "invalidName",
        detail,
    )


def test_apj_accepts_maximum_name_length_case_insensitively() -> None:
    payload = make_apj()
    _validate_envelope(
        payload,
        hashlib.sha256(payload).hexdigest(),
        f"{'x' * 124}.APJ",
    )


@pytest.mark.parametrize("expected_hash", [None, "A" * 64, "0" * 63])
def test_apj_rejects_malformed_expected_hash(expected_hash) -> None:
    payload = make_apj()
    assert_validation_error(
        lambda: _validate_envelope(payload, expected_hash, "firmware.apj"),
        "invalidHash",
        "sha256 must be 64 lowercase hexadecimal characters",
    )


def test_apj_envelope_size_boundaries() -> None:
    maximum = b"x" * MAX_APJ_SIZE
    _validate_envelope(maximum, hashlib.sha256(maximum).hexdigest(), "firmware.apj")
    for payload in (b"", maximum + b"x"):
        assert_validation_error(
            lambda payload=payload: _validate_envelope(
                payload, hashlib.sha256(payload).hexdigest(), "firmware.apj"
            ),
            "invalidSize",
            f"APJ file must contain 1 to {MAX_APJ_SIZE} bytes",
        )


@pytest.mark.parametrize(
    ("payload", "detail"),
    [
        (b"\xff", "APJ is not valid UTF-8 JSON"),
        (b"[]", "APJ magic must be APJFWv1"),
        (b'{"magic":"wrong"}', "APJ magic must be APJFWv1"),
    ],
)
def test_apj_document_validation(payload: bytes, detail: str) -> None:
    assert_validation_error(
        lambda: _decode_document(payload), "invalidContainer", detail
    )


@pytest.mark.parametrize(
    ("encoded", "detail"),
    [
        (None, "APJ image must be a base64 string"),
        (" eA==", "APJ image is not valid base64"),
        (base64.b64encode(b"not-zlib").decode(), "APJ image is not valid zlib data"),
        (
            base64.b64encode(zlib.compress(b"abc")[:-1]).decode(),
            "APJ image has incomplete or trailing zlib data",
        ),
        (
            base64.b64encode(zlib.compress(b"abc") + b"trailing").decode(),
            "APJ image has incomplete or trailing zlib data",
        ),
        (base64.b64encode(zlib.compress(b"")).decode(), "APJ firmware image is empty"),
    ],
)
def test_apj_rejects_invalid_compressed_images(encoded, detail: str) -> None:
    assert_validation_error(
        lambda: _decode_image({"image": encoded}, max_size=64),
        "invalidImage",
        detail,
    )


def test_apj_image_exactly_at_board_limit_is_valid() -> None:
    limit = MAX_IMAGE_SIZE_BY_BOARD[1177]
    image = b"x" * limit
    encoded = base64.b64encode(zlib.compress(image)).decode()
    assert _decode_image({"image": encoded}, max_size=limit) == image


def test_decompressor_receives_explicit_output_bounds(monkeypatch) -> None:
    calls = []

    class Inflater:
        unconsumed_tail = b""
        eof = True
        unused_data = b""

        def decompress(self, compressed, max_length):
            calls.append(("decompress", compressed, max_length))
            return b"x"

        def flush(self, max_length):
            calls.append(("flush", max_length))
            return b""

    monkeypatch.setattr(zlib, "decompressobj", Inflater)
    assert (
        _decode_image({"image": base64.b64encode(b"z").decode()}, max_size=10) == b"x"
    )
    assert calls == [("decompress", b"z", 11), ("flush", 10)]


@pytest.mark.parametrize("value", [None, "1", True, -1])
def test_integer_metadata_rejects_non_integer_or_negative_values(value) -> None:
    assert_validation_error(
        lambda: _require_int({"value": value}, "value"),
        "invalidMetadata",
        "value must be a non-negative integer",
    )
    assert _require_int({"value": 0}, "value") == 0


@pytest.mark.parametrize("value", [None, 123, "abcdef", "a" * 7, "a" * 9, "a" * 40])
def test_git_hash_validation(value) -> None:
    assert_validation_error(
        lambda: _require_git_hash({"git_identity": value}),
        "invalidMetadata",
        "git_identity must be an 8 digit hexadecimal hash",
    )
    assert _require_git_hash({"git_identity": "a" * 8}) == "a" * 8


@pytest.mark.parametrize("value", [True, False, "0", 1])
def test_external_flash_images_are_rejected(value) -> None:
    assert_validation_error(
        lambda: _reject_external_flash({"extf_image_size": value}),
        "unsupportedImage",
        "External-flash APJ images are not supported",
    )
    _reject_external_flash({})
    _reject_external_flash({"extf_image_size": 0})


def test_generated_image_matches_the_descriptor_layout() -> None:
    image = make_image()
    assert len(image) == 52
    assert image[DESCRIPTOR_START : DESCRIPTOR_START + 8] == bytes.fromhex(
        "40a2e4f164689106"
    )
    assert struct.unpack_from("<IIII", image, DESCRIPTOR_START + 8) == (
        0x7152E934,
        0xBFDEED04,
        52,
        0x0123ABCD,
    )
    assert struct.unpack_from("<BBH", image, DESCRIPTOR_START + 24) == (4, 7, 1177)
    _validate_app_descriptor(image, 1177)


@pytest.mark.parametrize(
    ("offset", "detail"),
    [
        (0, CORRUPT),
        (DESCRIPTOR_START - 1, CORRUPT),
        (DESCRIPTOR_START, NO_DESCRIPTOR),
        (DESCRIPTOR_START + 7, NO_DESCRIPTOR),
        (DESCRIPTOR_START + 8, CORRUPT),
        (DESCRIPTOR_START + 11, CORRUPT),
        (DESCRIPTOR_START + 12, CORRUPT),
        (DESCRIPTOR_START + 15, CORRUPT),
        (DESCRIPTOR_START + 16, "Firmware image describes 53 bytes but contains 52"),
        (DESCRIPTOR_START + 24, CORRUPT),
        (DESCRIPTOR_START + 25, CORRUPT),
        (
            DESCRIPTOR_START + 26,
            "Firmware image is built for board ID 1176, not 1177",
        ),
        (DESCRIPTOR_START + 28, CORRUPT),
        (DESCRIPTOR_START + 35, CORRUPT),
        (DESCRIPTOR_START + 36, CORRUPT),
        (51, CORRUPT),
    ],
)
def test_apj_rejects_an_image_with_one_byte_flipped(offset: int, detail: str) -> None:
    image = bytearray(make_image())
    image[offset] ^= 0x01
    assert_validation_error(
        lambda: parse(make_apj(bytes(image))), "invalidDescriptor", detail
    )


def test_git_hash_in_the_descriptor_is_not_covered_by_its_checksums() -> None:
    image = bytearray(make_image())
    for offset in range(DESCRIPTOR_START + 20, DESCRIPTOR_START + 24):
        image[offset] ^= 0xFF
    _validate_app_descriptor(bytes(image), 1177)


@pytest.mark.parametrize(
    "image",
    [
        b"firmware",
        b"x" * 4096,
        APP_DESCRIPTOR_SIGNATURE,
        make_image()[: DESCRIPTOR_START + 35],
        make_image().replace(APP_DESCRIPTOR_SIGNATURE, bytes(8)),
    ],
)
def test_apj_rejects_an_image_without_a_complete_descriptor(image: bytes) -> None:
    assert_validation_error(
        lambda: parse(make_apj(image)), "invalidDescriptor", NO_DESCRIPTOR
    )


def test_apj_accepts_a_descriptor_that_ends_the_image() -> None:
    image = make_image(b"")
    assert len(image) == DESCRIPTOR_START + 36
    assert parse(make_apj(image)).abin.endswith(image)


def test_apj_rejects_a_descriptor_for_another_board() -> None:
    assert_validation_error(
        lambda: parse(make_apj(make_image(board_id=1176))),
        "invalidDescriptor",
        "Firmware image is built for board ID 1176, not 1177",
    )


@pytest.mark.parametrize("described_size", [0, 51, 53])
def test_apj_rejects_a_descriptor_with_another_image_length(
    described_size: int,
) -> None:
    assert_validation_error(
        lambda: parse(make_apj(make_image(described_size=described_size))),
        "invalidDescriptor",
        f"Firmware image describes {described_size} bytes but contains 52",
    )


def test_apj_rejects_bytes_appended_after_the_described_image() -> None:
    assert_validation_error(
        lambda: parse(make_apj(make_image() + b"\x00")),
        "invalidDescriptor",
        "Firmware image describes 52 bytes but contains 53",
    )


def test_only_the_first_descriptor_in_the_image_counts() -> None:
    valid = make_image()
    broken = bytearray(valid)
    broken[-1] ^= 0x01
    assert_validation_error(
        lambda: _validate_app_descriptor(bytes(broken) + valid, 1177),
        "invalidDescriptor",
        "Firmware image describes 52 bytes but contains 104",
    )


def test_descriptor_is_checked_before_the_image_is_packaged(monkeypatch) -> None:
    monkeypatch.setattr(
        hashlib, "md5", lambda *_args, **_kwargs: pytest.fail("image was packaged")
    )
    assert_validation_error(
        lambda: parse(make_apj(b"firmware")), "invalidDescriptor", NO_DESCRIPTOR
    )
