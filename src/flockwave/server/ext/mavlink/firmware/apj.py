"""Bounded parser for ArduPilot APJ firmware containers."""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import re
import zlib
from dataclasses import dataclass
from hmac import compare_digest
from typing import Any

MAX_APJ_SIZE = 3 * 1024 * 1024
MAX_IMAGE_SIZE_BY_BOARD = {1177: 1_703_936}

_GIT_HASH_PATTERN = re.compile(r"[0-9a-fA-F]{8}")


class APJValidationError(ValueError):
    """Raised when an APJ container is malformed or incompatible."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


@dataclass(frozen=True)
class FirmwareImage:
    """Validated firmware metadata and its bootloader-ready payload."""

    abin: bytes
    board_id: int
    git_hash: str
    version: str

    @property
    def total_size(self) -> int:
        return len(self.abin)


def parse_apj(
    payload: bytes,
    *,
    expected_sha256: str,
    name: str,
) -> FirmwareImage:
    """Parse and validate an APJ file without unbounded decompression."""
    _validate_envelope(payload, expected_sha256, name)
    document = _decode_document(payload)
    board_id = _require_int(document, "board_id")
    max_size = MAX_IMAGE_SIZE_BY_BOARD.get(board_id)
    if max_size is None:
        raise APJValidationError(
            "unsupportedBoard", f"Unsupported ArduPilot board ID: {board_id}"
        )

    image = _decode_image(document, max_size=max_size)
    declared_size = _require_int(document, "image_size")
    if declared_size != len(image):
        raise APJValidationError(
            "sizeMismatch",
            f"APJ declares {declared_size} image bytes but contains {len(image)}",
        )
    _reject_external_flash(document)

    git_hash = _require_git_hash(document)
    version = document.get("version")
    if not isinstance(version, str) or not version or len(version) > 64:
        raise APJValidationError(
            "invalidMetadata", "version must contain 1 to 64 characters"
        )

    image_md5 = hashlib.md5(image, usedforsecurity=False).hexdigest()
    header = f"git version: {git_hash}\nMD5: {image_md5}\n--\n".encode()
    return FirmwareImage(
        abin=header + image,
        board_id=board_id,
        git_hash=git_hash.lower(),
        version=version,
    )


def _validate_envelope(payload: bytes, expected_sha256: str, name: str) -> None:
    if not payload or len(payload) > MAX_APJ_SIZE:
        raise APJValidationError(
            "invalidSize", f"APJ file must contain 1 to {MAX_APJ_SIZE} bytes"
        )
    if not isinstance(name, str) or not name or len(name) > 128:
        raise APJValidationError(
            "invalidName", "Firmware name must contain 1 to 128 characters"
        )
    if not name.lower().endswith(".apj"):
        raise APJValidationError("invalidName", "Firmware name must end in .apj")
    if not isinstance(expected_sha256, str) or not re.fullmatch(
        r"[0-9a-f]{64}", expected_sha256
    ):
        raise APJValidationError(
            "invalidHash", "sha256 must be 64 lowercase hexadecimal characters"
        )
    observed = hashlib.sha256(payload).hexdigest()
    if not compare_digest(observed, expected_sha256):
        raise APJValidationError(
            "hashMismatch", "APJ SHA-256 does not match the request"
        )


def _decode_document(payload: bytes) -> dict[str, Any]:
    try:
        document = json.loads(payload.decode())
    except (UnicodeDecodeError, json.JSONDecodeError) as ex:
        raise APJValidationError(
            "invalidContainer", "APJ is not valid UTF-8 JSON"
        ) from ex
    if not isinstance(document, dict) or document.get("magic") != "APJFWv1":
        raise APJValidationError("invalidContainer", "APJ magic must be APJFWv1")
    return document


def _decode_image(document: dict[str, Any], *, max_size: int) -> bytes:
    encoded = document.get("image")
    if not isinstance(encoded, str):
        raise APJValidationError("invalidImage", "APJ image must be a base64 string")
    try:
        compressed = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error) as ex:
        raise APJValidationError(
            "invalidImage", "APJ image is not valid base64"
        ) from ex

    inflater = zlib.decompressobj()
    try:
        image = inflater.decompress(compressed, max_size + 1)
    except zlib.error as ex:
        raise APJValidationError(
            "invalidImage", "APJ image is not valid zlib data"
        ) from ex
    if len(image) > max_size or inflater.unconsumed_tail:
        raise APJValidationError(
            "imageTooLarge", f"Firmware image exceeds the {max_size}-byte board limit"
        )
    image += inflater.flush(max_size + 1 - len(image))
    if not inflater.eof or inflater.unused_data:
        raise APJValidationError(
            "invalidImage", "APJ image has incomplete or trailing zlib data"
        )
    if not image:
        raise APJValidationError("invalidImage", "APJ firmware image is empty")
    return image


def _require_int(document: dict[str, Any], key: str) -> int:
    value = document.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise APJValidationError(
            "invalidMetadata", f"{key} must be a non-negative integer"
        )
    return value


def _require_git_hash(document: dict[str, Any]) -> str:
    value = document.get("git_identity")
    if not isinstance(value, str) or _GIT_HASH_PATTERN.fullmatch(value) is None:
        raise APJValidationError(
            "invalidMetadata", "git_identity must be an 8 digit hexadecimal hash"
        )
    return value


def _reject_external_flash(document: dict[str, Any]) -> None:
    size = document.get("extf_image_size", 0)
    if not isinstance(size, int) or isinstance(size, bool) or size != 0:
        raise APJValidationError(
            "unsupportedImage", "External-flash APJ images are not supported"
        )
