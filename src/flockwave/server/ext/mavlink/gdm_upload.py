"""Ground side of the `.gdm` show upload (`OBJ-CMD __gdm_upload`).

Axio Control sends each UAV its precompiled, certified `.gdmb` together with
the show hash of the `.gdm` it came from. Axio Server never re-encodes the
payload (ADR 0002 decision D2): it validates it against AxioStack
`contracts/gdm-format.md` §7.1, then writes it to the vehicle with an atomic
replace into the drone's show slot (`show_slot.py`).
"""

from __future__ import annotations

from base64 import b64decode
from binascii import Error as BinasciiError

from flockwave.server.show.gdmb import (
    GDMB_MAX_SIZE,
    GdmbSummary,
    GdmbValidationError,
    validate_gdmb,
)

__all__ = ("decode_gdm_upload_request",)

_MAX_BASE64_LENGTH = 4 * ((GDMB_MAX_SIZE + 2) // 3)
"""Length of the longest base64 string that can decode to a valid `.gdmb`."""


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
