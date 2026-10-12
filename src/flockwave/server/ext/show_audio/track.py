"""Show music tracks: chunked upload and decoding into memory."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from io import BytesIO
from typing import Any

import numpy as np
import soundfile

__all__ = ("AudioTrack", "TrackUpload", "decode_track")

MAX_CHUNK_SIZE = 256 * 1024
"""Maximum size of one decoded upload chunk, in bytes."""

MAX_TRACK_SIZE = 256 * 1024 * 1024
"""Maximum size of an encoded track, in bytes."""


@dataclass(frozen=True, eq=False)
class AudioTrack:
    """A decoded track, ready to be played."""

    sha256: str
    name: str
    media_type: str
    sample_rate: int
    samples: np.ndarray
    """Float32 samples, frames x channels."""

    @property
    def channels(self) -> int:
        return self.samples.shape[1]

    @property
    def duration(self) -> float:
        return self.samples.shape[0] / self.sample_rate

    @property
    def json(self) -> dict[str, Any]:
        return {
            "sha256": self.sha256,
            "name": self.name,
            "mediaType": self.media_type,
            "duration": self.duration,
        }


def decode_track(data: bytes, *, sha256: str, name: str, media_type: str) -> AudioTrack:
    """Decodes an encoded audio file (MP3, Ogg, FLAC, WAV) into memory.

    Raises:
        RuntimeError: if the data cannot be decoded
    """
    try:
        samples, sample_rate = soundfile.read(
            BytesIO(data), dtype="float32", always_2d=True
        )
    except Exception as ex:
        raise RuntimeError(f"Cannot decode {name!r}: {ex}") from ex

    if samples.shape[0] == 0:
        raise RuntimeError(f"{name!r} contains no audio")

    return AudioTrack(
        sha256=sha256,
        name=name,
        media_type=media_type,
        sample_rate=int(sample_rate),
        samples=np.ascontiguousarray(samples),
    )


class TrackUpload:
    """An encoded track being received in order, in chunks."""

    sha256: str
    name: str
    media_type: str
    size: int

    _data: bytearray

    def __init__(self, sha256: str, name: str, media_type: str, size: int):
        if size <= 0 or size > MAX_TRACK_SIZE:
            raise ValueError(f"Track size must be between 1 and {MAX_TRACK_SIZE} bytes")

        self.sha256 = sha256.lower()
        self.name = name
        self.media_type = media_type
        self.size = size
        self._data = bytearray()

    @property
    def complete(self) -> bool:
        return len(self._data) == self.size

    @property
    def received(self) -> int:
        return len(self._data)

    def add(self, offset: int, chunk: bytes) -> None:
        """Appends the next chunk.

        Raises:
            ValueError: if the chunk is out of order or too large
        """
        if offset != len(self._data):
            raise ValueError(
                f"Expected the chunk at offset {len(self._data)}, got {offset}"
            )
        if len(chunk) > MAX_CHUNK_SIZE:
            raise ValueError(f"Chunks must not exceed {MAX_CHUNK_SIZE} bytes")
        if offset + len(chunk) > self.size:
            raise ValueError("Chunk extends beyond the declared size")
        self._data += chunk

    def verified_data(self) -> bytes:
        """Returns the complete upload after checking its SHA-256 digest.

        Raises:
            ValueError: if the digest does not match
        """
        data = bytes(self._data)
        if hashlib.sha256(data).hexdigest() != self.sha256:
            raise ValueError("SHA-256 digest of the uploaded track does not match")
        return data
