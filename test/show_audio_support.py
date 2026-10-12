"""Shared helpers of the show_audio tests."""

from io import BytesIO
from logging import getLogger
from types import SimpleNamespace

import numpy as np
import soundfile

from flockwave.server.ext.show_audio.track import AudioTrack

log = getLogger("test_ext_show_audio")

EPOCH = 1_760_000_000.0


def make_track(seconds: float = 30.0, sample_rate: int = 1000, channels: int = 1):
    frames = int(seconds * sample_rate)
    samples = np.repeat(
        np.arange(1, frames + 1, dtype=np.float32)[:, None], channels, axis=1
    )
    return AudioTrack(
        sha256="0" * 64,
        name="track.wav",
        media_type="audio/wav",
        sample_rate=sample_rate,
        samples=samples,
    )


def encode(seconds: float, sample_rate: int, format: str) -> bytes:
    buffer = BytesIO()
    t = np.arange(int(seconds * sample_rate)) / sample_rate
    soundfile.write(
        buffer, 0.5 * np.sin(2 * np.pi * 440 * t), sample_rate, format=format
    )
    return buffer.getvalue()


def run_block(session, frames, *, wall, latency=0.2, underflow=False):
    out = np.full((frames, session.channels), np.nan, dtype=np.float32)
    session._clock = lambda: wall
    session.callback(
        out,
        frames,
        SimpleNamespace(currentTime=50.0, outputBufferDacTime=50.0 + latency),
        SimpleNamespace(output_underflow=underflow),
    )
    return out
