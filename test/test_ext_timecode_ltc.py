"""Tests for timecode labels and the LTC codec."""

import numpy as np
import pytest

from flockwave.server.ext.timecode.ltc import LtcDecoder, LtcEncoder, frame_word
from flockwave.server.ext.timecode.timecode import (
    SUPPORTED_RATES,
    FrameRate,
    Timecode,
)

FPS_25 = FrameRate(25)
DF = FrameRate(30, drop_frame=True)


def test_timecode_labels():
    assert Timecode.parse("01:00:00:00").to_frame_count(FPS_25) == 90_000
    assert Timecode.parse("00:59:30;00") == Timecode(0, 59, 30, 0)
    assert Timecode.parse(" 10:00:00.29\n") == Timecode(10, 0, 0, 29)
    for text in ("garbage", "00:61:00", "1:2:3:4:5"):
        with pytest.raises(ValueError):
            Timecode.parse(text)

    assert not Timecode(0, 0, 0, 25).is_valid_at(FPS_25)
    assert not Timecode(24, 0, 0, 0).is_valid_at(FPS_25)
    assert not Timecode(0, 1, 0, 1).is_valid_at(DF)
    assert Timecode(0, 10, 0, 1).is_valid_at(DF)


def test_drop_frame_labels():
    assert Timecode(1, 0, 0, 0).to_frame_count(DF) == 107_892
    assert Timecode.from_frame_count(1800, DF) == Timecode(0, 1, 0, 2)
    assert Timecode.from_frame_count(17_982, DF) == Timecode(0, 10, 0, 0)
    assert Timecode.from_frame_count(DF.frames_per_day, DF) == Timecode(0, 0, 0, 0)
    for count in range(0, 40_000, 7):
        label = Timecode.from_frame_count(count, DF)
        assert label.is_valid_at(DF)
        assert label.to_frame_count(DF) == count


#: Frames as encoded by libltc 1.3.2 (bytes of its LTCFrame struct), covering
#: the 24 h wrap, the drop-frame minute and the 25 fps polarity bit position.
LIBLTC_FRAMES = [
    (FrameRate(24), Timecode(23, 59, 59, 23), "0302090d09050302fcbf"),
    (FrameRate(24), Timecode(0, 0, 0, 0), "0000000800000000fcbf"),
    (FPS_25, Timecode(1, 2, 3, 5), "0500030002000108fcbf"),
    (FPS_25, Timecode(11, 0, 0, 0), "0000000000000109fcbf"),
    (DF, Timecode(0, 0, 59, 29), "0906090d00000000fcbf"),
    (DF, Timecode(0, 1, 0, 2), "0204000001000000fcbf"),
    (DF, Timecode(0, 10, 0, 0), "0004000800010000fcbf"),
    (FrameRate(30), Timecode(12, 34, 57, 0), "0000070d04030201fcbf"),
]


@pytest.mark.parametrize("rate,timecode,expected", LIBLTC_FRAMES)
def test_frame_bits_match_libltc(rate, timecode, expected):
    assert frame_word(timecode, rate).to_bytes(10, "little").hex() == expected


def _decode_in_blocks(decoder, samples, rng):
    frames, position = [], 0
    while position < len(samples):
        size = int(rng.integers(32, 4096))
        frames += decoder.decode(samples[position : position + size])
        position += size
    return frames


@pytest.mark.parametrize("sample_rate", [44_100, 48_000])
@pytest.mark.parametrize("rate", SUPPORTED_RATES, ids=str)
@pytest.mark.parametrize(
    "gain,offset,noise",
    [(0.5, 0.0, 0.0), (-0.05, 0.2, 0.005)],
    ids=["clean", "impaired"],
)
def test_round_trip(sample_rate, rate, gain, offset, noise):
    """Every frame decodes with its label, and its start is located at the
    sample position the encoder put it; inverted, attenuated, offset and noisy
    (20 dB SNR) signals included."""
    rng = np.random.default_rng(1)
    start = Timecode(0, 59, 59, 0)
    encoder = LtcEncoder(rate, sample_rate, start, start_sample=1000.3, amplitude=1)
    samples = encoder.render(0, int(2.5 * sample_rate))
    samples = gain * samples + offset + rng.normal(0.0, noise, samples.size)

    frames = _decode_in_blocks(LtcDecoder(sample_rate), samples, rng)

    complete = int((samples.size - 1000) / encoder.samples_per_frame)
    assert len(frames) >= complete - 1
    for frame in frames:
        index = round((frame.start_sample - 1000.3) / encoder.samples_per_frame)
        assert frame.timecode == encoder.timecode_of_frame(index)
        assert frame.drop_frame == rate.drop_frame
        if index > 0:
            error = frame.start_sample - encoder.frame_start(index)
            assert abs(error) < (0.25 if noise == 0 else 0.5)


def test_rendering_in_chunks_matches_rendering_at_once():
    encoder = LtcEncoder(DF, 44_100, Timecode(1, 0, 0, 0), start_sample=777.7)
    whole = encoder.render(0, 441 * 50)
    chunked = np.concatenate([encoder.render(i, 441) for i in range(0, 441 * 50, 441)])
    assert np.allclose(whole, chunked, atol=1e-6)
    assert not whole[:777].any()
