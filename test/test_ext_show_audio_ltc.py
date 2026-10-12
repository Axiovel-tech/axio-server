import numpy as np
from pytest import approx, raises

from flockwave.server.ext.show_audio.ltc_output import LtcOutput
from flockwave.server.ext.timecode.ltc import LtcDecoder
from flockwave.server.ext.timecode.timecode import FrameRate, Timecode

SAMPLE_RATE = 48000
BLOCK = 2400


def ltc_output(**config):
    return LtcOutput.from_json({"channel": 3, **config})


def render(output, show_times):
    """Renders consecutive blocks; `show_times(first_sample)` is the show time
    the session reports for the block starting at that stream sample."""
    writer = output.create_writer(SAMPLE_RATE)
    stream = np.zeros(len(show_times) * BLOCK, dtype=np.float32)
    for index, show_time in enumerate(show_times):
        writer(stream[index * BLOCK : (index + 1) * BLOCK], show_time)
    return stream


def blocks(count, t0_sample):
    return [(i * BLOCK - t0_sample) / SAMPLE_RATE for i in range(count)]


def decode(stream):
    decoder = LtcDecoder(SAMPLE_RATE)
    frames = []
    for first in range(0, len(stream), BLOCK):
        frames += decoder.decode(stream[first : first + BLOCK])
    return {str(frame.timecode): frame.start_sample for frame in frames}


def test_ltc_output_configuration():
    output = ltc_output()
    assert output.channel == 2
    assert output.rate == FrameRate(25)
    assert output.start == Timecode(1, 0, 0, 0)
    assert output.preroll_s == 10
    assert output.lead_s == 11

    with raises(ValueError):
        ltc_output(channel=0)
    with raises(ValueError):
        ltc_output(start="01:00:00:25")
    with raises(ValueError):
        ltc_output(fps=50)


def test_ltc_frame_at_t0_starts_on_the_exact_sample():
    t0_sample = 12 * SAMPLE_RATE + 0.3
    stream = render(ltc_output(), blocks(260, t0_sample))

    starts = decode(stream)
    assert starts["01:00:00:00"] == approx(t0_sample, abs=0.1)
    assert starts["01:00:00:01"] == approx(t0_sample + 1920, abs=0.1)
    assert starts["00:59:59:24"] == approx(t0_sample - 1920, abs=0.1)

    preroll_start = round(t0_sample - 10 * SAMPLE_RATE)
    assert not stream[: preroll_start - 1].any()
    assert stream[preroll_start + 1 : preroll_start + 20].any()


def test_ltc_is_anchored_on_the_settled_dac_timestamps():
    t0_sample = 12 * SAMPLE_RATE
    show_times = blocks(260, t0_sample)
    show_times[0] += 0.0002
    for index in range(1, len(show_times)):
        show_times[index] += 0.00002 if index % 2 else -0.00002

    starts = decode(render(ltc_output(), show_times))

    assert starts["01:00:00:00"] == approx(t0_sample, abs=1)


def test_ltc_joins_late_with_the_matching_timecode():
    stream = render(
        ltc_output(fps=30, start="10:00:00:00"), blocks(40, -4.99 * SAMPLE_RATE)
    )

    starts = decode(stream)
    assert starts["10:00:05:00"] == approx(0.01 * SAMPLE_RATE, abs=0.1)


def test_ltc_follows_a_moved_start_before_t0_but_never_jumps_after():
    t0_sample = 2 * SAMPLE_RATE
    moved = t0_sample + 0.05 * SAMPLE_RATE
    drifted = moved + 0.003 * SAMPLE_RATE
    show_times = (
        blocks(100, t0_sample)[:20]
        + blocks(100, moved)[20:50]
        + blocks(100, drifted)[50:]
    )

    starts = decode(render(ltc_output(preroll_s=1), show_times))

    assert starts["01:00:00:00"] == approx(moved, abs=0.1)
    assert starts["01:00:02:00"] == approx(moved + 2 * SAMPLE_RATE, abs=0.1)
