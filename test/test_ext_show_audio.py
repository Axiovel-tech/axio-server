import hashlib

import numpy as np
from pytest import approx, mark, raises
from show_audio_support import EPOCH, encode, make_track, run_block

from flockwave.server.ext.show.config import DroneShowConfiguration
from flockwave.server.ext.show_audio.session import STATS_BLOCKS, PlaybackSession
from flockwave.server.ext.show_audio.timing import (
    Phase,
    followed_start_time,
    playback_phase,
    render_block,
)
from flockwave.server.ext.show_audio.track import TrackUpload, decode_track

# Timing rules


def show_config(method="auto", authorized=True, scope="live"):
    config = DroneShowConfiguration()
    config.update_from_json(
        {
            "start": {
                "method": method,
                "authorized": authorized,
                "authorizationScope": scope,
            }
        }
    )
    return config


def test_music_follows_only_starts_the_server_makes_itself():
    assert followed_start_time(show_config(), EPOCH) == EPOCH
    assert followed_start_time(show_config(scope="lights"), EPOCH) == EPOCH
    assert followed_start_time(show_config(scope="rehearsal"), EPOCH) == EPOCH
    assert followed_start_time(show_config(), None) is None
    assert followed_start_time(show_config(method="rc"), EPOCH) is None
    assert followed_start_time(show_config(authorized=False), EPOCH) is None


def test_playback_phase():
    assert playback_phase(94.9, 100, 30, 5) is Phase.WAIT
    assert playback_phase(95.0, 100, 30, 5) is Phase.PLAY
    assert playback_phase(129.0, 100, 30, 5) is Phase.PLAY
    assert playback_phase(130.5, 100, 30, 5) is Phase.OVER


def test_render_block_places_track_after_lead_and_silences_the_rest():
    samples = np.arange(1, 11, dtype=np.float32)[:, None]
    out = np.full((6, 2), np.nan, dtype=np.float32)

    assert render_block(out, samples, 0, 2) == 4
    assert out[:, 0].tolist() == [0, 0, 1, 2, 3, 4]
    assert out[:, 1].tolist() == [0] * 6

    assert render_block(out, samples, 8, 0) == 10
    assert out[:, 0].tolist() == [9, 10, 0, 0, 0, 0]


# Playback session (audio callback)


def test_session_puts_sample_0_on_the_scheduled_frame():
    session = PlaybackSession(make_track(seconds=60), EPOCH, EPOCH + 0.5)

    out = run_block(session, 1000, wall=EPOCH - 1.0)
    assert not session.started
    assert not out.any()

    # Block reaches the DAC at T0 - 0.5 + 0.2 = T0 + 0.5 - 0.3 s
    out = run_block(session, 1000, wall=EPOCH)
    assert out[:300, 0].tolist() == [0] * 300
    assert out[300:, 0].tolist() == list(range(1, 701))
    assert session.joined_at == 0.0

    for i in range(1, STATS_BLOCKS):
        run_block(session, 1000, wall=EPOCH + i)
    assert session.start_error() == approx(0.0, abs=1e-6)


def test_session_joins_late_at_the_matching_position():
    session = PlaybackSession(make_track(), EPOCH, EPOCH)

    out = run_block(session, 100, wall=EPOCH + 3.0)

    assert session.joined_at == approx(3.2)
    assert out[0, 0] == 3201


def test_session_measures_drift_of_the_dac_against_the_wall_clock():
    session = PlaybackSession(make_track(seconds=100), EPOCH, EPOCH)
    run_block(session, 1000, wall=EPOCH - 0.2)

    # The device consumes 1000 frames per 0.999 s of wall-clock time
    for i in range(1, 80):
        run_block(session, 1000, wall=EPOCH - 0.2 + 0.999 * i)

    assert session.start_error() == approx(-0.0155, abs=1e-6)
    assert session.end_drift() == approx(-0.0635, abs=1e-6)


def test_session_counts_underflows_and_silences_on_request():
    session = PlaybackSession(make_track(), EPOCH, EPOCH)
    run_block(session, 100, wall=EPOCH, underflow=True)
    assert session.underflows == 1

    session.silenced = True
    assert not run_block(session, 100, wall=EPOCH + 0.1).any()


def test_session_extra_output_gets_its_channel_and_the_show_time():
    class Extra:
        channel = 2
        lead_s = 10.0
        show_times = []

        def fill(self, out, show_time, sample_rate):
            self.show_times.append(show_time)
            out[:] = -1

    extra = Extra()
    session = PlaybackSession(make_track(), EPOCH, EPOCH + 1, extra=extra)
    assert session.channels == 3

    out = run_block(session, 100, wall=EPOCH - 10)
    assert extra.show_times == [approx(-9.8)]
    assert (out[:, 2] == -1).all()
    assert not out[:, :2].any()


# Tracks and uploads


@mark.parametrize("format", ["WAV", "FLAC", "OGG", "MP3"])
def test_decode_track(format):
    track = decode_track(
        encode(0.5, 44100, format), sha256="x", name="a", media_type="audio/x"
    )
    assert track.sample_rate == 44100
    assert track.channels == 1
    assert track.samples.dtype == np.float32
    assert track.duration == approx(0.5, abs=0.06)


def test_decode_track_rejects_garbage():
    with raises(RuntimeError):
        decode_track(b"not audio" * 100, sha256="x", name="a", media_type="x")


def test_track_upload_accepts_chunks_in_order_and_checks_the_digest():
    data = bytes(range(256)) * 10
    upload = TrackUpload(hashlib.sha256(data).hexdigest().upper(), "a", "x", len(data))

    upload.add(0, data[:1000])
    with raises(ValueError):
        upload.add(500, data[1000:])
    upload.add(1000, data[1000:])

    assert upload.complete
    assert upload.verified_data() == data

    corrupt = TrackUpload("0" * 64, "a", "x", len(data))
    corrupt.add(0, data)
    with raises(ValueError):
        corrupt.verified_data()
