import hashlib
from base64 import b64encode
from types import SimpleNamespace

import trio
from pytest import approx, fixture, raises
from show_audio_support import EPOCH, encode, log, make_track, run_block

from flockwave.server.ext.show_audio.extension import ShowAudioExtension
from flockwave.server.ext.show_audio.player import (
    PlaybackCommittedError,
    PlayerState,
    ShowAudioPlayer,
)

# Player (scheduling around the show clock)


class FakeOutput:
    def __init__(self):
        self.streams = []
        self.failures = 0

    async def open(self, session, device):
        if self.failures:
            self.failures -= 1
            raise RuntimeError("device busy")
        stream = SimpleNamespace(
            session=session,
            closed=False,
            description="fake",
            calibrate=lambda: None,
        )

        async def aclose():
            stream.closed = True

        stream.aclose = aclose
        self.streams.append(stream)
        return stream

    @property
    def open_streams(self):
        return [stream for stream in self.streams if not stream.closed]


def wall_clock():
    return EPOCH + trio.current_time()


@fixture
def output():
    return FakeOutput()


@fixture
async def player(output, nursery):
    player = ShowAudioPlayer(
        log=log,
        arm_lead_s=5,
        point_of_no_return_s=lambda: -10,
        open_stream=output.open,
        clock=wall_clock,
    )
    player.set_track(make_track(seconds=30))
    nursery.start_soon(player.run)
    return player


async def sleep_until(seconds: float) -> None:
    await trio.sleep_until(seconds)


async def test_player_arms_ahead_and_follows_reschedules_before_commit(
    player, output, autojump_clock
):
    player.follow(EPOCH + 60)

    await sleep_until(40)
    assert output.streams == []
    player.follow(EPOCH + 70)

    await sleep_until(64.9)
    assert output.streams == []
    assert player.state is PlayerState.IDLE

    await sleep_until(65.1)
    [stream] = output.open_streams
    assert stream.session.sample0_time == EPOCH + 70
    assert player.state is PlayerState.ARMED
    assert player.json["startTime"] == EPOCH + 70


async def test_player_moves_an_armed_start_and_cancels_before_commit(
    output, player, autojump_clock
):
    player._arm_lead_s = 15
    player.follow(EPOCH + 60)

    await sleep_until(46)
    [stream] = output.open_streams
    player.follow(EPOCH + 61)
    await sleep_until(47)
    assert output.open_streams == [stream]
    assert stream.session.sample0_time == EPOCH + 61

    player.follow(None)
    await sleep_until(48)
    assert output.open_streams == []
    assert player.state is PlayerState.IDLE


async def test_player_is_committed_after_the_point_of_no_return(
    output, player, autojump_clock
):
    player.follow(EPOCH + 60)

    await sleep_until(50.5)
    player.follow(None)
    with raises(PlaybackCommittedError):
        player.set_track(None)
    with raises(PlaybackCommittedError):
        player.set_offset_ms(100)

    await sleep_until(55.5)
    [stream] = output.open_streams
    assert stream.session.sample0_time == EPOCH + 60
    assert player.json["startTime"] == EPOCH + 60

    player.stop()
    await sleep_until(56)
    assert output.open_streams == []
    assert player.state is PlayerState.IDLE
    player.set_track(None)


async def test_player_joins_late_and_stays_stopped_until_rescheduled(
    output, player, autojump_clock
):
    player.follow(EPOCH - 3)
    await sleep_until(0.05)

    [stream] = output.open_streams
    run_block(stream.session, 100, wall=wall_clock())
    assert stream.session.joined_at == approx(3.25, abs=0.01)
    await sleep_until(0.2)
    assert player.state is PlayerState.PLAYING

    player.stop()
    await sleep_until(1)
    assert output.open_streams == []

    player.follow(None)
    player.follow(EPOCH - 3)
    await sleep_until(1.1)
    assert len(output.open_streams) == 1


async def test_player_stop_holds_until_the_start_is_cleared_or_moves_by_1_s(
    output, player, autojump_clock
):
    player.follow(EPOCH - 3)
    await sleep_until(0.05)
    assert len(output.open_streams) == 1

    player.stop()
    await sleep_until(0.2)
    assert output.open_streams == []

    player.follow(EPOCH - 3 + 0.004)
    player.follow(EPOCH - 3 - 0.9)
    await sleep_until(0.4)
    assert output.open_streams == []
    assert player.state is PlayerState.IDLE

    player.follow(EPOCH - 3 + 1.5)
    await sleep_until(0.6)
    assert len(output.open_streams) == 1


async def test_player_finishes_the_track(output, player, autojump_clock):
    player.set_track(make_track(seconds=2))
    player.follow(EPOCH + 10)

    await sleep_until(10.1)
    [stream] = output.open_streams
    run_block(stream.session, 100, wall=wall_clock())
    assert stream.session.started

    await sleep_until(13)
    assert output.open_streams == []
    assert player.state is PlayerState.FINISHED

    player.follow(None)
    await sleep_until(13.1)
    assert player.state is PlayerState.IDLE


async def test_player_reports_output_errors_and_retries(output, player, autojump_clock):
    output.failures = 1
    player.follow(EPOCH + 10)

    await sleep_until(5.5)
    assert player.state is PlayerState.ERROR
    assert "device busy" in player.json["error"]

    await sleep_until(6.5)
    assert player.state is PlayerState.ARMED
    assert player.json["error"] is None


# Message handlers


class FakeHub:
    def acknowledge(self, message, outcome=True, reason=None):
        return {"type": "ACK-ACK"}

    def reject(self, message, reason=None):
        return {"type": "ACK-NAK", "reason": reason}

    def create_response_or_notification(self, body, in_response_to=None):
        return body


@fixture
def extension(output):
    extension = ShowAudioExtension()
    extension.log = log
    extension._player = ShowAudioPlayer(
        log=log, open_stream=output.open, clock=lambda: EPOCH
    )
    return extension


def message(**body):
    return SimpleNamespace(body=body)


async def upload(extension, data: bytes, chunk_size: int, **overrides):
    sha256 = hashlib.sha256(data).hexdigest()
    responses = []
    for offset in range(0, len(data), chunk_size):
        body = {
            "sha256": sha256,
            "name": "show.flac",
            "mediaType": "audio/flac",
            "size": len(data),
            "offset": offset,
            "data": b64encode(data[offset : offset + chunk_size]).decode("ascii"),
        }
        body.update(overrides)
        responses.append(
            await extension._handle_UPLOAD(message(**body), None, FakeHub())
        )
    return responses


async def test_upload_inf_clear(extension):
    hub = FakeHub()
    data = encode(0.5, 8000, "FLAC")

    responses = await upload(extension, data, 1024)
    assert [r.get("result") for r in responses[:-1]] == ["partial"] * (
        len(responses) - 1
    )
    assert responses[-1] == {"type": "X-SHOW-AUDIO-UPLOAD", "result": "loaded"}

    inf = extension._handle_INF(message(), None, hub)
    assert inf["type"] == "X-SHOW-AUDIO-INF"
    assert inf["track"] == {
        "sha256": hashlib.sha256(data).hexdigest(),
        "name": "show.flac",
        "mediaType": "audio/flac",
        "duration": approx(0.5),
    }
    assert inf["state"] == "idle"
    assert inf["startTime"] is None
    assert inf["offsetMs"] == 0

    assert extension._handle_CLEAR(message(), None, hub)["type"] == "ACK-ACK"
    assert extension._handle_INF(message(), None, hub)["track"] is None


async def test_upload_rejects_bad_chunks_and_keeps_the_current_track(extension):
    good = encode(0.5, 8000, "WAV")
    await upload(extension, good, 4096)
    loaded = extension._player.track

    responses = await upload(extension, good, 4096, sha256="0" * 64)
    assert responses[-1]["type"] == "ACK-NAK"

    responses = await upload(extension, b"garbage" * 1000, 4096)
    assert responses[-1]["type"] == "ACK-NAK"

    response = await extension._handle_UPLOAD(
        message(sha256="0" * 64, name="a", mediaType="x", size=10, offset=5, data=""),
        None,
        FakeHub(),
    )
    assert response["type"] == "ACK-NAK"

    assert extension._player.track is loaded


async def test_setcfg_and_stop(extension):
    hub = FakeHub()

    assert (
        extension._handle_SETCFG(message(offsetMs=-120), None, hub)["type"] == "ACK-ACK"
    )
    assert extension._handle_INF(message(), None, hub)["offsetMs"] == -120

    for bad in (None, "5", True, 20000, float("nan")):
        assert (
            extension._handle_SETCFG(message(offsetMs=bad), None, hub)["type"]
            == "ACK-NAK"
        )

    assert extension._handle_STOP(message(), None, hub)["type"] == "ACK-ACK"


async def test_changes_are_rejected_while_committed(extension):
    hub = FakeHub()
    await upload(extension, encode(0.5, 8000, "WAV"), 65536)
    extension._player.follow(EPOCH + 5)

    assert extension._handle_CLEAR(message(), None, hub)["type"] == "ACK-NAK"
    assert (
        extension._handle_SETCFG(message(offsetMs=10), None, hub)["type"] == "ACK-NAK"
    )
    responses = await upload(extension, encode(0.25, 8000, "WAV"), 65536)
    assert responses[0]["type"] == "ACK-NAK"
    assert "committed" in responses[0]["reason"]
