"""Extension that plays the show music in sync with the show clock."""

from __future__ import annotations

import binascii
from base64 import b64decode
from contextlib import ExitStack
from functools import partial
from math import isfinite
from typing import TYPE_CHECKING, Any

import trio

from flockwave.server.ext.base import Extension
from flockwave.server.ext.show.config import StartMethod
from flockwave.server.ext.show.types import ShowExtensionAPI
from flockwave.server.ext.signals import SignalsExtensionAPI

from .ltc_output import LtcOutput
from .player import PlaybackCommittedError, ShowAudioPlayer
from .timing import followed_start_time
from .track import TrackUpload, decode_track

if TYPE_CHECKING:
    from flockwave.server.message_hub import MessageHub
    from flockwave.server.model import Client, FlockwaveMessage

__all__ = ("construct", "dependencies", "description", "schema")

MAX_OFFSET_MS = 10000
"""Largest accepted playback offset magnitude, in milliseconds."""


class ShowAudioExtension(Extension):
    """Extension that receives the show music from Axio Control and plays it
    on an audio output of the server, sample-aligned with the show clock.
    """

    _player: ShowAudioPlayer | None
    _upload: TrackUpload | None
    _last_inf: dict[str, Any] | None
    _warned_about_rc_start: bool

    def __init__(self):
        super().__init__()
        self._player = None
        self._upload = None
        self._last_inf = None
        self._warned_about_rc_start = False

    async def run(self, app, configuration, logger):
        show = app.import_api("show", ShowExtensionAPI)
        signals = app.import_api("signals", SignalsExtensionAPI)

        device = configuration.get("device")
        player = ShowAudioPlayer(
            log=logger,
            device=None if device == "" else device,
            offset_ms=float(configuration.get("offset_ms", 0)),
            arm_lead_s=float(configuration.get("arm_lead_s", 15)),
            point_of_no_return_s=show.get_point_of_no_return_seconds,
        )
        player.on_change = self._broadcast_inf_if_changed

        ltc_config = configuration.get("ltc_output")
        if ltc_config:
            try:
                ltc_output = LtcOutput.from_json(ltc_config)
            except (KeyError, TypeError, ValueError) as ex:
                logger.error(f"Invalid ltc_output configuration, no LTC output: {ex}")
            else:
                player.extra_output = ltc_output
                logger.info(ltc_output.describe())

        handlers = {
            "X-SHOW-AUDIO-CLEAR": self._handle_CLEAR,
            "X-SHOW-AUDIO-INF": self._handle_INF,
            "X-SHOW-AUDIO-SETCFG": self._handle_SETCFG,
            "X-SHOW-AUDIO-STOP": self._handle_STOP,
            "X-SHOW-AUDIO-UPLOAD": self._handle_UPLOAD,
        }

        try:
            self._player = player
            with ExitStack() as stack:
                stack.enter_context(app.message_hub.use_message_handlers(handlers))
                stack.enter_context(
                    signals.use(
                        {
                            "show:clock_changed": self._on_show_changed,
                            "show:config_updated": self._on_show_changed,
                        }
                    )
                )
                self._on_show_changed()
                await player.run()
        finally:
            self._player = None
            self._upload = None
            self._last_inf = None

    def _broadcast_inf_if_changed(self) -> None:
        if self._player is None or self.app is None:
            return

        body = self._inf_body()
        if body == self._last_inf:
            return

        self._last_inf = body
        hub = self.app.message_hub
        hub.enqueue_broadcast_message(hub.create_notification(body))

    def _inf_body(self) -> dict[str, Any]:
        assert self._player is not None
        return {"type": "X-SHOW-AUDIO-INF", **self._player.json}

    def _on_show_changed(self, sender: Any = None, **kwds) -> None:
        if self._player is None or self.app is None:
            return

        show = self.app.import_api("show", ShowExtensionAPI)
        config = show.get_configuration()
        clock = show.get_clock()
        self._player.follow(
            followed_start_time(config, clock.start_time if clock else None)
        )

        needs_auto_start = (
            self._player.track is not None
            and config.authorized_to_start
            and config.start_method is StartMethod.RC
        )
        if needs_auto_start and not self._warned_about_rc_start:
            assert self.log is not None
            self.log.warning(
                "Show music needs the AUTO start method; it does not play when "
                "the show starts with the RC"
            )
        self._warned_about_rc_start = needs_auto_start

    def _handle_CLEAR(self, message: FlockwaveMessage, sender: Client, hub: MessageHub):
        assert self._player is not None
        try:
            self._player.set_track(None)
        except PlaybackCommittedError as ex:
            return hub.reject(message, reason=str(ex))
        self._upload = None
        return hub.acknowledge(message)

    def _handle_INF(self, message: FlockwaveMessage, sender: Client, hub: MessageHub):
        return hub.create_response_or_notification(
            body=self._inf_body(), in_response_to=message
        )

    def _handle_SETCFG(
        self, message: FlockwaveMessage, sender: Client, hub: MessageHub
    ):
        assert self._player is not None
        offset_ms = message.body.get("offsetMs")
        if (
            not isinstance(offset_ms, (int, float))
            or isinstance(offset_ms, bool)
            or not isfinite(offset_ms)
            or abs(offset_ms) > MAX_OFFSET_MS
        ):
            return hub.reject(
                message,
                reason=f"offsetMs must be a number between -{MAX_OFFSET_MS} "
                f"and {MAX_OFFSET_MS}",
            )

        try:
            self._player.set_offset_ms(float(offset_ms))
        except PlaybackCommittedError as ex:
            return hub.reject(message, reason=str(ex))
        return hub.acknowledge(message)

    def _handle_STOP(self, message: FlockwaveMessage, sender: Client, hub: MessageHub):
        assert self._player is not None
        self._player.stop()
        return hub.acknowledge(message)

    async def _handle_UPLOAD(
        self, message: FlockwaveMessage, sender: Client, hub: MessageHub
    ):
        assert self._player is not None
        body = message.body
        try:
            sha256 = _get_typed(body, "sha256", str)
            name = _get_typed(body, "name", str)
            media_type = _get_typed(body, "mediaType", str)
            size = _get_typed(body, "size", int)
            offset = _get_typed(body, "offset", int)
            chunk = b64decode(_get_typed(body, "data", str), validate=True)
        except (ValueError, binascii.Error) as ex:
            return hub.reject(message, reason=f"Invalid upload chunk: {ex}")

        try:
            if offset == 0:
                self._player.ensure_not_committed()
                self._upload = TrackUpload(sha256, name, media_type, size)
            elif self._upload is None or self._upload.sha256 != sha256.lower():
                raise ValueError("No upload of this track in progress")

            upload = self._upload
            upload.add(offset, chunk)
            if not upload.complete:
                return _upload_response(message, hub, "partial")

            self._upload = None
            data = upload.verified_data()
        except (ValueError, PlaybackCommittedError) as ex:
            return hub.reject(message, reason=str(ex))

        try:
            track = await trio.to_thread.run_sync(
                partial(
                    decode_track,
                    data,
                    sha256=upload.sha256,
                    name=upload.name,
                    media_type=upload.media_type,
                )
            )
            self._player.set_track(track)
        except (RuntimeError, PlaybackCommittedError) as ex:
            return hub.reject(message, reason=str(ex))

        assert self.log is not None
        self.log.info(
            f"Show music loaded: {track.name!r}, {track.duration:.1f} s, "
            f"{track.sample_rate} Hz, {track.channels} channel(s)"
        )
        self._on_show_changed()
        return _upload_response(message, hub, "loaded")


def _get_typed(body: dict[str, Any], key: str, kind: type) -> Any:
    value = body.get(key)
    if not isinstance(value, kind) or isinstance(value, bool):
        raise ValueError(f"{key!r} must be of type {kind.__name__}")
    return value


def _upload_response(message: FlockwaveMessage, hub: MessageHub, result: str):
    return hub.create_response_or_notification(
        body={"type": "X-SHOW-AUDIO-UPLOAD", "result": result},
        in_response_to=message,
    )


construct = ShowAudioExtension
dependencies = ("show", "signals")
description = "Plays the show music in sync with the show clock"
schema = {
    "properties": {
        "device": {
            "type": ["string", "integer", "null"],
            "title": "Audio output device",
            "description": (
                "PortAudio output device name (or part of it) or index of the "
                "ALSA host API; empty for the system default output"
            ),
            "default": None,
        },
        "offset_ms": {
            "type": "number",
            "title": "Playback offset (ms)",
            "description": "Positive values play the music later",
            "default": 0,
        },
        "arm_lead_s": {
            "type": "number",
            "title": "Arming lead time (s)",
            "description": (
                "How long before the scheduled first sample the audio output is "
                "opened; above 10 s an output failure shows before the drones "
                "commit to the start"
            ),
            "default": 15,
        },
        "ltc_output": {
            "type": ["object", "null"],
            "title": "LTC output",
            "description": (
                "Writes LTC on an extra channel of the music output, from the "
                "preroll before show T0 to the end of the track; empty for none"
            ),
            "properties": {
                "channel": {
                    "type": "integer",
                    "title": "Output channel",
                    "description": "Channel carrying the LTC, numbered from 1",
                },
                "fps": {
                    "type": "number",
                    "title": "Frame rate",
                    "description": "24, 25, 29.97 (drop-frame) or 30",
                    "default": 25,
                },
                "start": {
                    "type": "string",
                    "title": "Timecode at show T0",
                    "default": "01:00:00:00",
                },
                "preroll_s": {
                    "type": "number",
                    "title": "Preroll (s)",
                    "description": "How long before show T0 the LTC starts",
                    "default": 10,
                },
            },
            "required": ["channel"],
            "default": None,
        },
    }
}
