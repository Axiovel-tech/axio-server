"""Tests for Socket.IO transport limits needed by bounded firmware uploads."""

from contextlib import nullcontext
from types import SimpleNamespace
from typing import cast

import pytest

from flockwave.server.app import SkybrushServer
from flockwave.server.ext.socketio.extension import (
    MAX_SOCKETIO_MESSAGE_SIZE,
    SOCKETIO_V4_MAX_MESSAGE_SIZE,
    SocketIOCommunicationHandler,
    SocketIOProtocol,
)


@pytest.mark.parametrize(
    ("protocol", "expected_limit"),
    [
        (SocketIOProtocol.SOCKETIO_V4, SOCKETIO_V4_MAX_MESSAGE_SIZE),
        (SocketIOProtocol.SOCKETIO_V5, MAX_SOCKETIO_MESSAGE_SIZE),
    ],
)
def test_socketio_protocols_preserve_their_payload_limits(
    protocol, expected_limit: int
) -> None:
    registry = SimpleNamespace(use=lambda *args, **kwargs: nullcontext())
    app = cast(SkybrushServer, SimpleNamespace(channel_type_registry=registry))

    with SocketIOCommunicationHandler(app, protocol).use() as server:
        assert server.eio.max_http_buffer_size == expected_limit
        assert SOCKETIO_V4_MAX_MESSAGE_SIZE == 100_000_000
        assert MAX_SOCKETIO_MESSAGE_SIZE == 5 * 1024 * 1024
