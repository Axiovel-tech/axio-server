"""Tests for Socket.IO transport limits needed by bounded firmware uploads."""

from contextlib import nullcontext
from types import SimpleNamespace
from typing import cast

import pytest

from flockwave.server.app import SkybrushServer
from flockwave.server.ext.socketio.extension import (
    MAX_SOCKETIO_MESSAGE_SIZE,
    SocketIOCommunicationHandler,
    SocketIOProtocol,
)


@pytest.mark.parametrize("protocol", list(SocketIOProtocol))
def test_all_socketio_protocols_accept_bounded_apj_messages(protocol) -> None:
    registry = SimpleNamespace(use=lambda *args, **kwargs: nullcontext())
    app = cast(SkybrushServer, SimpleNamespace(channel_type_registry=registry))

    with SocketIOCommunicationHandler(app, protocol).use() as server:
        assert server.eio.max_http_buffer_size == MAX_SOCKETIO_MESSAGE_SIZE
        assert MAX_SOCKETIO_MESSAGE_SIZE == 5 * 1024 * 1024
