"""MAVLink access to one flight controller, through the server or directly."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import AsyncIterator, Iterator
from contextlib import AbstractContextManager, asynccontextmanager, contextmanager
from functools import partial
from pathlib import Path
from typing import Any, Protocol

import trio
from trio.abc import ReceiveChannel

__all__ = ("DirectTransport", "FCTransport", "MAVLinkMessage")

MAVLinkMessage = Any
"""A decoded pymavlink message."""

AUTOPILOT_COMPONENT = 1

_SUBSCRIBER_CAPACITY = 2048
"""Queued messages per subscriber; more than four full FTP bursts."""


class FCTransport(Protocol):
    """Sends to and receives from the autopilot component of one system."""

    @property
    def system_id(self) -> int: ...

    @property
    def description(self) -> dict[str, object]: ...

    async def send(self, message_type: str, **fields: Any) -> None:
        """Sends a message; the transport fills in the target system and component."""
        ...

    def subscribe(
        self, message_type: str
    ) -> AbstractContextManager[ReceiveChannel[MAVLinkMessage]]:
        """Delivers every message of this type from the autopilot.

        Messages are dropped while the receiver is more than the channel
        capacity behind.
        """
        ...


def serial_identity(connection: str) -> dict[str, str]:
    """Stable USB identity of a serial device; empty for network connections."""
    device = Path(connection.split(",")[0])
    if not device.exists():
        return {}
    real = device.resolve()
    by_id = Path("/dev/serial/by-id")
    aliases = (
        sorted(link.name for link in by_id.iterdir() if link.resolve() == real)
        if by_id.is_dir()
        else []
    )
    return {"device": str(real), **({"usb_id": aliases[0]} if aliases else {})}


class DirectTransport:
    """A pymavlink connection owned by this process (USB, serial, TCP, UDP)."""

    def __init__(self, connection: Any, system_id: int, description: dict[str, object]):
        self._connection = connection
        self._system_id = system_id
        self._description = description
        self.heard_systems: set[int] = set()
        self._subscribers: defaultdict[str, list[trio.MemorySendChannel]] = defaultdict(
            list
        )

    @classmethod
    @asynccontextmanager
    async def open(
        cls, address: str, system_id: int, *, baud: int = 115200
    ) -> AsyncIterator[DirectTransport]:
        from pymavlink import mavutil

        connection = await trio.to_thread.run_sync(
            partial(
                mavutil.mavlink_connection,
                address,
                baud=baud,
                source_system=253,
                source_component=190,
                autoreconnect=False,
            )
        )
        description: dict[str, object] = {
            "kind": "direct",
            "connection": address,
            **serial_identity(address),
        }
        transport = cls(connection, system_id, description)
        # Shielded so that replies keep arriving while a cancelled collection
        # restores parameters; it stops when the transport closes.
        receiving = trio.CancelScope(shield=True)
        try:
            async with trio.open_nursery() as nursery:
                nursery.start_soon(transport._pump, receiving)
                try:
                    yield transport
                finally:
                    receiving.cancel()
        finally:
            connection.close()

    @property
    def system_id(self) -> int:
        return self._system_id

    @property
    def description(self) -> dict[str, object]:
        return self._description

    async def send(self, message_type: str, **fields: Any) -> None:
        sender = getattr(self._connection.mav, f"{message_type.lower()}_send")
        sender(
            target_system=self._system_id,
            target_component=AUTOPILOT_COMPONENT,
            **fields,
        )

    @contextmanager
    def subscribe(self, message_type: str) -> Iterator[ReceiveChannel[MAVLinkMessage]]:
        sender, receiver = trio.open_memory_channel[MAVLinkMessage](
            _SUBSCRIBER_CAPACITY
        )
        subscribers = self._subscribers[message_type]
        subscribers.append(sender)
        try:
            with receiver:
                yield receiver
        finally:
            subscribers.remove(sender)
            sender.close()

    async def _pump(self, scope: trio.CancelScope) -> None:
        with scope:
            while True:
                for message in await trio.to_thread.run_sync(self._read_batch):
                    self._dispatch(message)

    def _read_batch(self) -> list[MAVLinkMessage]:
        batch = []
        message = self._connection.recv_match(blocking=True, timeout=0.1)
        while message is not None:
            batch.append(message)
            if len(batch) == 256:
                break
            message = self._connection.recv_match(blocking=False)
        return batch

    def _dispatch(self, message: MAVLinkMessage) -> None:
        if message.get_srcComponent() != AUTOPILOT_COMPONENT:
            return
        self.heard_systems.add(message.get_srcSystem())
        if message.get_srcSystem() != self._system_id:
            return
        for subscriber in self._subscribers.get(message.get_type(), ()):
            try:
                subscriber.send_nowait(message)
            except trio.WouldBlock:
                pass
