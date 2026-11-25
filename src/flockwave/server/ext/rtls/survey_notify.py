"""How a survey tells its clients and its own waits that something changed."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Optional

import trio

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("NOTIFY_INTERVAL", "Pulse", "StatePusher")

#: the state is pushed at most this often, in seconds; the last change of a
#: burst is pushed when the interval is over
NOTIFY_INTERVAL = 0.5


class Pulse:
    """Wakes every task that waits for the next change."""

    def __init__(self) -> None:
        self._event = trio.Event()

    def set(self) -> None:
        self._event.set()
        self._event = trio.Event()

    async def wait(self) -> None:
        await self._event.wait()


class StatePusher:
    """Broadcasts a state to the clients whenever it is marked as changed.

    A task of its own delivers it, so that an outbound queue without room
    holds up neither a survey run, which has anchors to return, nor an
    answer: only that task waits for room.
    """

    def __init__(self, ext: "RtlsExtension", body: Callable[[], dict[str, Any]]):
        self._ext = ext
        self._body = body
        self._dirty = False
        self._pushing = False
        self._pushed_at: Optional[float] = None

    def notify(self) -> None:
        """Marks the state as changed."""
        self._dirty = True
        nursery = self._ext._nursery
        if nursery is not None and not self._pushing:
            self._pushing = True
            nursery.start_soon(self._push)

    async def _push(self) -> None:
        """Pushes the state while it has changed since the last push, at
        most twice a second: the last change of a burst goes out when the
        half second is over."""
        try:
            while self._dirty:
                if self._pushed_at is not None:
                    await trio.sleep_until(self._pushed_at + NOTIFY_INTERVAL)
                self._dirty, self._pushed_at = False, trio.current_time()
                if self._ext.app is None:
                    self._ext._warn_no_app()
                    continue
                hub = self._ext.app.message_hub
                await hub.broadcast_message(hub.create_notification(self._body()))
        except Exception as ex:
            if self._ext.log:
                self._ext.log.warning(f"rtls: survey notification not sent: {ex!r}")
        finally:
            self._pushing = False
