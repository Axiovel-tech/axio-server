"""Periodic status around one vehicle's collection."""

from __future__ import annotations

from collections.abc import AsyncGenerator
from contextlib import ExitStack

import trio

from .collect import CollectRequest, Refused, collect, exclusive
from .download import write_json
from .status import Status
from .transport import FCTransport

__all__ = ("collect_with_status",)


@trio.as_safe_channel
async def collect_with_status(
    transport: FCTransport, request: CollectRequest, uav: str, interval: float
) -> AsyncGenerator[tuple[str, dict[str, object]], None]:
    """Yields ("status", snapshot) every interval, then ("receipt", receipt).

    The vehicle's status.json always holds the latest snapshot.
    """
    with ExitStack() as stack:
        try:
            stack.enter_context(exclusive(request.directory, transport.system_id))
        except Refused as error:
            yield "receipt", {"uav": uav, "complete": False, "error": str(error)}
            return
        status = Status(uav)
        status_path = request.directory / "status.json"
        receipt: dict[str, object] = {}
        finished = trio.Event()

        async def run() -> None:
            try:
                receipt.update(await collect(transport, request, status))
            finally:
                finished.set()

        async with trio.open_nursery() as nursery:
            nursery.start_soon(run)
            while not finished.is_set():
                with trio.move_on_after(interval):
                    await finished.wait()
                snapshot = status.snapshot()
                write_json(status_path, snapshot)
                if not finished.is_set():
                    yield "status", snapshot
        yield "receipt", receipt
