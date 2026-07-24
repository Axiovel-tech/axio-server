"""Regression test: the message hub's outbound pump must stay bounded when
a client cannot keep up.

Before the fix, ``MessageHub.run()`` drained its 4096-slot queue into an
unbounded number of suspended ``_broadcast_message`` tasks whenever one
connected client applied backpressure, so the ``enqueue_broadcast_message``
drop path could never engage and a high-rate telemetry stream (e.g. the
~160 Hz X-RTLS-POS broadcasts of a 16-tag fleet) grew the process by
hundreds of KB/s until the OOM killer fired.
"""

import logging

import trio
import trio.testing

from flockwave.server.message_hub import MessageHub
from flockwave.server.model.messages import FlockwaveNotification


def _notification() -> FlockwaveNotification:
    hub = MessageHub()
    return hub._message_builder.create_notification({"type": "X-TEST"})


async def test_slow_broadcast_saturates_queue_instead_of_spawning_tasks(
    autojump_clock, caplog
):
    hub = MessageHub()

    in_flight = 0
    peak_in_flight = 0
    release = trio.Event()

    async def stalled_broadcast(message, notify_sent=None):
        nonlocal in_flight, peak_in_flight
        in_flight += 1
        peak_in_flight = max(peak_in_flight, in_flight)
        await release.wait()
        in_flight -= 1

    # stand in for a broadcast whose client channel never accepts the write
    hub._broadcast_message = stalled_broadcast  # type: ignore[method-assign]

    total = 4096 + MessageHub.MAX_CONCURRENT_SENDS + 500
    async with trio.open_nursery() as nursery:
        nursery.start_soon(hub.run)
        await trio.testing.wait_all_tasks_blocked()

        notification = _notification()
        with caplog.at_level(logging.WARNING):
            for _ in range(total):
                hub.enqueue_broadcast_message(notification)
        await trio.testing.wait_all_tasks_blocked()

        # in-flight sends are capped, the rest waits in the bounded queue...
        assert peak_in_flight == MessageHub.MAX_CONCURRENT_SENDS
        # ...and the overflow was dropped, not accumulated
        assert any(
            "dropping broadcast message" in record.message
            for record in caplog.records
        )

        # once the client drains, the backlog flows again without loss of
        # service and the pump keeps honoring the cap
        release.set()
        await trio.testing.wait_all_tasks_blocked()
        assert in_flight == 0
        assert peak_in_flight == MessageHub.MAX_CONCURRENT_SENDS

        nursery.cancel_scope.cancel()
