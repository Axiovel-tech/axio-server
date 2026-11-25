"""Tests for downloading logs with LOG_REQUEST_DATA when LOG_DATA messages get lost."""

from base64 import b64decode
from types import SimpleNamespace

import trio

from flockwave.server.ext.mavlink.log_download import (
    LOG_DATA_GAP_TIMEOUT,
    MAVLinkLogDownloader,
)
from flockwave.server.model.log import FlightLog

LOG_ID = 1
LOG_SIZE = 90 * 1200  # three requests of at most 512 messages
FRAME_INTERVAL = 0.001


class FakeAutopilot:
    """Answers LOG_REQUEST_DATA like ArduPilot: sends the requested range in order,
    one LOG_DATA at a time, and drops a range it is sending when a new request
    arrives. Messages at the offsets in `lose_once` are lost the first time they
    are sent."""

    def __init__(self, nursery: trio.Nursery, lose_once: set[int]):
        self.log = bytes(i % 251 for i in range(LOG_SIZE))
        self.lose_once = set(lose_once)
        self.requests: list[tuple[int, int]] = []
        self.downloader = MAVLinkLogDownloader(self.send)
        self._nursery = nursery
        self._stream: trio.CancelScope | None = None

    def _log_data(self, ofs: int) -> SimpleNamespace:
        data = self.log[ofs : ofs + 90]
        return SimpleNamespace(
            get_type=lambda: "LOG_DATA",
            id=LOG_ID,
            ofs=ofs,
            count=len(data),
            data=data.ljust(90, b"\0"),
        )

    async def send(self, message, wait_for_response=None):
        name, fields = message
        if name == "LOG_REQUEST_LIST":
            return SimpleNamespace(
                get_type=lambda: "LOG_ENTRY",
                id=LOG_ID,
                num_logs=1,
                size=LOG_SIZE,
                time_utc=0,
            )
        if name == "LOG_REQUEST_END":
            if self._stream:
                self._stream.cancel()
            return None
        assert name == "LOG_REQUEST_DATA"
        self.requests.append((fields["ofs"], fields["count"]))
        if self._stream:
            self._stream.cancel()
        first = trio.Event()
        reply: list[SimpleNamespace] = []
        self._nursery.start_soon(
            self._send_range, fields["ofs"], fields["count"], first, reply
        )
        await first.wait()
        return reply[0]

    async def _send_range(self, ofs, count, first, reply):
        with trio.CancelScope() as self._stream:
            for frame_ofs in range(ofs, min(ofs + count, LOG_SIZE), 90):
                await trio.sleep(FRAME_INTERVAL)
                if frame_ofs in self.lose_once:
                    self.lose_once.discard(frame_ofs)
                    continue
                message = self._log_data(frame_ofs)
                if not first.is_set():
                    reply.append(message)
                    first.set()
                else:
                    self.downloader.handle_message_log_data(message)


async def download(lose_once: set[int]) -> tuple[bytes, float, FakeAutopilot]:
    async with trio.open_nursery() as nursery:
        autopilot = FakeAutopilot(nursery, lose_once)
        start = trio.current_time()
        log = None
        async for item in autopilot.downloader.get_log(LOG_ID):
            if isinstance(item, FlightLog):
                log = item
        elapsed = trio.current_time() - start
        nursery.cancel_scope.cancel()
    assert log is not None
    return b64decode(log.body), elapsed, autopilot


LOSSLESS_TIME = LOG_SIZE / 90 * FRAME_INTERVAL


async def test_download_without_losses(autojump_clock):
    data, elapsed, autopilot = await download(set())
    assert data == autopilot.log
    assert len(autopilot.requests) == 3
    assert elapsed < LOSSLESS_TIME * 1.1


async def test_losses_inside_a_range_are_requested_again_without_waiting(
    autojump_clock,
):
    lost = {90 * 10, 90 * 300, 90 * 700}
    data, elapsed, autopilot = await download(lost)
    assert data == autopilot.log
    # each loss is asked for again as soon as the end of its range arrives
    assert (90 * 10, 90) in autopilot.requests
    assert elapsed < LOSSLESS_TIME * 1.1 + 0.1


async def test_a_lost_end_of_range_costs_one_gap_timeout(autojump_clock):
    last_of_first_range = 512 * 90 - 90
    data, elapsed, autopilot = await download({last_of_first_range})
    assert data == autopilot.log
    # the next request starts at the lost message
    assert autopilot.requests[1][0] == last_of_first_range
    assert elapsed < LOSSLESS_TIME * 1.1 + LOG_DATA_GAP_TIMEOUT + 0.1
