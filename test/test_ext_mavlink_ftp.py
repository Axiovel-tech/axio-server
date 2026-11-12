"""Focused tests for MAVFTP operations used by firmware OTA."""

from types import SimpleNamespace
from typing import cast

import trio
from flockwave.concurrency import AdaptiveExponentialBackoffPolicy

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.ftp import MAVFTP, MAVFTPMessage, MAVFTPOpCode
from flockwave.server.ext.mavlink.types import UAVBoundPacketSenderFn


async def test_mavftp_retries_lost_crc_requests() -> None:
    calls = 0

    async def sender(packet, **kwargs):
        nonlocal calls
        calls += 1
        if calls < 3:
            raise TimeoutError
        request = packet[1]["payload"]
        request_sequence = request[0] | request[1] << 8
        reply = MAVFTPMessage(MAVFTPOpCode.ACK, data=b"\x78\x56\x34\x12")
        return SimpleNamespace(payload=reply.encode((request_sequence + 1) % 65536))

    ftp = MAVFTP(
        cast(UAVBoundPacketSenderFn, sender),
        retry_policy=AdaptiveExponentialBackoffPolicy(
            max_retries=3, base_timeout=0, max_timeout=0
        ),
    )
    assert await ftp.crc32("/firmware.part") == 0x12345678
    assert calls == 3


async def test_mavftp_rename_uses_two_nul_separated_paths() -> None:
    sent = []

    class RecordingMAVFTP(MAVFTP):
        async def _send_and_wait(self, message, *, allow_nak=False):
            sent.append(message)
            return MAVFTPMessage(MAVFTPOpCode.ACK)

    async def unused_sender(*args, **kwargs):
        return None

    ftp = RecordingMAVFTP(cast(UAVBoundPacketSenderFn, unused_sender))
    await ftp.rename("/ardupilot.abin.part", "/ardupilot.abin")

    assert sent[0].opcode == MAVFTPOpCode.RENAME
    assert sent[0].data == b"ardupilot.abin.part\x00ardupilot.abin"


async def test_mavftp_connections_to_one_uav_do_not_overlap() -> None:
    events: list[str] = []
    connection_count = 0
    first_entered = trio.Event()
    release_first = trio.Event()
    second_started = trio.Event()
    second_entered = trio.Event()

    class Connection:
        def __init__(self, name: str):
            self.name = name

        async def aclose(self) -> None:
            events.append(f"close {self.name}")

    class RecordingMAVFTP(MAVFTP):
        @classmethod
        def for_uav(cls, uav, *, retry_policy=None):
            nonlocal connection_count
            del cls, retry_policy
            assert uav is candidate
            connection_count += 1
            name = f"connection {connection_count}"
            events.append(f"open {name}")
            return Connection(name)

    candidate = cast(MAVLinkUAV, SimpleNamespace(mavftp_lock=trio.Lock()))

    async def use_first_connection() -> None:
        async with RecordingMAVFTP.use_for_uav(candidate):
            events.append("use first")
            first_entered.set()
            await release_first.wait()

    async def use_second_connection() -> None:
        second_started.set()
        async with RecordingMAVFTP.use_for_uav(candidate):
            events.append("use second")
            second_entered.set()

    async with trio.open_nursery() as nursery:
        nursery.start_soon(use_first_connection)
        await first_entered.wait()
        nursery.start_soon(use_second_connection)
        await second_started.wait()
        await trio.lowlevel.checkpoint()
        assert not second_entered.is_set()
        release_first.set()
        await second_entered.wait()

    assert events == [
        "open connection 1",
        "use first",
        "close connection 1",
        "open connection 2",
        "use second",
        "close connection 2",
    ]
