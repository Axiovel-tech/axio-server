"""Focused tests for MAVFTP operations used by firmware OTA."""

from types import SimpleNamespace
from typing import cast

from flockwave.concurrency import AdaptiveExponentialBackoffPolicy

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
