"""MAVFTP client for reading flight-controller logs, including burst reads."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
from random import randint
from struct import Struct

import trio
from trio.abc import ReceiveChannel

from flockwave.server.errors import FlockwaveError
from flockwave.server.ext.mavlink.ftp import MAVFTPErrorCode, MAVFTPOpCode

from .transport import FCTransport, MAVLinkMessage

__all__ = ("CHUNK_SIZE", "FTPClient", "FTPError", "ListingEntry")

CHUNK_SIZE = 239

BURST_LENGTH = 500
"""Replies per BurstReadFile request in ArduPilot's MAVFTP server."""

_HEADER = Struct("<HBBBBBxI")

_SESSION_TIMEOUT = 3.0
"""ArduPilot frees another session's open file only after this long without
sending any MAVFTP reply; every reply, a NAK included, restarts the clock."""

_BUSY_ATTEMPTS = 4


class FTPError(FlockwaveError):
    def __init__(self, operation: MAVFTPOpCode, code: int, errno: int | None):
        super().__init__(
            f"MAVFTP {operation.name} failed: {MAVFTPErrorCode.to_string(code, errno)}"
        )
        self.code = code


@dataclass(frozen=True)
class Reply:
    seq: int
    session: int
    opcode: int
    request: int
    burst_complete: bool
    offset: int
    data: bytes

    @classmethod
    def decode(cls, message: MAVLinkMessage) -> Reply:
        payload = bytes(message.payload)
        seq, session, opcode, size, request, burst_complete, offset = (
            _HEADER.unpack_from(payload)
        )
        start = _HEADER.size
        return cls(
            seq,
            session,
            opcode,
            request,
            bool(burst_complete),
            offset,
            payload[start : start + size],
        )

    def error(self, operation: MAVFTPOpCode) -> FTPError:
        errno = self.data[1] if len(self.data) > 1 else None
        return FTPError(operation, self.data[0] if self.data else 0, errno)

    @property
    def is_eof(self) -> bool:
        return (
            self.opcode == MAVFTPOpCode.NAK
            and bool(self.data)
            and self.data[0] == MAVFTPErrorCode.EOF
        )


@dataclass(frozen=True)
class ListingEntry:
    name: str
    size: int
    is_file: bool


def _encode(
    seq: int, session: int, opcode: int, offset: int, data: bytes, size: int | None
) -> bytes:
    header = _HEADER.pack(
        seq, session, opcode, len(data) if size is None else size, 0, 0, offset
    )
    return (header + data).ljust(251, b"\x00")


class FTPClient:
    """One MAVFTP session against one flight controller.

    The caller owns the FILE_TRANSFER_PROTOCOL subscription so that every reply
    sent while this client exists reaches it.
    """

    def __init__(
        self,
        transport: FCTransport,
        replies: ReceiveChannel[MAVLinkMessage],
        *,
        timeout: float = 1.0,
        attempts: int = 8,
    ):
        self._transport = transport
        self._replies = replies
        self._timeout = timeout
        self._attempts = attempts
        self._seq = randint(0, 0xFFFF)
        self.session = randint(1, 254)

    async def request(
        self,
        opcode: MAVFTPOpCode,
        *,
        data: bytes = b"",
        offset: int = 0,
        size: int | None = None,
        timeout: float | None = None,
        attempts: int | None = None,
    ) -> Reply:
        """Sends a request and returns its ACK; a NAK raises FTPError.

        While a stale session (a client that died mid-transfer) still holds
        the flight controller's single open file, this waits silently until
        ArduPilot releases it.
        """
        exchange = partial(
            self._exchange, opcode, data, offset, size, timeout, attempts
        )
        for _ in range(_BUSY_ATTEMPTS - 1):
            try:
                return await exchange()
            except FTPError as error:
                held = error.code == MAVFTPErrorCode.INVALID_SESSION or (
                    opcode == MAVFTPOpCode.OPEN_FILE_RO
                    and error.code == MAVFTPErrorCode.FAIL
                )
                if not held:
                    raise
            await trio.sleep(_SESSION_TIMEOUT + 1)
        return await exchange()

    async def _exchange(
        self,
        opcode: MAVFTPOpCode,
        data: bytes,
        offset: int,
        size: int | None,
        timeout: float | None,
        attempts: int | None,
    ) -> Reply:
        """Retries reuse the sequence number so that ArduPilot answers a request
        it already executed with its cached reply instead of executing it again.
        """
        seq = self._seq
        expected = (seq + 1) & 0xFFFF
        payload = _encode(seq, self.session, opcode, offset, data, size)
        for _ in range(attempts or self._attempts):
            await self._send(payload)
            with trio.move_on_after(timeout or self._timeout):
                while True:
                    reply = Reply.decode(await self._replies.receive())
                    if reply.seq == expected and reply.request == opcode:
                        self._seq = (expected + 1) & 0xFFFF
                        if reply.opcode == MAVFTPOpCode.NAK:
                            raise reply.error(opcode)
                        return reply
        raise trio.TooSlowError(f"no MAVFTP reply to {opcode.name}")

    async def burst(self, offset: int, sink: Callable[[int, bytes], None]) -> bool:
        """Requests one burst from the open file and passes each chunk to sink.

        Returns whether the burst reached the end of the file. A burst that goes
        quiet for the request timeout ends early; the caller requests the
        missing ranges again.
        """
        seq = self._seq
        # Skip past every sequence number the burst can use so that late
        # replies can never match a later request.
        self._seq = (seq + BURST_LENGTH + 1) & 0xFFFF
        await self._send(
            _encode(
                seq,
                self.session,
                MAVFTPOpCode.BURST_READ_FILE,
                offset,
                b"",
                CHUNK_SIZE,
            )
        )
        while True:
            reply = None
            with trio.move_on_after(self._timeout):
                reply = Reply.decode(await self._replies.receive())
            if reply is None:
                return False
            if (
                reply.request != MAVFTPOpCode.BURST_READ_FILE
                or reply.session != self.session
                or (reply.seq - seq - 1) & 0xFFFF >= BURST_LENGTH
            ):
                continue
            if reply.opcode == MAVFTPOpCode.NAK:
                if reply.is_eof:
                    return True
                raise reply.error(MAVFTPOpCode.BURST_READ_FILE)
            sink(reply.offset, reply.data)
            if reply.burst_complete:
                return False

    async def listdir(self, path: str) -> list[ListingEntry]:
        entries: list[ListingEntry] = []
        while True:
            try:
                reply = await self.request(
                    MAVFTPOpCode.LIST_DIRECTORY,
                    data=path.encode(),
                    offset=len(entries),
                )
            except FTPError as error:
                if error.code == MAVFTPErrorCode.EOF:
                    return entries
                raise
            parts = [part for part in reply.data.split(b"\x00") if part]
            if not parts:
                return entries
            for part in parts:
                name, _, size = part[1:].decode().partition("\t")
                entries.append(ListingEntry(name, int(size or 0), part[:1] == b"F"))

    async def open_read(self, path: str) -> int:
        """Opens a file for reading and returns its size."""
        reply = await self.request(MAVFTPOpCode.OPEN_FILE_RO, data=path.encode())
        return int.from_bytes(reply.data[:4], "little")

    async def read(self, offset: int, size: int) -> bytes:
        try:
            reply = await self.request(MAVFTPOpCode.READ_FILE, offset=offset, size=size)
        except FTPError as error:
            if error.code == MAVFTPErrorCode.EOF:
                return b""
            raise
        return reply.data

    async def terminate(self) -> None:
        await self.request(MAVFTPOpCode.TERMINATE_SESSION, attempts=2)

    async def crc32(self, path: str, deadline: float) -> int:
        """Remote whole-file CRC32.

        The flight controller computes it in one blocking step, so this resends
        slowly until the deadline instead of using the normal retry budget.
        """
        resend = 5.0
        reply = await self.request(
            MAVFTPOpCode.CALC_FILE_CRC32,
            data=path.encode(),
            timeout=resend,
            attempts=max(1, int(deadline / resend)),
        )
        return int.from_bytes(reply.data[:4], "little")

    async def _send(self, payload: bytes) -> None:
        await self._transport.send(
            "FILE_TRANSFER_PROTOCOL", target_network=0, payload=payload
        )
