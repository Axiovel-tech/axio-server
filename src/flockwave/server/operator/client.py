"""Small sequential Flockwave client with async receipt completion."""

from __future__ import annotations

from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from typing import AsyncIterator

import trio
from flockwave.channels import ParserChannel
from flockwave.encoders.json import create_json_encoder
from flockwave.parsers.json import create_json_parser


@dataclass
class FleetResult:
    results: dict[str, dict[str, object]] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)

    @property
    def complete(self) -> bool:
        return not self.errors and all(
            result.get("complete", False) for result in self.results.values()
        )


class ServerClient:
    """Reuse one TCP connection; operations on this object are serialized."""

    def __init__(self, stream: trio.SocketStream):
        self._stream = stream
        self._messages = ParserChannel(stream.receive_some, create_json_parser())
        self._encode = create_json_encoder()
        self._lock = trio.Lock()
        self._sequence = 0

    @classmethod
    @asynccontextmanager
    async def connect(
        cls, host: str = "127.0.0.1", port: int = 5001
    ) -> AsyncIterator[ServerClient]:
        async with await trio.open_tcp_stream(host, port) as stream:
            yield cls(stream)

    async def request(
        self, body: dict[str, object], *, timeout: float = 30
    ) -> dict[str, object]:
        async with self._lock:
            with trio.fail_after(timeout):
                self._sequence += 1
                identifier = f"operator-{self._sequence}"
                await self._stream.send_all(
                    self._encode({"$fw.version": "1.0", "id": identifier, "body": body})
                )
                while True:
                    message = await self._messages.receive()
                    if identifier == message.get("refs"):
                        result = message["body"]
                        if result.get("type") == "ACK-NAK":
                            raise RuntimeError(
                                result.get("reason", "server rejected the request")
                            )
                        return result

    async def operate(
        self,
        uav_ids: list[str],
        operation: str,
        *,
        names: list[str] | None = None,
        values: dict[str, float] | None = None,
        timeout: float = 30,
    ) -> FleetResult:
        if not uav_ids or len(uav_ids) > 16 or len(set(uav_ids)) != len(uav_ids):
            raise ValueError("select 1..16 distinct UAV IDs")
        if not 0 < timeout <= 120:
            raise ValueError("timeout must be within 0..120 seconds")
        # Leave time for the server's partial result to reach this connection.
        server_timeout = timeout - min(1.0, timeout / 5)
        result = FleetResult()
        async with self._lock:
            with trio.move_on_after(timeout) as deadline:
                self._sequence += 1
                identifier = f"operator-{self._sequence}"
                await self._stream.send_all(
                    self._encode(
                        {
                            "$fw.version": "1.0",
                            "id": identifier,
                            "body": {
                                "type": "OBJ-CMD",
                                "ids": uav_ids,
                                "command": "__operator",
                                "args": [operation],
                                "kwds": {
                                    # JSON encoders may sort object keys; carry
                                    # write order separately from the value map.
                                    "names": list(values)
                                    if values is not None
                                    else names,
                                    "values": values,
                                    "timeout": server_timeout,
                                },
                            },
                        }
                    )
                )
                pending: dict[str, str] = {}
                early: dict[str, dict] = {}
                initial_received = False
                while not initial_received or pending:
                    message = await self._messages.receive()
                    body = message["body"]
                    if identifier == message.get("refs"):
                        if body.get("type") == "ACK-NAK":
                            raise RuntimeError(
                                body.get("reason", "server rejected the operation")
                            )
                        initial_received = True
                        result.results.update(body.get("result", {}))
                        result.errors.update(body.get("error", {}))
                        pending = {
                            receipt: uid
                            for uid, receipt in body.get("receipt", {}).items()
                        }
                        for receipt, response in early.items():
                            self._finish(receipt, response, pending, result)
                    elif body.get("type") == "ASYNC-RESP":
                        receipt = body["id"]
                        if not initial_received and len(early) < len(uav_ids):
                            early[receipt] = body
                        else:
                            self._finish(receipt, body, pending, result)
                    elif body.get("type") == "ASYNC-TIMEOUT":
                        for receipt in body["ids"]:
                            self._finish(
                                receipt,
                                {"error": "server operation timed out"},
                                pending,
                                result,
                            )
            if deadline.cancelled_caught:
                for uid in uav_ids:
                    if uid not in result.results and uid not in result.errors:
                        result.errors[uid] = (
                            "deadline reached; an accepted operation may still finish"
                        )
        for uid in uav_ids:
            if uid not in result.results and uid not in result.errors:
                result.errors[uid] = "server omitted a result for this UAV"
        return result

    @staticmethod
    def _finish(
        receipt: str, body: dict, pending: dict[str, str], result: FleetResult
    ) -> None:
        uid = pending.pop(receipt, None)
        if uid is not None:
            if "error" in body:
                result.errors[uid] = str(body["error"])
            else:
                payload = body.get("result")
                if isinstance(payload, dict):
                    result.results[uid] = payload
                else:
                    result.errors[uid] = "server returned an invalid operator result"
