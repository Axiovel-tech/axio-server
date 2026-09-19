"""Fast and partial Flockwave receipts must not be lost by the operator client."""

import json

import trio
from trio.testing import memory_stream_pair

from flockwave.server.model.messages import FlockwaveMessage
from flockwave.server.operator.client import ServerClient


async def read_request(stream):
    raw = b""
    while b"\n" not in raw:
        raw += await stream.receive_some()
    message = json.loads(raw)
    FlockwaveMessage.from_json(message)
    return message


async def send(stream, body, refs=None):
    await stream.send_all(
        (
            json.dumps(
                {"id": "server", "$fw.version": "1.0", "body": body, "refs": refs}
            )
            + "\n"
        ).encode()
    )


async def test_early_and_partial_receipts_are_preserved():
    local, remote = memory_stream_pair()

    async def server():
        request = await read_request(remote)
        await send(
            remote,
            {
                "type": "ASYNC-RESP",
                "id": "r1",
                "result": {"complete": True, "values": {"A": 1}},
            },
        )
        await send(
            remote,
            {
                "type": "OBJ-CMD",
                "receipt": {"11": "r1", "12": "r2"},
                "error": {"13": "offline"},
            },
            request["id"],
        )
        await send(
            remote,
            {
                "type": "ASYNC-RESP",
                "id": "r2",
                "result": {"complete": False, "errors": {"A": "missing"}},
            },
        )

    async with trio.open_nursery() as nursery:
        nursery.start_soon(server)
        result = await ServerClient(local).operate(
            ["11", "12", "13"], "read", names=["A"]
        )
    assert not result.complete
    assert result.results["11"]["values"] == {"A": 1}
    assert result.results["12"]["errors"] == {"A": "missing"}
    assert result.errors == {"13": "offline"}


async def test_deadline_keeps_completed_devices_and_marks_pending_unknown(
    autojump_clock,
):
    local, remote = memory_stream_pair()

    async def server():
        request = await read_request(remote)
        await send(
            remote,
            {
                "type": "OBJ-CMD",
                "receipt": {"12": "r2"},
                "result": {"11": {"complete": True}},
            },
            request["id"],
        )

    async with trio.open_nursery() as nursery:
        nursery.start_soon(server)
        result = await ServerClient(local).operate(
            ["11", "12"], "apply", values={"A": 1}, timeout=1
        )
    assert not result.complete
    assert result.results == {"11": {"complete": True}}
    assert "may still finish" in result.errors["12"]


async def test_server_omission_is_not_complete():
    local, remote = memory_stream_pair()

    async def server():
        request = await read_request(remote)
        await send(remote, {"type": "OBJ-CMD", "result": {}}, request["id"])

    async with trio.open_nursery() as nursery:
        nursery.start_soon(server)
        result = await ServerClient(local).operate(["11"], "read", names=["A"])
    assert not result.complete
    assert result.errors["11"]


async def test_server_deadline_result_arrives_before_client_expires(autojump_clock):
    local, remote = memory_stream_pair()

    async def server():
        request = await read_request(remote)
        budget = request["body"]["kwds"]["timeout"]
        assert 0 < budget < 1
        await send(remote, {"type": "OBJ-CMD", "receipt": {"11": "r1"}}, request["id"])
        await trio.sleep(budget + 0.05)
        await send(
            remote,
            {
                "type": "ASYNC-RESP",
                "id": "r1",
                "result": {
                    "complete": False,
                    "values": {"A": 1},
                    "error": "deadline reached",
                },
            },
        )

    async with trio.open_nursery() as nursery:
        nursery.start_soon(server)
        result = await ServerClient(local).operate(
            ["11"], "read", names=["A", "B"], timeout=1
        )
    assert not result.complete and not result.errors
    assert result.results["11"]["values"] == {"A": 1}
