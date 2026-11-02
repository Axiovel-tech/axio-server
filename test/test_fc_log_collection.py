import json
import os
from collections import defaultdict
from pathlib import Path
from struct import Struct
from types import SimpleNamespace

import pytest
import trio

from flockwave.server.ext.mavlink.ftp import MAVFTPErrorCode, MAVFTPOpCode
from flockwave.server.ext.mavlink.log_collection.cli import ServerSession
from flockwave.server.ext.mavlink.log_collection.collect import (
    CollectRequest,
    collect,
    exclusive,
    newest_first,
)
from flockwave.server.ext.mavlink.log_collection.download import (
    Coverage,
    mavftp_crc32,
)
from flockwave.server.ext.mavlink.log_collection.ftp_client import ListingEntry
from flockwave.server.ext.mavlink.log_collection.runner import collect_with_status
from flockwave.server.ext.mavlink.log_collection.status import Status
from flockwave.server.show.utils import crc32_mavftp

HEADER = Struct("<HBBBBBxI")
OP = MAVFTPOpCode


class Message(SimpleNamespace):
    def get_type(self):
        return self.mavpackettype

    def get_srcSystem(self):
        return 3

    def get_srcComponent(self):
        return 1


class FakeFlightController:
    """ArduPilot-like MAVFTP, parameters and heartbeat for one system."""

    system_id = 3
    description = {"kind": "fake"}

    def __init__(self, files: dict[str, bytes], lastlog: int | None = None):
        self.files = dict(files)
        if lastlog is not None:
            self.files["LASTLOG.TXT"] = f"{lastlog}\r\n".encode()
        self.params = {
            "SHOW_START_AUTH": 0.0,
            "SHOW_START_TIME": -1.0,
            "LOG_DISARMED": 1.0,
        }
        self.base_mode = 89
        self.millivolts = 12600
        self.dropped_burst_packets: set[int] = set()
        self.crc_override: int | None = None
        self.answer_reads_until: int | None = None
        self.param_writes: list[tuple[str, float]] = []
        self.on_read = None
        self.stale_session = False
        self._last_reply = 0.0
        self._subscribers = defaultdict(list)

    def subscribe(self, message_type):
        sender, receiver = trio.open_memory_channel(4096)
        subscribers = self._subscribers[message_type]

        class Subscription:
            def __enter__(self):
                subscribers.append(sender)
                return receiver

            def __exit__(self, *_):
                subscribers.remove(sender)

        return Subscription()

    async def run_telemetry(self):
        while True:
            self._publish("HEARTBEAT", base_mode=self.base_mode)
            self._publish("SYS_STATUS", voltage_battery=self.millivolts)
            await trio.sleep(0.2)

    async def send(self, message_type, **fields):
        await trio.lowlevel.checkpoint()
        getattr(self, f"_on_{message_type.lower()}")(**fields)

    def _publish(self, message_type, **fields):
        for subscriber in self._subscribers.get(message_type, ()):
            subscriber.send_nowait(Message(mavpackettype=message_type, **fields))

    def _on_param_request_read(self, param_id, **_):
        name = param_id.decode()
        if name in self.params:
            self._publish(
                "PARAM_VALUE",
                param_id=name,
                param_value=self.params[name],
                param_type=2,
            )

    def _on_param_set(self, param_id, param_value, **_):
        name = param_id.decode()
        self.params[name] = param_value
        self.param_writes.append((name, param_value))
        self._on_param_request_read(param_id)

    def _reply(self, seq, req, opcode=OP.ACK, data=b"", offset=0, burst=False):
        self._last_reply = trio.current_time()
        payload = HEADER.pack(
            seq & 0xFFFF, self.session, opcode, len(data), req, burst, offset
        )
        self._publish(
            "FILE_TRANSFER_PROTOCOL", payload=(payload + data).ljust(251, b"\0")
        )

    def _nak(self, seq, req, code):
        self._reply(seq, req, OP.NAK, bytes([code]))

    def _on_file_transfer_protocol(self, payload, **_):
        seq, self.session, opcode, size, _, _, offset = HEADER.unpack_from(payload)
        data = bytes(payload[12 : 12 + size])
        name = data.decode().rsplit("/", 1)[-1]
        if self.stale_session:
            if trio.current_time() - self._last_reply < 3:
                return self._nak(seq + 1, opcode, MAVFTPErrorCode.INVALID_SESSION)
            self.stale_session = False
        if opcode == OP.LIST_DIRECTORY:
            names = sorted(self.files)
            if offset >= len(names):
                return self._nak(seq + 1, opcode, MAVFTPErrorCode.EOF)
            entry = f"F{names[offset]}\t{len(self.files[names[offset]])}\0"
            return self._reply(seq + 1, opcode, data=entry.encode())
        if opcode == OP.OPEN_FILE_RO:
            if name not in self.files:
                return self._nak(seq + 1, opcode, MAVFTPErrorCode.FAIL_ERRNO)
            self.open = name
            return self._reply(
                seq + 1, opcode, data=len(self.files[name]).to_bytes(4, "little")
            )
        if opcode == OP.CALC_FILE_CRC32:
            crc = (
                self.crc_override
                if self.crc_override is not None
                else crc32_mavftp(self.files[name])
            )
            return self._reply(seq + 1, opcode, data=crc.to_bytes(4, "little"))
        if opcode == OP.TERMINATE_SESSION:
            return self._reply(seq + 1, opcode)
        content = self.files[self.open]
        if opcode == OP.READ_FILE:
            return self._read(seq + 1, opcode, content, offset, size)
        if opcode == OP.BURST_READ_FILE:
            for index in range(500):
                if not self._read(
                    seq + 1 + index,
                    opcode,
                    content,
                    offset + index * 239,
                    239,
                    burst=index == 499,
                    index=index,
                ):
                    return

    def _read(self, seq, opcode, content, offset, size, burst=False, index=None):
        if self.answer_reads_until is not None and offset >= self.answer_reads_until:
            return False
        chunk = content[offset : offset + size]
        if not chunk:
            self._nak(seq, opcode, MAVFTPErrorCode.EOF)
            return False
        if index not in self.dropped_burst_packets:
            self._reply(seq, opcode, data=chunk, offset=offset, burst=burst)
        if self.on_read:
            self.on_read(offset)
        return True


def log_bytes(number: int, size: int) -> bytes:
    return bytes((number * 31 + i * 7) & 0xFF for i in range(size))


@pytest.fixture
def fc(nursery):
    controller = FakeFlightController(
        {f"{n:08}.BIN": log_bytes(n, 30_000 + n * 1_000) for n in (1, 2, 3)},
        lastlog=3,
    )
    nursery.start_soon(controller.run_telemetry)
    return controller


def request(tmp_path: Path, **kwds) -> CollectRequest:
    return CollectRequest(directory=tmp_path / "03", log_dir="logs", **kwds)


async def run(fc, tmp_path, **kwds):
    with exclusive(tmp_path / "03"):
        return await collect(fc, request(tmp_path, **kwds), Status("03"))


def test_crc_matches_ardupilot_definition():
    data = os.urandom(10_000)
    assert mavftp_crc32(data) == crc32_mavftp(data)
    assert mavftp_crc32(data[5000:], mavftp_crc32(data[:5000])) == crc32_mavftp(data)


def test_coverage_merges_and_reports_gaps():
    coverage = Coverage([[10, 20], [30, 40]])
    coverage.add(18, 31)
    coverage.add(50, 60)
    assert coverage.spans == [[10, 40], [50, 60]]
    assert coverage.gaps(70) == [(0, 10), (40, 50), (60, 70)]
    assert coverage.covered == 40


def test_newest_first_follows_lastlog_across_wraparound():
    entries = [ListingEntry(f"{n:08}.BIN", 1, True) for n in (1, 2, 499, 500)]
    assert [e.name[:8] for e in newest_first(entries, 2)] == [
        "00000002",
        "00000001",
        "00000500",
        "00000499",
    ]
    assert newest_first(entries, None)[0].name == "00000500.BIN"


async def test_collects_newest_logs_and_restores_logging(fc, tmp_path, autojump_clock):
    receipt = await run(fc, tmp_path, latest=2)

    assert receipt["complete"], receipt
    assert [f["file"] for f in receipt["files"]] == ["00000003.BIN", "00000002.BIN"]
    directory = tmp_path / "03"
    for name in ("00000003.BIN", "00000002.BIN"):
        assert (directory / name).read_bytes() == fc.files[name]
        assert json.loads((directory / f"{name}.json").read_text())["bytes"] == len(
            fc.files[name]
        )
    manifest = json.loads((directory / "manifest.json").read_text())
    assert [f["file"] for f in manifest["files"]] == ["00000002.BIN", "00000003.BIN"]
    assert fc.param_writes == [("LOG_DISARMED", 0), ("LOG_DISARMED", 1.0)]
    assert not (directory / "collect-state.json").exists()
    assert not list(directory.glob("*.part*"))
    assert receipt["provenance"]["tool"] == "axio-fc-logs"


async def test_lost_burst_packets_are_fetched_again(fc, tmp_path, autojump_clock):
    fc.dropped_burst_packets = {3, 4, 50}
    receipt = await run(fc, tmp_path, files=("00000001.BIN",))
    assert receipt["complete"], receipt
    assert (tmp_path / "03" / "00000001.BIN").read_bytes() == fc.files["00000001.BIN"]


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"base_mode": 89 | 128}, "armed"),
        ({"params": {"SHOW_START_AUTH": 1.0}}, "authorized"),
        ({"params": {"SHOW_START_TIME": 1234.0}}, "scheduled"),
    ],
)
async def test_refuses_before_touching_the_flight_controller(
    fc, tmp_path, autojump_clock, change, reason
):
    fc.base_mode = change.get("base_mode", fc.base_mode)
    fc.params.update(change.get("params", {}))
    receipt = await run(fc, tmp_path)
    assert not receipt["complete"]
    assert reason in receipt["error"]
    assert fc.param_writes == []
    assert not list((tmp_path / "03").glob("*.BIN*"))


async def test_crc_mismatch_rejects_and_discards_the_copy(fc, tmp_path, autojump_clock):
    fc.crc_override = 0x12345678
    receipt = await run(fc, tmp_path)
    assert not receipt["complete"]
    assert receipt["files"][0]["status"] == "failed"
    assert "CRC32" in receipt["files"][0]["error"]
    directory = tmp_path / "03"
    assert not list(directory.glob("*.BIN*"))
    assert fc.params["LOG_DISARMED"] == 1.0
    assert receipt["log_disarmed"]["restore_verified"]


async def test_stalled_transfer_resumes_on_the_next_run(fc, tmp_path, autojump_clock):
    fc.answer_reads_until = 20_000
    first = await run(fc, tmp_path)
    assert not first["complete"]
    assert "stalled" in first["error"]
    assert fc.params["LOG_DISARMED"] == 1.0
    assert not (tmp_path / "03" / "00000003.BIN").exists()

    fc.answer_reads_until = None
    second = await run(fc, tmp_path)
    assert second["complete"], second
    assert second["files"][0]["resumed_from_bytes"] >= 19_000
    assert (tmp_path / "03" / "00000003.BIN").read_bytes() == fc.files["00000003.BIN"]


async def test_corrupted_partial_is_rejected_on_resume(fc, tmp_path, autojump_clock):
    fc.answer_reads_until = 20_000
    await run(fc, tmp_path)
    part = tmp_path / "03" / "00000003.BIN.part"
    with part.open("r+b") as stream:
        stream.seek(1000)
        stream.write(b"\xff" * 16)

    fc.answer_reads_until = None
    receipt = await run(fc, tmp_path)
    assert not receipt["complete"]
    assert "CRC32" in receipt["files"][0]["error"]
    assert not part.exists()
    assert not (tmp_path / "03" / "00000003.BIN").exists()

    assert (await run(fc, tmp_path))["complete"]


async def test_session_left_by_a_dead_client_is_waited_out(
    fc, tmp_path, autojump_clock
):
    fc.stale_session = True
    receipt = await run(fc, tmp_path)
    assert receipt["complete"], receipt


async def test_cancellation_mid_transfer_still_restores(fc, tmp_path, autojump_clock):
    with trio.CancelScope() as scope:
        fc.on_read = lambda offset: offset > 10_000 and scope.cancel()
        await run(fc, tmp_path)
    assert scope.cancelled_caught
    assert fc.params["LOG_DISARMED"] == 1.0
    directory = tmp_path / "03"
    assert not (directory / "collect-state.json").exists()
    receipt = json.loads(next(directory.glob("run-*.json")).read_text())
    assert receipt["log_disarmed"]["restore_verified"]
    assert not receipt["complete"]


async def test_original_from_an_interrupted_run_is_restored(
    fc, tmp_path, autojump_clock
):
    directory = tmp_path / "03"
    directory.mkdir()
    (directory / "collect-state.json").write_text('{"log_disarmed_original": 1.0}')
    fc.params["LOG_DISARMED"] = 0.0
    receipt = await run(fc, tmp_path)
    assert receipt["complete"], receipt
    assert receipt["log_disarmed"]["recovered_from_interrupted_run"]
    assert fc.params["LOG_DISARMED"] == 1.0


async def test_battery_limit_refuses_and_stops(fc, tmp_path, autojump_clock):
    refused = await run(fc, tmp_path, min_battery_voltage=13.0)
    assert "below 13.0 V" in refused["error"]
    assert fc.param_writes == []

    def drain(offset):
        if offset > 5_000:
            fc.millivolts = 10_500

    fc.on_read = drain
    stopped = await run(fc, tmp_path, min_battery_voltage=11.0)
    assert "battery fell to 10.5 V" in stopped["error"]
    assert fc.params["LOG_DISARMED"] == 1.0


async def test_arming_mid_collection_stops_the_transfer(fc, tmp_path, autojump_clock):
    def arm(offset):
        if offset > 5_000:
            fc.base_mode = 89 | 128

    fc.on_read = arm
    receipt = await run(fc, tmp_path)
    assert not receipt["complete"]
    assert "armed" in receipt["error"]
    assert fc.params["LOG_DISARMED"] == 1.0


async def test_verified_copy_is_reused_and_a_changed_one_is_kept(
    fc, tmp_path, autojump_clock
):
    assert (await run(fc, tmp_path))["complete"]
    again = await run(fc, tmp_path)
    assert again["complete"] and again["files"][0]["reused"]

    fc.files["00000003.BIN"] += b"later"
    changed = await run(fc, tmp_path)
    assert not changed["complete"]
    assert "changed on the flight controller" in changed["files"][0]["error"]
    assert (tmp_path / "03" / "00000003.BIN").stat().st_size == len(
        fc.files["00000003.BIN"]
    ) - 5


async def test_status_snapshots_are_bounded_and_end_with_a_receipt(
    fc, tmp_path, autojump_clock
):
    events = []
    async with collect_with_status(
        fc, request(tmp_path, latest=3), "03", 0.05
    ) as items:
        async for kind, data in items:
            events.append((kind, data))
    kinds = [kind for kind, _ in events]
    assert kinds[-1] == "receipt" and kinds.count("receipt") == 1
    snapshot = events[0][1]
    assert set(snapshot) == {
        "uav", "phase", "file", "files_done", "files_total", "bytes_done",
        "bytes_total", "rate_Bps", "eta_s", "battery_v", "error", "updated_at",
    }  # fmt: skip
    final = json.loads((tmp_path / "03" / "status.json").read_text())
    assert final["phase"] == "done" and final["files_done"] == 3


async def test_second_collection_of_a_vehicle_is_refused(fc, tmp_path, autojump_clock):
    with exclusive(tmp_path / "03"):
        async with collect_with_status(fc, request(tmp_path), "03", 1) as items:
            events = [event async for event in items]
    assert events == [
        (
            "receipt",
            {
                "uav": "03",
                "complete": False,
                "error": "another collection is running for this vehicle",
            },
        )
    ]
    assert fc.param_writes == []
    assert not (tmp_path / "03" / "status.json").exists()


def test_server_session_matches_early_async_messages_to_receipts(capsys):
    session = ServerSession.__new__(ServerSession)
    session._uavs = ["03", "04"]
    session.receipts, session.results = {}, {}
    session._accept({"receipt": {"03": "r1"}, "error": {"04": "no such UAV"}})
    session._handle(
        {
            "type": "ASYNC-ST",
            "id": "r1",
            "progress": {"object": {"phase": "transferring"}},
        }
    )
    session._handle({"type": "ASYNC-RESP", "id": "r1", "result": {"complete": True}})
    assert session.results == {
        "03": {"complete": True},
        "04": {"complete": False, "error": "no such UAV"},
    }
    assert '"phase":"transferring"' in capsys.readouterr().out
