"""The cell settings tests' cell (anchor_lock_support's) with devices that
behave like the firmware: a cell write applies at once, so the next stats
report the new code, and a write whose schedule would not run is refused in
its ack."""

import time

from anchor_lock_support import HEALTHY_A0, HEALTHY_A1, SESSION, _cell
from rtls_support import make_message
from rtlslink.cell_settings import CODE_PARAMS, code
from rtlslink.protocol import PARAM_ACK_FAILED, raw_field_bytes

CODE_83 = code({**{k: v for k, (v, _) in SESSION.items()}, "UWB_BLOCK_MS": 12})

TAG = 42

_SCHEDULE = ("UWB_BLOCK_MS", "UWB_SLOTS_RR", "UWB_SLOT_RSTU")
_RSTU_PER_MS = 1200


def _param(device, name):
    return int.from_bytes(device.params[name][0], "little")


def _runs_live(device, dialect):
    """``device`` refuses a schedule write whose round outgrows its block,
    as the firmware's write-time check does."""
    handle, parser = device.handle, dialect.MAVLink(None)

    def checked(data):
        message = (parser.parse_buffer(data) or [None])[0]
        if message is not None and message.get_type() == "PARAM_EXT_SET":
            name = message.param_id
            if name in _SCHEDULE:
                schedule = {n: _param(device, n) for n in _SCHEDULE}
                value = raw_field_bytes(message, "param_value") or b""
                schedule[name] = int.from_bytes(value[:2], "little")
                block, slots, rstu = (schedule[n] for n in _SCHEDULE)
                if slots * rstu > block * _RSTU_PER_MS:
                    kept, device.set_result = device.set_result, PARAM_ACK_FAILED
                    try:
                        return handle(data)
                    finally:
                        device.set_result = kept
        return handle(data)

    device.handle = checked


async def _report(ext, devices, ids=None, **stats):
    """Each device of ``ids`` (default: all) sends its stats: the code of
    what it stores, which it runs, and an anchor its TX level, operating
    unless ``stats`` say otherwise."""
    for system_id in ids or devices:
        device = devices[system_id]
        body = {"cellcfg": float(code({n: _param(device, n) for n in CODE_PARAMS}))}
        if system_id != TAG:
            healthy = HEALTHY_A0 if system_id == 201 else HEALTHY_A1
            tx = float(_param(device, "UWB_TX_GAIN"))
            body = {**healthy, **body, "txgain": tx, **stats}
        await ext._on_stats(system_id, body, time.monotonic())


async def _live_cell(dialect, **kwargs):
    """The cell, every device running live and reporting what it runs."""
    ext, devices = await _cell(dialect, **kwargs)
    for device in devices.values():
        _runs_live(device, dialect)
    await _report(ext, devices)
    return ext, devices


async def _request(ext, builder, hub, **body):
    message = make_message(builder, {"type": "X-RTLS-CELL"} | body)
    response = await ext._handle_RTLS_CELL(message, None, hub)
    return response.body


def _cell_settings(ext):
    return ext._inf_body(time.monotonic())["cellSettings"]


def _states(ext):
    return {
        int(sid): (entry.get("cellSettings"), entry.get("cellSettingsDetail"))
        for sid, entry in ext._inf_status(time.monotonic()).items()
    }


def _record_writes(ext):
    """The ``(name, value)`` written to each device, in order."""
    writes, write = {}, ext.set_param

    async def set_param(system_id, name, value, *args, **kwargs):
        writes.setdefault(system_id, []).append((name, value))
        return await write(system_id, name, value, *args, **kwargs)

    ext.set_param = set_param
    return writes
