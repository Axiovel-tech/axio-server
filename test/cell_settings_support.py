"""The cell settings tests' cell (anchor_lock_support's, with every device
reporting what it runs) and the helpers that drive X-RTLS-CELL against it."""

import time

from anchor_lock_support import CODE_50, HEALTHY_A0, HEALTHY_A1, SESSION, _cell
from rtls_support import make_message
from rtlslink.cell_settings import code

CODE_83 = code({**{k: v for k, (v, _) in SESSION.items()}, "UWB_BLOCK_MS": 12})


TAG = 42


async def _running(ext, codes, tx=47, **stats):
    """Each device reports the code (anchors also the TX level) it runs;
    anchors operating unless ``stats`` say otherwise."""
    for system_id, running in codes.items():
        body = {"cellcfg": float(running)}
        if system_id != TAG:
            healthy = HEALTHY_A0 if system_id == 201 else HEALTHY_A1
            body = {**healthy, **stats, **body, "txgain": float(tx)}
        await ext._on_stats(system_id, body, time.monotonic())


async def _converged_cell(dialect, running=CODE_50):
    ext, devices = await _cell(dialect)
    await _running(ext, {TAG: running, 201: running, 202: running})
    return ext, devices


async def _request(ext, builder, hub, **body):
    message = make_message(builder, {"type": "X-RTLS-CELL"} | body)
    response = await ext._handle_RTLS_CELL(message, None, hub)
    return response.body


def _cell_settings(ext):
    return ext._inf_body(time.monotonic())["cellSettings"]


def _param(device, name):
    return int.from_bytes(device.params[name][0], "little")


_SCHEDULE = ("UWB_BLOCK_MS", "UWB_SLOTS_RR", "UWB_SLOT_RSTU")


def _schedule(device):
    """The ``{name: value}`` of the schedule ``device`` stores now."""
    return {name: _param(device, name) for name in _SCHEDULE}


def _steps(start, writes):
    """The ``(block ms, slots, slot RSTU)`` stored after each schedule write of
    ``writes``, starting from the ``start`` schedule."""
    params, steps = dict(start), []
    for name, value in writes:
        if name in params:
            params[name] = value
            steps.append(tuple(params[name] for name in _SCHEDULE))
    return steps


def _boots(schedule):
    """Whether the firmware boots ``schedule``: its round fits its block."""
    block_ms, slots, slot_rstu = schedule
    return slots * slot_rstu <= block_ms * 1200


def _states(ext):
    return {
        int(sid): (entry.get("cellSettings"), entry.get("cellSettingsDetail"))
        for sid, entry in ext._inf_status(time.monotonic()).items()
    }


def _lose_acks(device, dialect, name, *, after=0):
    """``device`` stores every write of ``name`` but the acks after the first
    ``after`` are lost."""
    handle, parser, acked = device.handle, dialect.MAVLink(None), [0]

    def lossy(data):
        out = []
        for reply in handle(data):
            message = (parser.parse_buffer(reply) or [None])[0]
            if message is not None and message.get_type() == "PARAM_EXT_ACK":
                if message.param_id == name:
                    acked[0] += 1
                    if acked[0] > after:
                        continue
            out.append(reply)
        return out

    device.handle = lossy


def _dropped_from_dump(name, *, answers=True):
    def prepare(devices):
        devices[202].drop_from_list = {name}
        devices[202].respond_to_read = answers

    return prepare


def _record_writes(ext):
    """The ``(name, value)`` written to each device, in order."""
    writes, write = {}, ext.set_param

    async def set_param(system_id, name, value, *args, **kwargs):
        writes.setdefault(system_id, []).append((name, value))
        return await write(system_id, name, value, *args, **kwargs)

    ext.set_param = set_param
    return writes


async def _a_cell_without_a0(dialect):
    ext, devices = await _cell(dialect)
    ext._stats.pop(TAG)
    await _running(ext, {202: CODE_50})
    return ext, devices
