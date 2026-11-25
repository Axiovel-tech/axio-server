"""A surveyed cell for the tests of anchor-owned geometry:
anchor_lock_support's cell with anchors that store their own table and
position, and a tag that holds an origin and reports its anchor table."""

import time

from anchor_lock_support import CODE_50, HEALTHY_A0, HEALTHY_A1, _cell, location
from rtls_support import DEVICE_SYSID, make_message, set_fake_param

TAG = DEVICE_SYSID

#: system ID, UWB_ROLE, UWB_MAC: 201 is the stored initiator, row 0
ANCHORS = ((201, 2, 1), (202, 3, 2), (203, 3, 3), (204, 3, 4))

#: where each anchor stores it stands: north, east, down in metres
POSITIONS = {
    201: (0.0, 0.0, 0.0),
    202: (4.0, 0.0, 0.0),
    203: (4.0, 3.0, -2.5),
    204: (0.0, 3.0, -2.5),
}

ORIGIN = {"ORIGIN_LAT_E7": 413900000, "ORIGIN_LON_E7": 21500000, "ORIGIN_ALT_MM": 10000}

#: codes of the ``antbl`` states
IN_USE, OTHER_SURVEY, NO_POSITION, UNKNOWN_LAYOUT, TABLE_FULL = 1, 2, 3, 4, 5

#: the row an ``antbl`` entry carries for an anchor the tag leaves out
NO_ROW = 15


def antbl(mac, row=NO_ROW, state=None):
    """One ``antbl`` stat: in use when a row is given, else left out."""
    if state is None:
        state = IN_USE if row != NO_ROW else NO_POSITION
    return float(row << 20 | state << 16 | mac)


#: what a tag positioned on the cell's four anchors reports
TAG_STATS = {
    "rate": 83.0,
    "solvepct": 99.0,
    "agems": 12.0,
    "ppm": 1.2,
    "ancmask": 15.0,
    "clkok": 1.0,
    "cellcfg": float(CODE_50),
    "sid": 1.0,
    "anuse": 4.0,
    "anexc": 0.0,
    "sdrift": 0.02,
    "antbl": antbl(1, row=0),
}


def give_origin(device, **changed):
    for name, value in (ORIGIN | changed).items():
        set_fake_param(device, name, value, "int32")
    set_fake_param(device, "POS_YAW_DEG", 90.0, "real32")


async def surveyed_cell(dialect, *, anchors=ANCHORS, blank=(), prepare=None):
    """An extension that discovered a tag holding an origin and the anchors
    of one cell, each storing its table and, unless in ``blank``, its
    position on survey 1; returns it and the devices by system ID."""

    def setup(devices):
        give_origin(devices[TAG])
        for system_id, _, _ in anchors:
            device = devices[system_id]
            set_fake_param(device, "UWB_AN_COUNT", len(anchors), "uint8")
            for row, (_, _, mac) in enumerate(anchors):
                set_fake_param(device, f"UWB_AN{row}_MAC", mac, "uint16")
            stored = "" if system_id in blank else location(*POSITIONS[system_id])
            set_fake_param(device, "UWB_LOC", stored, "custom")
        if prepare is not None:
            prepare(devices)

    return await _cell(dialect, anchors=anchors, prepare=setup)


async def anchors_report(ext, ids=None, **stats):
    """Each anchor of ``ids`` (default: all four) sends its health report,
    advertising survey 1 unless ``stats`` say otherwise."""
    for system_id in [a[0] for a in ANCHORS] if ids is None else ids:
        healthy = HEALTHY_A0 if system_id == 201 else HEALTHY_A1
        await ext._on_stats(system_id, {**healthy, **stats}, time.monotonic())


async def tag_reports(ext, *entries, system_id=TAG, **stats):
    """The tag sends one health report per ``antbl`` entry (one report
    naming A0 without any)."""
    for entry in entries or (TAG_STATS["antbl"],):
        report = {**TAG_STATS, "antbl": entry, **stats}
        await ext._on_stats(system_id, report, time.monotonic())


async def inf(ext, builder, hub):
    """The body of the answer to X-RTLS-INF."""
    message = make_message(builder, {"type": "X-RTLS-INF"})
    return (await ext._handle_RTLS_INF(message, None, hub)).body


def anchor_table(ext, system_id=TAG):
    return ext._stats[system_id].get("anchorTable")


def record_requests(device, dialect):
    """Every parameter the server writes to or reads from ``device`` by
    name from now on, as ``(message type, name)``."""
    parser = dialect.MAVLink(None)
    parser.robust_parsing = True
    seen = []
    handle = device.handle

    def handled(data):
        for message in parser.parse_buffer(data) or []:
            if message.get_type() in ("PARAM_EXT_SET", "PARAM_EXT_REQUEST_READ"):
                seen.append((message.get_type(), message.param_id))
        return handle(data)

    device.handle = handled
    return seen
