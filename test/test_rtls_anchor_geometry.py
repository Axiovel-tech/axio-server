"""Anchor-owned geometry: an anchor's position is the one it stores itself
(``UWB_LOC``), a tag holds no anchor list and reports the table it builds
from the air, and nothing of the tags' own cell fit is left."""

import importlib.util
import logging
import socket
import time
from contextlib import contextmanager

import pytest
import trio
from anchor_lock_support import HEALTHY_A1, location
from rtls_support import (
    FakeDevice,
    StubBeaconAPI,
    StubMessageHub,
    discover,
    make_message,
    set_fake_param,
)
from rtlslink import Origin, ned_to_global_e7
from rtlslink.dialect import load_dialect
from survey_support import TWO_TIERS, commit, measure, survey_cell
from survey_view_support import (
    NO_POSITION,
    ORIGIN,
    OTHER_SURVEY,
    TAG,
    TAG_STATS,
    UNKNOWN_LAYOUT,
    anchor_table,
    anchors_report,
    antbl,
    give_origin,
    inf,
    record_requests,
    surveyed_cell,
    tag_reports,
)

from flockwave.server.ext.rtls.extension import (
    LOCATION_READ_INTERVAL,
    RtlsExtension,
    _stats_json,
)
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.builders import FlockwaveMessageBuilder


@pytest.fixture(scope="module")
def dialect():
    return load_dialect()


@pytest.fixture
def hub():
    return MessageHub()


@pytest.fixture
def builder():
    return FlockwaveMessageBuilder()


@pytest.fixture
def autojump_clock():
    return trio.testing.MockClock(autojump_threshold=0)


def _on_map(north, east, down):
    lat_e7, lon_e7, alt_mm = ned_to_global_e7(
        Origin(*ORIGIN.values()), north, east, down
    )
    return {
        "lat": pytest.approx(lat_e7 * 1e-7),
        "lon": pytest.approx(lon_e7 * 1e-7),
        "amsl": pytest.approx(alt_mm * 1e-3),
    }


async def _anchors(ext, builder, hub):
    return {a["id"]: a for a in (await inf(ext, builder, hub))["anchors"]}


# ---- X-RTLS-INF anchors ----------------------------------------------------


async def test_an_anchor_is_listed_with_the_position_it_stores(dialect, builder, hub):
    ext, _ = await surveyed_cell(dialect)

    anchors = await _anchors(ext, builder, hub)

    assert list(anchors) == [f"rtls::default::anchor_{row}" for row in range(4)]
    assert anchors["rtls::default::anchor_2"] == {
        "id": "rtls::default::anchor_2",
        "cell": "default",
        "index": 2,
        "mac": 3,
        "position": _on_map(4.0, 3.0, -2.5),
        "ned": {"north": 4.0, "east": 3.0, "down": -2.5},
        "surveyId": 1,
        "planar": False,
        "active": True,
        "systemId": 203,
    }


async def test_the_planar_flag_and_survey_id_are_the_stored_ones(dialect, builder, hub):
    def prepare(devices):
        stored = location(1.0, 2.0, 0.0, 0x0102, planar=True, reference_mm=4567)
        set_fake_param(devices[202], "UWB_LOC", stored, "custom")

    ext, _ = await surveyed_cell(dialect, prepare=prepare)

    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]

    assert (anchor["surveyId"], anchor["planar"]) == (0x0102, True)
    assert anchor["ned"] == {"north": 1.0, "east": 2.0, "down": 0.0}


async def test_a_blank_anchor_is_listed_without_a_position(dialect, builder, hub):
    def prepare(devices):
        # what a tag of the old firmware still holds says nothing of an anchor
        for name, value in (("X", 7.0), ("Y", 8.0), ("Z", -1.0)):
            set_fake_param(devices[TAG], f"UWB_AN1_{name}", value, "real32")
        set_fake_param(devices[TAG], "UWB_AN1_MAC", 2, "uint16")
        set_fake_param(devices[TAG], "UWB_AN_COUNT", 4, "uint8")

    ext, _ = await surveyed_cell(dialect, blank=(202,), prepare=prepare)

    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]

    assert anchor["ned"] is None
    assert anchor["position"] is None
    assert (anchor["surveyId"], anchor["planar"]) == (None, False)
    assert (anchor["active"], anchor["systemId"], anchor["mac"]) == (True, 202, 2)


async def test_a_value_that_is_no_location_is_no_position(dialect, builder, hub):
    def prepare(devices):
        set_fake_param(devices[202], "UWB_LOC", "not a location", "custom")

    ext, _ = await surveyed_cell(dialect, prepare=prepare)

    assert (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]["ned"] is None


async def test_anchors_are_listed_without_any_tag(dialect, builder, hub):
    ext, devices = await surveyed_cell(dialect)
    ext._protocol.devices[TAG].last_seen = -1e9
    ext._expire_devices(time.monotonic())

    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]

    # the frame is the anchors' own; only a tag says where on the map it is
    assert anchor["ned"] == {"north": 4.0, "east": 0.0, "down": 0.0}
    assert anchor["position"] is None


async def test_the_lowest_tag_of_the_cell_places_it_on_the_map(dialect, builder, hub):
    ext, _ = await surveyed_cell(dialect)
    other = FakeDevice(dialect, system_id=7)
    other.address = ("192.168.4.7", 3333)
    set_fake_param(other, "UWB_ROLE", 1, "uint8")
    give_origin(other, ORIGIN_LAT_E7=ORIGIN["ORIGIN_LAT_E7"] + 1000)
    ext.routes[other.address] = other
    await discover(ext, other)

    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_0"]

    assert anchor["position"]["lat"] == pytest.approx(41.3901)


async def test_a_tag_of_another_cell_does_not_place_the_anchors(dialect, builder, hub):
    def prepare(devices):
        set_fake_param(devices[TAG], "CELL_ID", "hall-b", "custom")

    ext, _ = await surveyed_cell(dialect, prepare=prepare)

    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_0"]

    assert anchor["ned"] is not None and anchor["position"] is None


async def test_an_anchor_in_no_row_of_its_table_is_listed_by_system_id(
    dialect, builder, hub
):
    def prepare(devices):
        set_fake_param(devices[204], "UWB_MAC", 99, "uint16")

    ext, _ = await surveyed_cell(dialect, prepare=prepare)

    anchors = await _anchors(ext, builder, hub)

    assert list(anchors)[-1] == "rtls::default::device_204"
    assert anchors["rtls::default::device_204"]["index"] is None
    assert anchors["rtls::default::device_204"]["mac"] == 99


async def test_a_lost_anchor_stays_listed_inactive_until_it_is_forgotten(
    dialect, builder, hub
):
    ext, _ = await surveyed_cell(dialect)
    ext._beacon_api = api = StubBeaconAPI()
    ext._sync_anchor_beacons()
    assert api.beacons["rtls::default::anchor_1"].active is True

    ext._protocol.devices[202].last_seen = -1e9
    ext._expire_devices(time.monotonic())

    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]
    assert (anchor["active"], anchor["systemId"]) == (False, None)
    assert anchor["ned"] == {"north": 4.0, "east": 0.0, "down": 0.0}
    assert api.beacons["rtls::default::anchor_1"].active is False

    assert ext.forget_device(202) == {"forgotten": True}
    assert "rtls::default::anchor_1" not in await _anchors(ext, builder, hub)
    assert "rtls::default::anchor_1" not in api.beacons


# ---- the map beacons -------------------------------------------------------


async def test_anchors_with_a_map_position_are_beacons(dialect):
    ext, _ = await surveyed_cell(dialect, blank=(204,))
    ext._beacon_api = api = StubBeaconAPI()

    ext._sync_anchor_beacons()

    assert set(api.beacons) == {f"rtls::default::anchor_{row}" for row in range(3)}
    beacon = api.beacons["rtls::default::anchor_2"]
    assert beacon.basic_properties.name == "RTLS A2"
    assert beacon.active is True
    on_map = _on_map(4.0, 3.0, -2.5)
    assert (beacon.position.lat, beacon.position.lon) == (on_map["lat"], on_map["lon"])
    assert beacon.position.amsl == on_map["amsl"]


async def test_the_beacons_go_with_the_tag_that_placed_them(dialect):
    ext, _ = await surveyed_cell(dialect)
    ext._beacon_api = api = StubBeaconAPI()
    ext._sync_anchor_beacons()
    assert len(api.beacons) == 4

    ext._protocol.devices[TAG].last_seen = -1e9
    ext._expire_devices(time.monotonic())

    assert api.beacons == {}


async def test_a_written_position_moves_the_anchor_and_is_pushed(dialect, builder, hub):
    ext, devices = await surveyed_cell(dialect)
    ext._beacon_api = api = StubBeaconAPI()
    ext._sync_anchor_beacons()
    ext._inf_pending = False

    moved = location(5.0, 1.0, -0.5, 2)
    ack = await ext.set_param(202, "UWB_LOC", moved, "custom")

    assert ack["accepted"] is True
    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]
    assert anchor["ned"] == {"north": 5.0, "east": 1.0, "down": -0.5}
    assert anchor["surveyId"] == 2
    beacon = api.beacons["rtls::default::anchor_1"]
    assert beacon.position.lat == _on_map(5.0, 1.0, -0.5)["lat"]
    assert ext._inf_pending is True


async def test_a_position_written_past_the_server_is_read_again(dialect, builder, hub):
    ext, devices = await surveyed_cell(dialect, blank=(202,))
    await anchors_report(ext, [202], sid=0.0)
    requests = record_requests(devices[202], dialect)
    assert requests == []

    # another client commits survey 7 to the anchor
    set_fake_param(devices[202], "UWB_LOC", location(9.0, 9.0, -1.0, 7), "custom")
    ext._inf_pending = False
    await anchors_report(ext, [202], sid=7.0)

    assert requests == [("PARAM_EXT_REQUEST_READ", "UWB_LOC")]
    anchor = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]
    assert anchor["ned"] == {"north": 9.0, "east": 9.0, "down": -1.0}
    assert anchor["surveyId"] == 7
    assert ext._inf_pending is True

    await anchors_report(ext, [202], sid=7.0)
    assert len(requests) == 1


async def test_a_lost_read_of_the_position_is_asked_again_at_a_bounded_rate(
    dialect, builder, hub
):
    ext, devices = await surveyed_cell(dialect)
    anchor = devices[202]
    anchor.respond_to_read = False
    requests = record_requests(anchor, dialect)

    async def reports(at, sid=7.0):
        await ext._on_stats(202, {**HEALTHY_A1, "sid": sid}, at)
        return len(requests)

    assert await reports(100.0) == 1
    assert await reports(101.0) == 1
    assert await reports(100.0 + LOCATION_READ_INTERVAL - 0.1) == 1
    # the answer was lost: the anchor still advertises what is not cached
    assert await reports(100.0 + LOCATION_READ_INTERVAL) == 2
    # no session runs through a restart or a sleep: that is not another survey
    assert await reports(120.0, sid=-1.0) == 2
    # another survey is asked for at once
    assert await reports(121.0, sid=8.0) == 3

    set_fake_param(anchor, "UWB_LOC", location(9.0, 9.0, -1.0, 8), "custom")
    anchor.respond_to_read = True
    assert await reports(121.0 + LOCATION_READ_INTERVAL, sid=8.0) == 4
    entry = (await _anchors(ext, builder, hub))["rtls::default::anchor_1"]
    assert (entry["surveyId"], entry["ned"]["north"]) == (8, 9.0)
    # the cache has it: nothing more is asked
    assert await reports(500.0, sid=8.0) == 4


async def test_a_tags_survey_id_does_not_make_the_server_read_a_position(dialect):
    ext, devices = await surveyed_cell(dialect)
    requests = record_requests(devices[TAG], dialect)

    await tag_reports(ext, sid=7.0)

    assert requests == []


# ---- X-RTLS-STATS ----------------------------------------------------------


def test_an_anchors_survey_stats():
    body = _stats_json(201, {"sid": 12.0, "rdev": -0.0834}, "anchor-responder")
    assert body["surveyId"] == 12
    assert body["rangeDeviationM"] == -0.0834

    # no reading is no field: it never means "in place"
    assert "rangeDeviationM" not in _stats_json(201, {"sid": 12.0}, "anchor-responder")
    for unsurveyed in (0.0, -1.0):
        body = _stats_json(201, {"sid": unsurveyed}, "anchor-initiator")
        assert body["surveyId"] is None


def test_a_tags_survey_stats():
    body = _stats_json(TAG, {**TAG_STATS, "anexc": 2.0, "sdrift": 0.03456})

    assert body["surveyId"] == 1
    assert (body["anchorsUsed"], body["anchorsExcluded"]) == (4, 2)
    assert body["surveyDriftM"] == 0.0346
    assert _stats_json(TAG, {**TAG_STATS, "sid": 0.0})["surveyId"] is None


def test_a_tag_without_the_table_stats_reports_none_of_them():
    old = {k: v for k, v in TAG_STATS.items() if k in ("rate", "solvepct", "agems")}

    body = _stats_json(TAG, old)

    for key in ("surveyId", "anchorsUsed", "anchorsExcluded", "surveyDriftM"):
        assert key not in body


def test_the_stats_of_the_tags_own_fit_are_gone():
    fit = {"geom": 3.0, "gfail": 0.0, "gres": -0.02, "gdrift": 0.001}
    fit |= {f"gd{i}": 4.0 + i for i in range(1, 8)}

    body = _stats_json(TAG, {**TAG_STATS, **fit})

    assert not [key for key in body if key.lower().startswith("geometry")]


async def test_a_tags_report_reaches_the_stats_as_the_firmware_sends_it(dialect):
    ext, devices = await surveyed_cell(dialect)
    tag = devices[TAG]

    def report(entry, boot_ms):
        """A whole health report in one datagram."""
        stats = {**TAG_STATS, "antbl": entry}
        return b"".join(
            tag.named_value_float(name, value, boot_ms) for name, value in stats.items()
        )

    for boot_ms, entry in (
        (500, antbl(1, row=0)),
        (1000, antbl(4, state=OTHER_SURVEY)),
    ):
        await ext._process_datagram(
            report(entry, boot_ms), tag.address, time.monotonic()
        )

    stats = ext._stats[TAG]
    assert (stats["surveyId"], stats["anchorsUsed"], stats["anchorsExcluded"]) == (
        1,
        4,
        0,
    )
    assert stats["surveyDriftM"] == 0.02
    assert stats["anchorTable"] == [
        {"row": 0, "mac": 1, "state": "in-use", "systemId": 201},
        {"row": None, "mac": 4, "state": "other-survey", "systemId": 204},
    ]


async def test_the_anchor_table_is_assembled_over_the_rotation(dialect):
    ext, _ = await surveyed_cell(dialect)

    await tag_reports(ext, antbl(2, row=1))
    assert anchor_table(ext) == [
        {"row": 1, "mac": 2, "state": "in-use", "systemId": 202}
    ]

    await tag_reports(
        ext,
        antbl(0x0777, state=UNKNOWN_LAYOUT),
        antbl(4, state=OTHER_SURVEY),
        antbl(1, row=0),
        antbl(3, row=2),
    )

    # the rows first, by row; then the anchors left out; an anchor that is
    # no known device has no system ID
    assert anchor_table(ext) == [
        {"row": 0, "mac": 1, "state": "in-use", "systemId": 201},
        {"row": 1, "mac": 2, "state": "in-use", "systemId": 202},
        {"row": 2, "mac": 3, "state": "in-use", "systemId": 203},
        {"row": None, "mac": 4, "state": "other-survey", "systemId": 204},
        {"row": None, "mac": 0x0777, "state": "unknown-layout", "systemId": None},
    ]


async def test_a_row_names_an_anchor_the_server_lost(dialect):
    ext, _ = await surveyed_cell(dialect)
    ext._protocol.devices[202].last_seen = -1e9
    ext._expire_devices(time.monotonic())

    await tag_reports(ext, antbl(2, row=1))

    # the tag still hears an anchor whose management link dropped
    assert anchor_table(ext)[0]["systemId"] == 202


async def test_an_entry_not_named_for_two_rotations_is_dropped(dialect):
    ext, _ = await surveyed_cell(dialect)
    rotation = [antbl(1, row=0), antbl(2, row=1), antbl(3, row=2)]
    await tag_reports(ext, *rotation, antbl(4, row=3))

    def macs():
        return [entry["mac"] for entry in anchor_table(ext)]

    # anchor 4 leaves the tag's tracker: two rotations of the four held
    await tag_reports(ext, *(rotation * 3)[:8])
    assert macs() == [1, 2, 3, 4]
    await tag_reports(ext, rotation[2])
    assert macs() == [1, 2, 3]

    # an anchor named once per rotation stays, however long
    await tag_reports(ext, *rotation * 5)
    assert macs() == [1, 2, 3]


async def test_an_anchor_that_changes_state_or_takes_a_row_replaces_the_old(dialect):
    ext, _ = await surveyed_cell(dialect)
    await tag_reports(ext, antbl(1, row=0), antbl(2, row=1))

    await tag_reports(ext, antbl(2, state=NO_POSITION), antbl(3, row=0))

    assert anchor_table(ext) == [
        {"row": 0, "mac": 3, "state": "in-use", "systemId": 203},
        {"row": None, "mac": 2, "state": "no-position", "systemId": 202},
    ]


async def test_a_tag_that_tracks_no_anchor_has_an_empty_table(dialect):
    ext, _ = await surveyed_cell(dialect)
    await tag_reports(ext, antbl(1, row=0), antbl(2, row=1))

    await tag_reports(ext, 0.0, sid=0.0, anuse=0.0)

    assert anchor_table(ext) == []
    # a value that is no entry leaves the table as it is
    await tag_reports(ext, antbl(1, row=0), antbl(9, state=6))
    assert [entry["mac"] for entry in anchor_table(ext)] == [1]


async def test_a_tag_that_reports_no_table_has_none(dialect):
    ext, _ = await surveyed_cell(dialect)
    await tag_reports(ext, antbl(1, row=0))
    without = {k: v for k, v in TAG_STATS.items() if k != "antbl"}

    await ext._on_stats(TAG, without, time.monotonic())

    assert "anchorTable" not in ext._stats[TAG]
    # and what it reports next starts a new table
    await tag_reports(ext, antbl(2, row=1))
    assert [entry["mac"] for entry in anchor_table(ext)] == [2]


async def test_an_anchor_reports_no_table(dialect):
    ext, _ = await surveyed_cell(dialect)

    await anchors_report(ext)

    assert "anchorTable" not in ext._stats[201]


# ---- nothing of the tags' own fit is left ----------------------------------


class _RecordingHub(StubMessageHub):
    def __init__(self):
        super().__init__()
        self.handlers = {}

    @contextmanager
    def use_message_handlers(self, handlers):
        self.handlers = dict(handlers)
        yield


class _App:
    def __init__(self):
        self.message_hub = _RecordingHub()

    def import_api(self, name):
        return StubBeaconAPI()


def _free_udp_port():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


async def test_the_geometry_message_is_not_handled_any_more():
    ext = RtlsExtension()
    ext.app = app = _App()
    config = {"listen_port": _free_udp_port()}

    async with trio.open_nursery() as nursery:
        nursery.start_soon(ext.run, app, config, logging.getLogger("test-rtls"))
        with trio.fail_after(5):
            while not app.message_hub.handlers:
                await trio.sleep(0.01)
        nursery.cancel_scope.cancel()

    handled = set(app.message_hub.handlers)
    assert "X-RTLS-GEOM" not in handled
    assert {"X-RTLS-INF", "X-RTLS-STATS", "X-RTLS-VERIFY", "X-RTLS-SURVEY"} <= handled


@pytest.mark.parametrize("module", ["geometry_agreement", "fitted_cell"])
def test_the_modules_of_the_tags_own_fit_are_gone(module):
    assert importlib.util.find_spec(f"flockwave.server.ext.rtls.{module}") is None


async def test_no_anchor_table_is_written_to_or_read_from_a_tag(
    dialect, nursery, builder, hub, autojump_clock
):
    cell = await survey_cell(dialect, nursery)
    requests = record_requests(cell.devices[TAG], dialect)

    message = make_message(builder, {"type": "X-RTLS-CELL", "rateHz": 83})
    applied = (await cell.ext._handle_RTLS_CELL(message, None, hub)).body
    await cell.report(cellcfg=HEALTHY_A1["cellcfg"])
    survey = await measure(cell, builder, hub, frame=TWO_TIERS)
    answer = await commit(cell, builder, hub)

    assert applied["result"] == "applied", applied["reason"]
    assert (survey["state"], answer["result"]) == ("ready", "committed")
    assert requests == [("PARAM_EXT_SET", "UWB_BLOCK_MS")]
    # and the anchors' positions went to the anchors
    assert {system_id for system_id, _ in cell.locations} == set(cell.positions)
