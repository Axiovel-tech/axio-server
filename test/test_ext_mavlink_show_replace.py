"""Tests that every show upload, `.skyc` and `.gdm` alike, first removes the
show loaded on the drone and writes nothing if the drone refuses.
"""

import json
from pathlib import Path
from types import SimpleNamespace

from pytest import fixture, raises

from flockwave.server.ext.mavlink import driver as driver_module
from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.gdm_upload import GDMB_LIVE_PATH, GDMB_TEMP_PATH

VECTORS = Path(__file__).parent / "fixtures" / "gdm-vectors"
FLAT = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()

CONFIG = {
    "coordinateSystem": {
        "type": "nwu",
        "origin": [18.9151, 47.4863],
        "orientation": "0",
    },
    "amslReference": None,
    "geofence": {"version": 1, "enabled": True, "polygons": [], "rallyPoints": []},
}

SKYC_SHOW = {
    **CONFIG,
    "trajectory": {
        "version": 1,
        "points": [[0, [0, 0, 0], []], [5, [0, 0, 1.5], []], [10, [0, 0, 0], []]],
    },
    "lights": {"version": 1, "data": "AAE="},
}


class RecordingFTP:
    """Records the MAVFTP operations of an upload in a shared call log."""

    def __init__(self, calls: list[tuple]):
        self.calls = calls
        self.files: dict[str, bytes] = {}

    async def aclose(self):
        pass

    async def mkdir(self, path, parents=False, exist_ok=False):
        self.calls.append(("mkdir", path))

    async def put(self, data, remote_path, *, parents=False, skip_crc_check=False):
        self.calls.append(("put", remote_path))
        self.files[remote_path] = bytes(data)

    async def rm(self, path):
        self.calls.append(("rm", path))
        self.files.pop(path, None)

    async def rename(self, old_path, new_path):
        self.calls.append(("rename", old_path, new_path))
        self.files[new_path] = self.files.pop(old_path)

    async def crc32(self, path):
        from flockwave.server.show.utils import crc32_mavftp

        return crc32_mavftp(self.files[path])


@fixture
def calls() -> list[tuple]:
    return []


@fixture
def uav(calls, monkeypatch) -> MAVLinkUAV:
    uav = MAVLinkUAV("01", driver=SimpleNamespace(autopilot_factory=None))

    async def remove_show():
        calls.append(("remove",))

    async def configure_and_reload(coordinate_system, altitude_reference, geofence):
        calls.append(("configure+reload",))

    monkeypatch.setattr(uav, "remove_show", remove_show)
    monkeypatch.setattr(uav, "_configure_show_frame_and_reload", configure_and_reload)
    monkeypatch.setattr(
        driver_module.MAVFTP, "for_uav", classmethod(lambda cls, _: RecordingFTP(calls))
    )
    return uav


def refuse_remove(uav: MAVLinkUAV, calls: list[tuple], monkeypatch) -> None:
    async def remove_show():
        calls.append(("remove",))
        raise RuntimeError("Failed to remove show file")

    monkeypatch.setattr(uav, "remove_show", remove_show)


async def test_gdm_upload_removes_loaded_show_first(uav, calls):
    await uav.upload_gdm_show(FLAT, json.loads(json.dumps(CONFIG)))

    assert calls == [
        ("remove",),
        ("mkdir", "/axio"),
        ("put", GDMB_TEMP_PATH),
        ("rm", GDMB_LIVE_PATH),
        ("rename", GDMB_TEMP_PATH, GDMB_LIVE_PATH),
        ("configure+reload",),
    ]


async def test_skyc_upload_removes_loaded_show_first(uav, calls):
    await uav.upload_show(json.loads(json.dumps(SKYC_SHOW)))

    assert calls == [
        ("remove",),
        ("put", "/collmot/show.skyb"),
        ("configure+reload",),
    ]


async def test_gdm_upload_writes_nothing_if_remove_fails(uav, calls, monkeypatch):
    refuse_remove(uav, calls, monkeypatch)

    with raises(RuntimeError, match="refused to remove the loaded show"):
        await uav.upload_gdm_show(FLAT, json.loads(json.dumps(CONFIG)))

    assert calls == [("remove",)]


async def test_skyc_upload_writes_nothing_if_remove_fails(uav, calls, monkeypatch):
    refuse_remove(uav, calls, monkeypatch)

    with raises(RuntimeError, match="refused to remove the loaded show"):
        await uav.upload_show(json.loads(json.dumps(SKYC_SHOW)))

    assert calls == [("remove",)]


async def test_invalid_config_fails_before_removing_the_show(uav, calls):
    config = {**CONFIG, "coordinateSystem": None}

    with raises(RuntimeError, match="coordinate system"):
        await uav.upload_gdm_show(FLAT, config)

    assert calls == []
