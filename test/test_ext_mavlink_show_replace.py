"""Tests that every show upload, `.skyc` and `.gdm` alike, writes the drone's
single show slot `/axio/show.bin`: only on firmware that advertises the slot,
only after the loaded show was removed, and nothing at all if either check
fails.
"""

import json
from pathlib import Path
from types import SimpleNamespace

from pytest import fixture, mark, raises
from trio import TooSlowError

from flockwave.server.ext.mavlink import driver as driver_module
from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.show_slot import SHOW_SLOT_PATH, SHOW_SLOT_TEMP_PATH

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


SLOT_WRITE = [
    ("mkdir", "/axio"),
    ("put", SHOW_SLOT_TEMP_PATH),
    ("rm", SHOW_SLOT_PATH),
    ("rename", SHOW_SLOT_TEMP_PATH, SHOW_SLOT_PATH),
]


@fixture
def calls() -> list[tuple]:
    return []


@fixture
def ftp(calls) -> RecordingFTP:
    return RecordingFTP(calls)


@fixture
def uav(calls, ftp, monkeypatch) -> MAVLinkUAV:
    uav = MAVLinkUAV("01", driver=SimpleNamespace(autopilot_factory=None))

    async def get_parameter(name, fetch=False):
        calls.append(("check", name))
        return 1.0

    async def remove_show():
        calls.append(("remove",))

    async def configure_and_reload(coordinate_system, altitude_reference, geofence):
        calls.append(("configure+reload",))

    monkeypatch.setattr(uav, "get_parameter", get_parameter)
    monkeypatch.setattr(uav, "remove_show", remove_show)
    monkeypatch.setattr(uav, "_configure_show_frame_and_reload", configure_and_reload)
    monkeypatch.setattr(
        driver_module.MAVFTP, "for_uav", classmethod(lambda cls, _: ftp)
    )
    return uav


def refuse_remove(uav: MAVLinkUAV, calls: list[tuple], monkeypatch) -> None:
    async def remove_show():
        calls.append(("remove",))
        raise RuntimeError("Failed to remove show file")

    monkeypatch.setattr(uav, "remove_show", remove_show)


def report_slot_version(uav: MAVLinkUAV, calls: list[tuple], monkeypatch, value):
    """Makes the drone report `value` for SHOW_SLOT_VER; `None` means that
    the drone never answers, as for a parameter it does not have.
    """

    async def get_parameter(name, fetch=False):
        calls.append(("check", name))
        if value is None:
            raise TooSlowError()
        return value

    monkeypatch.setattr(uav, "get_parameter", get_parameter)


async def upload(uav: MAVLinkUAV, kind: str) -> None:
    if kind == "gdm":
        await uav.upload_gdm_show(FLAT, json.loads(json.dumps(CONFIG)))
    else:
        await uav.upload_show(json.loads(json.dumps(SKYC_SHOW)))


@mark.parametrize("kind", ["gdm", "skyc"])
async def test_upload_checks_removes_writes_slot_then_reloads(uav, calls, kind):
    await upload(uav, kind)

    assert calls == [
        ("check", "SHOW_SLOT_VER"),
        ("remove",),
        *SLOT_WRITE,
        ("configure+reload",),
    ]


async def test_both_formats_land_in_the_same_slot(uav, ftp):
    await upload(uav, "gdm")
    assert ftp.files == {SHOW_SLOT_PATH: FLAT}

    await upload(uav, "skyc")
    assert list(ftp.files) == [SHOW_SLOT_PATH]
    assert ftp.files[SHOW_SLOT_PATH].startswith(b"skyb\x02")


@mark.parametrize("kind", ["gdm", "skyc"])
@mark.parametrize("version", [None, 0.0, 2.0])
async def test_upload_refused_without_show_slot_support(
    uav, calls, monkeypatch, kind, version
):
    report_slot_version(uav, calls, monkeypatch, version)

    with raises(RuntimeError, match="does not support the /axio show slot"):
        await upload(uav, kind)

    # Nothing reached the drone beyond the parameter read
    assert calls == [("check", "SHOW_SLOT_VER")]


@mark.parametrize("kind", ["gdm", "skyc"])
async def test_upload_writes_nothing_if_remove_fails(uav, calls, monkeypatch, kind):
    refuse_remove(uav, calls, monkeypatch)

    with raises(RuntimeError, match="refused to remove the loaded show"):
        await upload(uav, kind)

    assert calls == [("check", "SHOW_SLOT_VER"), ("remove",)]


async def test_invalid_config_fails_before_touching_the_drone(uav, calls):
    config = {**CONFIG, "coordinateSystem": None}

    with raises(RuntimeError, match="coordinate system"):
        await uav.upload_gdm_show(FLAT, config)

    assert calls == []
