"""Tests that every show upload, `.skyc` and `.gdm` alike, writes the drone's
single show slot `/axio/show.bin` with the flow of AxioStack#89 decision 1:
refuse if armed, check SHOW_SLOT_VER, remove the legacy file, put the slot
(which truncates it), then configure and reload. Nothing is written if a check
fails.
"""

import json
import logging
from pathlib import Path
from types import SimpleNamespace

from flockwave.spec.errors import FlockwaveErrorCode
from pytest import fixture, mark, raises
from trio import TooSlowError

from flockwave.server.ext.mavlink import driver as driver_module
from flockwave.server.ext.mavlink.autopilots import ArduPilot
from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.enums import MAVModeFlag, MAVParamError
from flockwave.server.ext.mavlink.errors import ParameterNotFoundError
from flockwave.server.ext.mavlink.ftp import (
    MAVFTPErrorCode,
    OperationNotAcknowledgedError,
)
from flockwave.server.ext.mavlink.show_slot import (
    LEGACY_SHOW_PATH,
    SHOW_SLOT_PATH,
    ShowUploadRefusedError,
)

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
        #: NAK that every rm and mkdir gets instead of its own reply
        self.nak: OperationNotAcknowledgedError | None = None

    async def aclose(self):
        pass

    async def mkdir(self, path, parents=False, exist_ok=False):
        self.calls.append(("mkdir", path))
        if self.nak:
            raise self.nak

    async def put(self, data, remote_path, *, parents=False, skip_crc_check=False):
        self.calls.append(("put", remote_path))
        self.files[remote_path] = bytes(data)

    async def rm(self, path):
        self.calls.append(("rm", path))
        if self.nak:
            raise self.nak
        if path not in self.files:
            raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_NOT_FOUND)
        del self.files[path]


SLOT_WRITE = [
    ("rm", LEGACY_SHOW_PATH),
    ("mkdir", "/axio"),
    ("put", SHOW_SLOT_PATH),
]


MISSING = "missing"
"""A parameter the drone answers with PARAM_ERROR DOES_NOT_EXIST."""


class FakeDrone:
    """What the tested UAV reads from its drone: the latest heartbeat and
    parameters. A parameter mapped to `None` gets no reply, like a parameter
    that firmware without PARAM_ERROR does not have, or a lost packet.
    """

    def __init__(self, calls: list[tuple]):
        self.calls = calls
        self.armed = False
        self.heartbeat = True
        self.parameters: dict[str, float | str | None] = {
            "SHOW_SLOT_VER": 1.0,
            "SHOW_ORIENTATION": 1.0,
        }
        #: Number of upcoming reads of each parameter that get no reply
        self.lost: dict[str, int] = {}

    def get_last_message(self, type):
        if not self.heartbeat:
            return None
        mode = MAVModeFlag.SAFETY_ARMED.value if self.armed else 0
        return SimpleNamespace(base_mode=mode)

    async def get_parameter(self, name, fetch=False):
        self.calls.append(("param", name))
        if self.lost.get(name):
            self.lost[name] -= 1
            raise TooSlowError()
        value = self.parameters.get(name)
        if value is None:
            raise TooSlowError()
        if value == MISSING:
            raise ParameterNotFoundError(name)
        return value


@fixture
def calls() -> list[tuple]:
    return []


@fixture
def ftp(calls) -> RecordingFTP:
    return RecordingFTP(calls)


@fixture
def drone(calls) -> FakeDrone:
    return FakeDrone(calls)


@fixture
def uav(calls, ftp, drone, monkeypatch) -> MAVLinkUAV:
    uav = MAVLinkUAV(
        "01",
        driver=SimpleNamespace(autopilot_factory=None, log=logging.getLogger("t")),
    )
    uav.notify_updated = lambda: None

    async def remove_show():
        calls.append(("remove",))

    async def configure_and_reload(coordinate_system, altitude_reference, geofence):
        calls.append(("configure+reload",))

    monkeypatch.setattr(uav, "get_last_message", drone.get_last_message)
    monkeypatch.setattr(uav, "get_parameter", drone.get_parameter)
    monkeypatch.setattr(uav, "remove_show", remove_show)
    monkeypatch.setattr(uav, "_configure_show_frame_and_reload", configure_and_reload)
    monkeypatch.setattr(
        driver_module.MAVFTP, "for_uav", classmethod(lambda cls, _, **_kwargs: ftp)
    )
    return uav


async def upload(uav: MAVLinkUAV, kind: str) -> None:
    if kind == "gdm":
        await uav.upload_gdm_show(FLAT, json.loads(json.dumps(CONFIG)))
    else:
        await uav.upload_show(json.loads(json.dumps(SKYC_SHOW)))


KINDS = mark.parametrize("kind", ["gdm", "skyc"])


@KINDS
async def test_upload_flow(uav, calls, kind):
    await upload(uav, kind)

    # No REMOVE_SHOW command: the MAVFTP removals replace it
    assert calls == [
        ("param", "SHOW_SLOT_VER"),
        *SLOT_WRITE,
        ("configure+reload",),
    ]


async def test_both_formats_land_in_the_same_slot(uav, ftp):
    ftp.files[LEGACY_SHOW_PATH] = b"skyb from the old server"

    await upload(uav, "gdm")
    assert ftp.files == {SHOW_SLOT_PATH: FLAT}

    await upload(uav, "skyc")
    assert list(ftp.files) == [SHOW_SLOT_PATH]
    assert ftp.files[SHOW_SLOT_PATH].startswith(b"skyb\x02")


@KINDS
async def test_upload_refused_while_armed(uav, calls, drone, kind):
    drone.armed = True

    with raises(ShowUploadRefusedError, match="armed") as info:
        await upload(uav, kind)

    assert info.value.code == "E_ARMED"
    assert calls == []


async def test_upload_refused_if_the_drone_arms_during_the_version_read(
    uav, calls, drone, monkeypatch
):
    read = drone.get_parameter

    async def arm_while_reading(name, fetch=False):
        drone.armed = True
        return await read(name, fetch)

    monkeypatch.setattr(uav, "get_parameter", arm_while_reading)

    with raises(ShowUploadRefusedError, match="armed") as info:
        await upload(uav, "gdm")

    assert info.value.code == "E_ARMED"
    assert calls == [("param", "SHOW_SLOT_VER")]


async def test_upload_refused_without_heartbeat(uav, calls, drone):
    drone.heartbeat = False

    with raises(RuntimeError, match="No heartbeat") as info:
        await upload(uav, "gdm")

    # Retryable: once the drone is heard from, the upload may go ahead
    assert not isinstance(info.value, ShowUploadRefusedError)
    assert calls == []


@KINDS
async def test_firmware_without_show_slot_is_refused_for_good(uav, calls, drone, kind):
    drone.parameters["SHOW_SLOT_VER"] = None

    with raises(ShowUploadRefusedError, match="firmware is too old") as info:
        await upload(uav, kind)

    assert info.value.code == "E_SHOW_SLOT_UNSUPPORTED"
    # SHOW_ORIENTATION answered, so the link works, and SHOW_SLOT_VER still
    # did not: the parameter is missing
    assert calls == [
        ("param", "SHOW_SLOT_VER"),
        ("param", "SHOW_ORIENTATION"),
        ("param", "SHOW_SLOT_VER"),
    ]


async def test_param_error_for_the_show_slot_is_refused_at_once(uav, calls, drone):
    drone.parameters["SHOW_SLOT_VER"] = MISSING

    with raises(ShowUploadRefusedError, match="firmware is too old"):
        await upload(uav, "gdm")

    assert calls == [("param", "SHOW_SLOT_VER")]


async def test_a_lost_show_slot_reply_does_not_refuse_the_upload(uav, calls, drone):
    """Bench 2026-11-02, drone 1: one SHOW_SLOT_VER read went unanswered
    while SHOW_ORIENTATION was answered."""
    drone.lost["SHOW_SLOT_VER"] = 1

    await upload(uav, "gdm")

    assert calls[:3] == [
        ("param", "SHOW_SLOT_VER"),
        ("param", "SHOW_ORIENTATION"),
        ("param", "SHOW_SLOT_VER"),
    ]
    assert calls[3:] == [*SLOT_WRITE, ("configure+reload",)]
    assert INCOMPATIBLE not in uav.status.errors


@mark.parametrize("errno", [0, 34])
async def test_upload_survives_naks_whose_errno_was_overwritten(uav, ftp, errno):
    """Bench 2026-11-02, drone 8: REMOVE_FILE of a missing file came back as
    FAIL_ERRNO with errno 34, then 0, instead of FILE_NOT_FOUND; CREATE_DIRECTORY
    of an existing directory uses the same translation."""
    ftp.nak = OperationNotAcknowledgedError(MAVFTPErrorCode.FAIL_ERRNO, errno)

    await upload(uav, "gdm")

    assert ftp.files == {SHOW_SLOT_PATH: FLAT}


async def test_a_failing_slot_write_still_fails_the_upload(uav, ftp, monkeypatch):
    ftp.nak = OperationNotAcknowledgedError(MAVFTPErrorCode.FAIL_ERRNO, 5)

    async def put(data, remote_path, *, parents=False, skip_crc_check=False):
        raise OperationNotAcknowledgedError(MAVFTPErrorCode.FILE_NOT_FOUND)

    monkeypatch.setattr(ftp, "put", put)

    with raises(OperationNotAcknowledgedError):
        await upload(uav, "gdm")


@mark.parametrize("version", [0.0, 2.0])
async def test_other_show_slot_version_is_refused_for_good(uav, calls, drone, version):
    drone.parameters["SHOW_SLOT_VER"] = version

    with raises(ShowUploadRefusedError, match="show slot version") as info:
        await upload(uav, "gdm")

    assert info.value.code == "E_SHOW_SLOT_UNSUPPORTED"
    assert calls == [("param", "SHOW_SLOT_VER")]


@KINDS
async def test_no_reply_is_a_link_problem_and_retryable(uav, calls, drone, kind):
    drone.parameters["SHOW_SLOT_VER"] = None
    drone.parameters["SHOW_ORIENTATION"] = None

    with raises(RuntimeError, match="No reply from the drone") as info:
        await upload(uav, kind)

    assert not isinstance(info.value, ShowUploadRefusedError)
    assert calls == [("param", "SHOW_SLOT_VER"), ("param", "SHOW_ORIENTATION")]


async def test_invalid_config_fails_before_touching_the_drone(uav, calls):
    config = {**CONFIG, "coordinateSystem": None}

    with raises(RuntimeError, match="coordinate system"):
        await uav.upload_gdm_show(FLAT, config)

    assert calls == []


# ---------------------------------------------------------------------------
# Partial flash: drones without the show slot are flagged on connection
# ---------------------------------------------------------------------------

INCOMPATIBLE = FlockwaveErrorCode.HW_SW_INCOMPATIBLE


@fixture
def no_boot_delay(monkeypatch):
    async def no_sleep(_):
        pass

    monkeypatch.setattr(driver_module, "sleep", no_sleep)


async def test_drone_without_show_slot_is_flagged_on_connection(
    uav, drone, no_boot_delay, caplog
):
    drone.parameters["SHOW_SLOT_VER"] = None

    with caplog.at_level(logging.WARNING):
        await uav._check_show_slot_support()

    assert INCOMPATIBLE in uav.status.errors
    assert "no SHOW_SLOT_VER" in caplog.text


async def test_drone_with_show_slot_is_not_flagged(uav, no_boot_delay):
    await uav._check_show_slot_support()
    assert INCOMPATIBLE not in uav.status.errors


async def test_flag_clears_after_the_drone_is_flashed(uav, drone, no_boot_delay):
    drone.parameters["SHOW_SLOT_VER"] = None
    await uav._check_show_slot_support()
    assert INCOMPATIBLE in uav.status.errors

    drone.parameters["SHOW_SLOT_VER"] = 1.0
    await uav._check_show_slot_support()
    assert INCOMPATIBLE not in uav.status.errors


async def test_no_reply_leaves_the_flag_alone(uav, drone, no_boot_delay):
    drone.parameters["SHOW_SLOT_VER"] = None
    await uav._check_show_slot_support()

    drone.parameters["SHOW_ORIENTATION"] = None
    await uav._check_show_slot_support()

    assert INCOMPATIBLE in uav.status.errors


async def test_refused_upload_flags_the_drone(uav, drone):
    drone.parameters["SHOW_SLOT_VER"] = 2.0

    with raises(ShowUploadRefusedError):
        await upload(uav, "gdm")

    assert INCOMPATIBLE in uav.status.errors


# ---------------------------------------------------------------------------
# Parameter reads answered with PARAM_ERROR
# ---------------------------------------------------------------------------


@mark.parametrize(
    "reply,expected",
    [
        (("value", SimpleNamespace(param_value=1.0, param_type=9)), 1.0),
        (("error", SimpleNamespace(error=MAVParamError.DOES_NOT_EXIST)), None),
    ],
)
async def test_param_read_tells_a_missing_parameter_from_a_value(reply, expected):
    sent = []

    async def send_packet_with_retries(spec, target, **kwds):
        sent.append(kwds)
        return reply

    uav = MAVLinkUAV(
        "01",
        driver=SimpleNamespace(
            autopilot_factory=ArduPilot,
            log=logging.getLogger("t"),
            send_packet_with_retries=send_packet_with_retries,
        ),
    )

    if expected is None:
        with raises(ParameterNotFoundError, match="SHOW_SLOT_VER"):
            await uav.get_parameter("SHOW_SLOT_VER")
    else:
        assert await uav.get_parameter("SHOW_SLOT_VER") == expected

    assert set(sent[0]["wait_for_one_of"]) == {"value", "error"}
