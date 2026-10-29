"""Tests for the `__gdm_upload` support outside the MAVLink driver: the show
upload log and virtual UAVs.
"""

import json
import logging
from base64 import b64encode
from pathlib import Path

from flockwave.gps.vectors import GPSCoordinate
from pytest import raises

from flockwave.server.ext.show.logging import ShowUploadLoggingMiddleware
from flockwave.server.ext.virtual_uavs.driver import VirtualUAVDriver
from flockwave.server.model.messages import FlockwaveMessage
from flockwave.server.show import ShowSpecification
from flockwave.server.show.gdmb import GdmbValidationError

VECTORS = Path(__file__).parent / "fixtures" / "gdm-vectors"
FLAT = (VECTORS / "flat-single" / "d01.gdmb").read_bytes()
FLAT_HASH = json.loads((VECTORS / "flat-single" / "expected.json").read_text())[
    "show_hash"
]

CONFIG: ShowSpecification = {
    "coordinateSystem": {
        "type": "nwu",
        "origin": [18.9151, 47.4863],
        "orientation": "12.5",
    },
    "geofence": {"version": 1, "enabled": True, "polygons": [], "rallyPoints": []},
    "mission": {"id": "fable-v2", "title": "Higher fable v2", "numDrones": 14},
}


def gdm_upload_message() -> FlockwaveMessage:
    return FlockwaveMessage.from_json(  # ty: ignore[unresolved-attribute]
        {
            "$fw.version": "1.0",
            "id": "1",
            "body": {
                "type": "OBJ-CMD",
                "ids": ["01"],
                "command": "__gdm_upload",
                "kwds": {
                    "gdmb": b64encode(FLAT).decode(),
                    "showHash": FLAT_HASH,
                    "config": CONFIG,
                },
            },
        }
    )


def test_logging_middleware_logs_gdm_uploads(caplog):
    middleware = ShowUploadLoggingMiddleware(logging.getLogger("test"))

    with caplog.at_level(logging.INFO):
        middleware(gdm_upload_message(), None)  # type: ignore

    assert "Show upload started" in caplog.text
    metadata = middleware.last_show_metadata
    assert metadata is not None
    assert metadata["mission"]["id"] == "fable-v2"
    assert metadata["coordinateSystem"]["orientation"] == "12.5"


async def test_virtual_uav_validates_and_stores_gdmb():
    driver = VirtualUAVDriver()
    uav = driver.create_uav("01", GPSCoordinate(lat=47.4863, lon=18.9151, amsl=0))

    result = await driver.handle_command___gdm_upload(
        uav, gdmb=b64encode(FLAT).decode(), showHash=FLAT_HASH, config=CONFIG
    )

    assert "Stored .gdmb (542 bytes" in result
    assert uav._gdmb == FLAT
    assert not uav.has_trajectory


async def test_virtual_uav_rejects_invalid_gdmb():
    driver = VirtualUAVDriver()
    uav = driver.create_uav("01", GPSCoordinate(lat=47.4863, lon=18.9151, amsl=0))

    with raises(GdmbValidationError) as info:
        await driver.handle_command___gdm_upload(
            uav, gdmb=b64encode(FLAT).decode(), showHash="00" * 32, config=CONFIG
        )

    assert info.value.code == "E_SHOW_HASH"
    assert uav._gdmb is None
