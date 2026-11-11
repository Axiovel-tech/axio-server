"""Log collection over the server's own MAVLink link to a UAV."""

from __future__ import annotations

import re
from collections.abc import AsyncIterator
from contextlib import AbstractContextManager
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

from trio.abc import ReceiveChannel

from flockwave.server.model.commands import Progress

from ..types import MAVLinkExtensionAPI, spec
from .collect import CollectRequest
from .runner import collect_with_status
from .transport import MAVLinkMessage

if TYPE_CHECKING:
    from ..driver import MAVLinkUAV

__all__ = ("ServerTransport", "collect_progress", "vehicle_directory")

_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


class ServerTransport:
    def __init__(self, uav: MAVLinkUAV):
        api = cast(MAVLinkExtensionAPI, uav.driver.app.import_api("mavlink"))
        network = api.find_network_by_id(uav.network_id)
        if network is None:
            raise RuntimeError(f"UAV {uav.id} has no MAVLink network")
        self._uav = uav
        self._network = network

    @property
    def system_id(self) -> int:
        return self._uav.system_id

    @property
    def description(self) -> dict[str, object]:
        return {"kind": "server", "uav": self._uav.id, "network": self._network.id}

    async def send(self, message_type: str, **fields: Any) -> None:
        message = getattr(spec, message_type.lower())(**fields)
        await self._uav.driver.send_packet(message, self._uav)

    def subscribe(
        self, message_type: str
    ) -> AbstractContextManager[ReceiveChannel[MAVLinkMessage]]:
        return self._network.subscribe(message_type, self._uav.system_id)


def vehicle_directory(dest: str, uav_id: str) -> Path:
    root = Path(dest)
    if not root.is_absolute() or not root.is_dir():
        raise ValueError(
            "dest must be an existing absolute directory on the server host"
        )
    if not _SAFE_ID.match(uav_id):
        raise ValueError(f"UAV ID {uav_id!r} cannot name a directory")
    return root / uav_id


async def collect_progress(
    uav: MAVLinkUAV,
    *,
    dest: str,
    latest: int = 1,
    files: list[str] | None = None,
    log_dir: str = "/APM/LOGS",
    min_battery_voltage: float | None = None,
    status_interval: float = 5,
) -> AsyncIterator[Progress | dict[str, object]]:
    """Progress carries the status snapshot; the result is the run receipt."""
    request = CollectRequest(
        directory=vehicle_directory(dest, uav.id),
        log_dir=log_dir,
        latest=latest,
        files=tuple(files or ()),
        min_battery_voltage=min_battery_voltage,
    )
    interval = min(max(float(status_interval), 1.0), 10.0)
    async with collect_with_status(
        ServerTransport(uav), request, uav.id, interval
    ) as items:
        async for kind, data in items:
            yield Progress(object=data) if kind == "status" else data
