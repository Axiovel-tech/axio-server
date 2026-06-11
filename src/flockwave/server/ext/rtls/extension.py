"""Trio/flockwave wrapper around the sans-IO rtls protocol core."""

from __future__ import annotations

import time

import trio
import trio.socket

from flockwave.server.ext.base import Extension

from .protocol import RtlsProtocol

__all__ = ("construct", "description", "schema")


class RtlsExtension(Extension):
    """Manages rtls-link UWB positioning devices on the show network."""

    _protocol: RtlsProtocol | None = None

    async def run(self, app, configuration, logger):
        port = int(configuration.get("port", 3333))
        targets = [(host, port) for host in configuration.get("devices", [])]
        broadcast = [(host, port)
                     for host in configuration.get("broadcast", ["255.255.255.255"])]

        from flockwave.protocols.mavlink.introspection import import_dialect

        dialect = import_dialect("ardupilotmega")
        self._protocol = protocol = RtlsProtocol(
            dialect, targets=targets, broadcast_targets=broadcast,
            heartbeat_interval=float(configuration.get("heartbeat_interval", 2)),
            device_timeout=float(configuration.get("device_timeout", 6)),
        )

        sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
        sock.setsockopt(trio.socket.SOL_SOCKET, trio.socket.SO_BROADCAST, 1)
        await sock.bind(("0.0.0.0", 0))

        logger.info(f"rtls: managing devices on UDP :{port} "
                    f"({len(targets)} static, broadcast {len(broadcast)})")

        while True:
            now = time.monotonic()
            for payload, address in protocol.poll(now):
                try:
                    await sock.sendto(payload, address)
                except OSError:
                    pass

            for event in protocol.expire(now):
                logger.warning(f"rtls: device sysid {event.system_id} lost")

            with trio.move_on_after(0.25):
                try:
                    data, address = await sock.recvfrom(2048)
                except OSError:
                    continue
                for event in protocol.feed(data, address, time.monotonic()):
                    if event.kind == "discovered":
                        logger.info(
                            f"rtls: device sysid {event.system_id} discovered "
                            f"at {event.data['address'][0]}")
                        request = protocol.request_param_list(event.system_id)
                        if request is not None:
                            await sock.sendto(*request)
                    elif event.kind == "param_ack":
                        logger.info(
                            f"rtls: sysid {event.system_id} "
                            f"{event.data['name']} ack={event.data['result']}")

    def exports(self):
        return {
            "devices": self._get_devices,
            "protocol": lambda: self._protocol,
        }

    def _get_devices(self):
        if self._protocol is None:
            return {}
        return dict(self._protocol.devices)


construct = RtlsExtension
description = "Axiovel rtls-link UWB device management"
schema = {
    "properties": {
        "port": {
            "type": "integer",
            "title": "Management UDP port of the devices",
            "default": 3333,
        },
        "devices": {
            "type": "array",
            "title": "Static device addresses (in addition to broadcast)",
            "items": {"type": "string"},
            "default": [],
        },
        "broadcast": {
            "type": "array",
            "title": "Broadcast addresses for discovery",
            "items": {"type": "string"},
            "default": ["255.255.255.255"],
        },
    }
}
