"""Trio/flockwave wrapper around the sans-IO rtls protocol core.

Besides running the discovery/keepalive loop, the extension exposes the
managed devices to Skybrush clients through experimental (``X-``
prefixed) message types on the server's message hub; see ``README.md``
in this directory for the message API.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from functools import partial
from typing import TYPE_CHECKING, Any, Iterator, Optional

import trio
import trio.socket

from flockwave.server.ext.base import Extension

from .protocol import (
    PARAM_ACK_ACCEPTED,
    PARAM_ACK_IN_PROGRESS,
    ProtocolEvent,
    RtlsProtocol,
    decode_param_value,
    encode_param_value,
    param_type_from_name,
    param_type_to_name,
)

if TYPE_CHECKING:
    from flockwave.server.message_hub import MessageHub
    from flockwave.server.model import Client, FlockwaveMessage

__all__ = ("construct", "description", "schema")

#: device parameter that holds the firmware version string, if the
#: firmware exposes one
FIRMWARE_VERSION_PARAM = "FW_VERSION"

#: default timeout for a single-parameter read/write transaction, in seconds
DEFAULT_PARAM_TIMEOUT = 5.0

#: default timeout for a full parameter list transaction, in seconds
DEFAULT_PARAM_LIST_TIMEOUT = 10.0

#: upper bound for client-supplied transaction timeouts, in seconds
MAX_PARAM_TIMEOUT = 60.0

#: how often OTA progress notifications are broadcast at most, in seconds
OTA_PROGRESS_INTERVAL = 0.5


class RtlsExtension(Extension):
    """Manages rtls-link UWB positioning devices on the show network."""

    def __init__(self):
        super().__init__()
        self._protocol: Optional[RtlsProtocol] = None
        self._sock = None
        self._nursery: Optional[trio.Nursery] = None
        self._watchers: list[trio.MemorySendChannel] = []
        self._ota_jobs: dict[int, dict[str, Any]] = {}
        #: test hook; ``None`` means "use ota.upgrade"
        self._ota_upgrade = None

    async def run(self, app, configuration, logger):
        port = int(configuration.get("port", 3333))
        targets = [(host, port) for host in configuration.get("devices", [])]
        broadcast = [
            (host, port) for host in configuration.get("broadcast", ["255.255.255.255"])
        ]

        from flockwave.protocols.mavlink.introspection import import_dialect

        dialect = import_dialect("ardupilotmega")
        self._protocol = protocol = RtlsProtocol(
            dialect,
            targets=targets,
            broadcast_targets=broadcast,
            heartbeat_interval=float(configuration.get("heartbeat_interval", 2)),
            device_timeout=float(configuration.get("device_timeout", 6)),
        )

        sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
        sock.setsockopt(trio.socket.SOL_SOCKET, trio.socket.SO_BROADCAST, 1)
        await sock.bind(("0.0.0.0", 0))
        self._sock = sock

        logger.info(
            f"rtls: managing devices on UDP :{port} "
            f"({len(targets)} static, broadcast {len(broadcast)})"
        )

        try:
            async with trio.open_nursery() as nursery:
                self._nursery = nursery
                with app.message_hub.use_message_handlers(
                    {
                        "X-RTLS-INF": self._handle_RTLS_INF,
                        "X-RTLS-PARAM-LIST": self._handle_RTLS_PARAM_LIST,
                        "X-RTLS-PARAM-GET": self._handle_RTLS_PARAM_GET,
                        "X-RTLS-PARAM-SET": self._handle_RTLS_PARAM_SET,
                        "X-RTLS-OTA": self._handle_RTLS_OTA,
                    }
                ):
                    await self._run_protocol_loop(protocol, sock, logger)
        finally:
            self._nursery = None
            self._sock = None

    async def _run_protocol_loop(self, protocol, sock, logger) -> None:
        while True:
            now = time.monotonic()
            for payload, address in protocol.poll(now):
                await self._send(payload, address)

            for event in protocol.expire(now):
                logger.warning(f"rtls: device sysid {event.system_id} lost")
                self._dispatch_event(event)

            with trio.move_on_after(0.25):
                try:
                    data, address = await sock.recvfrom(2048)
                except OSError:
                    continue
                await self._process_datagram(data, address, time.monotonic())

    # ---- protocol plumbing ----
    #
    # ``_send`` and ``_process_datagram`` are the only points where the
    # extension touches the wire; tests replace ``_send`` with a fake
    # transport and push device datagrams through ``_process_datagram``.

    async def _send(self, payload: bytes, address: tuple[str, int]) -> None:
        if self._sock is None:
            return
        try:
            await self._sock.sendto(payload, address)
        except OSError:
            pass

    async def _process_datagram(
        self, data: bytes, address: tuple[str, int], now: float
    ) -> None:
        if self._protocol is None:
            return
        for event in self._protocol.feed(data, address, now):
            if event.kind == "discovered":
                if self.log:
                    self.log.info(
                        f"rtls: device sysid {event.system_id} discovered "
                        f"at {event.data['address'][0]}"
                    )
                request = self._protocol.request_param_list(event.system_id)
                if request is not None:
                    await self._send(*request)
            self._dispatch_event(event)

    def _dispatch_event(self, event: ProtocolEvent) -> None:
        for channel in list(self._watchers):
            try:
                channel.send_nowait(event)
            except trio.WouldBlock:
                pass

    @contextmanager
    def _subscribed_events(self) -> Iterator[trio.MemoryReceiveChannel]:
        """Context manager that subscribes the caller to the stream of
        protocol events for the duration of the context. Subscribe
        *before* sending a request so no reply can race past you."""
        tx, rx = trio.open_memory_channel(512)
        self._watchers.append(tx)
        try:
            yield rx
        finally:
            self._watchers.remove(tx)

    # ---- async device operations ----

    async def get_param_list(
        self, system_id: int, *, timeout: float = DEFAULT_PARAM_LIST_TIMEOUT
    ) -> dict[str, dict[str, Any]]:
        """Fetches the full parameter list of a device. Returns a mapping
        from parameter names to ``{"value", "type", "index"}`` dicts.

        Raises KeyError if the device is unknown and trio.TooSlowError
        if the transaction does not complete within the timeout."""
        protocol = self._require_protocol()
        with self._subscribed_events() as events:
            request = protocol.request_param_list(system_id)
            if request is None:
                raise KeyError(system_id)
            await self._send(*request)

            params: dict[str, dict[str, Any]] = {}
            count: Optional[int] = None
            with trio.fail_after(timeout):
                while count is None or len(params) < count:
                    event = await events.receive()
                    if event.kind != "param_value" or event.system_id != system_id:
                        continue
                    params[event.data["name"]] = {
                        "value": decode_param_value(
                            event.data["value"], event.data["type"]
                        ),
                        "type": param_type_to_name(event.data["type"]),
                        "index": event.data["index"],
                    }
                    if event.data["count"]:
                        count = event.data["count"]
            return params

    async def get_param(
        self, system_id: int, name: str, *, timeout: float = DEFAULT_PARAM_TIMEOUT
    ) -> dict[str, Any]:
        """Reads a single parameter from a device; returns a
        ``{"value", "type"}`` dict.

        Raises KeyError if the device is unknown and trio.TooSlowError
        if the device does not answer within the timeout."""
        protocol = self._require_protocol()
        with self._subscribed_events() as events:
            request = protocol.request_param_read(system_id, name)
            if request is None:
                raise KeyError(system_id)
            await self._send(*request)

            with trio.fail_after(timeout):
                while True:
                    event = await events.receive()
                    if (
                        event.kind == "param_value"
                        and event.system_id == system_id
                        and event.data["name"] == name
                    ):
                        return {
                            "value": decode_param_value(
                                event.data["value"], event.data["type"]
                            ),
                            "type": param_type_to_name(event.data["type"]),
                        }

    async def set_param(
        self,
        system_id: int,
        name: str,
        value: Any,
        param_type: Any = None,
        *,
        timeout: float = DEFAULT_PARAM_TIMEOUT,
    ) -> dict[str, Any]:
        """Writes a single parameter and waits for the device-side ack.
        Returns ``{"value", "type", "result", "accepted"}`` where
        ``value`` is the value *acknowledged by the device* (decoded) and
        ``result`` is the raw PARAM_ACK code.

        The parameter type may be omitted if it is already known from an
        earlier parameter listing (devices are listed automatically on
        discovery). Raises KeyError for unknown devices, ValueError if
        the type cannot be determined and trio.TooSlowError on timeout."""
        protocol = self._require_protocol()
        device = protocol.devices.get(system_id)
        if device is None:
            raise KeyError(system_id)

        if param_type is None:
            param_type = device.param_types.get(name)
            if param_type is None:
                raise ValueError(
                    f"type of parameter {name!r} is unknown; specify it "
                    f"explicitly or fetch the parameter list first"
                )
        else:
            param_type = param_type_from_name(param_type)

        encoded = encode_param_value(value, param_type)
        with self._subscribed_events() as events:
            request = protocol.set_param(system_id, name, encoded, param_type)
            if request is None:
                raise KeyError(system_id)
            await self._send(*request)

            with trio.fail_after(timeout):
                while True:
                    event = await events.receive()
                    if (
                        event.kind != "param_ack"
                        or event.system_id != system_id
                        or event.data["name"] != name
                    ):
                        continue
                    result = event.data["result"]
                    if result == PARAM_ACK_IN_PROGRESS:
                        continue  # final ack still to come
                    return {
                        "value": decode_param_value(event.data["value"], param_type),
                        "type": param_type_to_name(param_type),
                        "result": result,
                        "accepted": result == PARAM_ACK_ACCEPTED,
                    }

    async def start_ota(self, system_id: int, image_path: str) -> dict[str, Any]:
        """Starts an OTA upgrade for a device in the background; returns
        a snapshot of the newly created job record.

        Raises KeyError for unknown devices and RuntimeError if an OTA
        job is already running for the device or the extension is not
        running."""
        protocol = self._require_protocol()
        device = protocol.devices.get(system_id)
        if device is None:
            raise KeyError(system_id)

        job = self._ota_jobs.get(system_id)
        if job is not None and job["status"] == "running":
            raise RuntimeError(f"OTA already in progress for device {system_id}")
        if self._nursery is None:
            raise RuntimeError("extension is not running")

        job = {
            "id": system_id,
            "image": image_path,
            "status": "running",
            "progress": 0.0,
            "version": None,
            "error": None,
        }
        self._ota_jobs[system_id] = job
        self._nursery.start_soon(self._run_ota, device.address[0], image_path, job)
        return dict(job)

    async def _run_ota(
        self, address: str, image_path: str, job: dict[str, Any]
    ) -> None:
        upgrade = self._ota_upgrade
        if upgrade is None:
            from .ota import upgrade

        def on_progress(offset: int, total: int) -> None:
            # called from the worker thread; plain dict writes only
            if total:
                job["progress"] = min(1.0, offset / total)

        async def report_progress() -> None:
            last = None
            while True:
                await trio.sleep(OTA_PROGRESS_INTERVAL)
                if job["progress"] != last:
                    last = job["progress"]
                    await self._broadcast_ota_status(job)

        version: Optional[str] = None
        error: Optional[str] = None
        async with trio.open_nursery() as nursery:
            nursery.start_soon(report_progress)
            try:
                version = await trio.to_thread.run_sync(
                    partial(upgrade, address, image_path, on_progress=on_progress)
                )
            except Exception as ex:
                error = str(ex) or type(ex).__name__
            finally:
                nursery.cancel_scope.cancel()

        if error is not None:
            job["status"] = "error"
            job["error"] = error
            if self.log:
                self.log.warning(f"rtls: OTA for sysid {job['id']} failed: {error}")
        else:
            job["status"] = "success"
            job["progress"] = 1.0
            job["version"] = version
            if self.log:
                self.log.info(
                    f"rtls: OTA for sysid {job['id']} finished, "
                    f"uploaded version {version}"
                )
        await self._broadcast_ota_status(job)

    async def _broadcast_ota_status(self, job: dict[str, Any]) -> None:
        if self.app is None:
            return
        hub = self.app.message_hub
        body = {"type": "X-RTLS-OTA", "id": job["id"], "job": dict(job)}
        await hub.broadcast_message(hub.create_notification(body))

    # ---- client message handlers ----

    async def _handle_RTLS_INF(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        devices = self._protocol.devices if self._protocol else {}
        now = time.monotonic()
        status = {
            str(device.system_id): self._device_json(device, now)
            for device in devices.values()
        }
        return hub.create_response_or_notification(
            body={"type": "X-RTLS-INF", "status": status}, in_response_to=message
        )

    async def _handle_RTLS_PARAM_LIST(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            system_id = _get_device_id(message)
            timeout = _get_timeout(message, DEFAULT_PARAM_LIST_TIMEOUT)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        try:
            params = await self.get_param_list(system_id, timeout=timeout)
        except KeyError:
            return hub.reject(message, reason=f"No such device: {system_id}")
        except trio.TooSlowError:
            return hub.reject(
                message,
                reason=f"Timeout while listing parameters of device {system_id}",
            )

        return hub.create_response_or_notification(
            body={
                "type": "X-RTLS-PARAM-LIST",
                "id": system_id,
                "params": params,
                "count": len(params),
            },
            in_response_to=message,
        )

    async def _handle_RTLS_PARAM_GET(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            system_id = _get_device_id(message)
            name = _get_param_name(message)
            timeout = _get_timeout(message, DEFAULT_PARAM_TIMEOUT)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        try:
            result = await self.get_param(system_id, name, timeout=timeout)
        except KeyError:
            return hub.reject(message, reason=f"No such device: {system_id}")
        except trio.TooSlowError:
            return hub.reject(
                message,
                reason=f"Timeout while reading parameter {name!r} "
                f"of device {system_id}",
            )

        return hub.create_response_or_notification(
            body={
                "type": "X-RTLS-PARAM-GET",
                "id": system_id,
                "name": name,
                "value": result["value"],
                "paramType": result["type"],
            },
            in_response_to=message,
        )

    async def _handle_RTLS_PARAM_SET(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            system_id = _get_device_id(message)
            name = _get_param_name(message)
            timeout = _get_timeout(message, DEFAULT_PARAM_TIMEOUT)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        if "value" not in message.body:
            return hub.reject(message, reason="Missing parameter value")
        value = message.body["value"]
        param_type = message.body.get("paramType")

        try:
            result = await self.set_param(
                system_id, name, value, param_type, timeout=timeout
            )
        except KeyError:
            return hub.reject(message, reason=f"No such device: {system_id}")
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        except trio.TooSlowError:
            return hub.reject(
                message,
                reason=f"Timeout while waiting for device {system_id} to "
                f"acknowledge parameter {name!r}",
            )

        return hub.create_response_or_notification(
            body={
                "type": "X-RTLS-PARAM-SET",
                "id": system_id,
                "name": name,
                "value": result["value"],
                "paramType": result["type"],
                "result": result["result"],
                "accepted": result["accepted"],
            },
            in_response_to=message,
        )

    async def _handle_RTLS_OTA(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            system_id = _get_device_id(message)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        image = message.body.get("image")
        if image is None:
            # status query
            job = self._ota_jobs.get(system_id)
            return hub.create_response_or_notification(
                body={
                    "type": "X-RTLS-OTA",
                    "id": system_id,
                    "job": dict(job) if job is not None else None,
                },
                in_response_to=message,
            )

        if not isinstance(image, str):
            return hub.reject(message, reason="Image path must be a string")

        import os

        if not os.path.isfile(image):
            return hub.reject(message, reason=f"No such image file: {image}")

        try:
            job = await self.start_ota(system_id, image)
        except KeyError:
            return hub.reject(message, reason=f"No such device: {system_id}")
        except RuntimeError as ex:
            return hub.reject(message, reason=str(ex))

        return hub.create_response_or_notification(
            body={"type": "X-RTLS-OTA", "id": system_id, "job": job},
            in_response_to=message,
        )

    # ---- helpers / exports ----

    def _require_protocol(self) -> RtlsProtocol:
        if self._protocol is None:
            raise RuntimeError("rtls protocol is not running")
        return self._protocol

    def _device_json(self, device, now: float) -> dict[str, Any]:
        version = device.params.get(FIRMWARE_VERSION_PARAM)
        if version is not None:
            version = version.split(b"\x00")[0].decode("utf-8", errors="replace")
        job = self._ota_jobs.get(device.system_id)
        return {
            "id": device.system_id,
            "address": list(device.address),
            "age": round(now - device.last_seen, 3),
            "firmwareVersion": version,
            "paramCount": device.param_count,
            "otaStatus": job["status"] if job is not None else None,
        }

    def exports(self):
        return {
            "devices": self._get_devices,
            "protocol": lambda: self._protocol,
            "get_param": self.get_param,
            "get_param_list": self.get_param_list,
            "set_param": self.set_param,
            "start_ota": self.start_ota,
        }

    def _get_devices(self):
        if self._protocol is None:
            return {}
        return dict(self._protocol.devices)


def _get_device_id(message: "FlockwaveMessage") -> int:
    value = message.body.get("id")
    if value is None:
        raise ValueError("Missing device ID")
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ValueError(f"Invalid device ID: {value!r}") from None


def _get_param_name(message: "FlockwaveMessage") -> str:
    name = message.body.get("name")
    if not isinstance(name, str) or not name:
        raise ValueError("Missing parameter name")
    return name


def _get_timeout(message: "FlockwaveMessage", default: float) -> float:
    value = message.body.get("timeout", default)
    try:
        timeout = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"Invalid timeout: {value!r}") from None
    if not 0 < timeout <= MAX_PARAM_TIMEOUT:
        raise ValueError(f"Timeout must be between 0 and {MAX_PARAM_TIMEOUT} seconds")
    return timeout


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
