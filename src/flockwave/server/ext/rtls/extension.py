"""Trio/flockwave glue between the server's message hub and the
``rtlslink`` SDK (the standalone Python package shipped with the
rtls-link firmware repo).

The sans-IO protocol core (:class:`rtlslink.RtlsProtocol`) and the
MCUmgr/SMP OTA helper (:func:`rtlslink.ota.upgrade`) live in the SDK;
this extension only runs the discovery/keepalive loop on the server's
Trio socket and exposes the managed devices to Skybrush clients through
experimental (``X-`` prefixed) message types on the message hub; see
``README.md`` in this directory for the message API.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from functools import partial
from typing import TYPE_CHECKING, Any, Iterator, Optional

import trio
import trio.socket
from rtlslink import (
    STATS_FIELDS,
    ProtocolEvent,
    RtlsProtocol,
    decode_param_value,
    encode_param_value,
    firmware_version,
    param_type_from_name,
    param_type_to_name,
)
from rtlslink.dialect import load_dialect
from rtlslink.protocol import PARAM_ACK_ACCEPTED, PARAM_ACK_IN_PROGRESS

from flockwave.server.ext.base import Extension

if TYPE_CHECKING:
    from flockwave.server.message_hub import MessageHub
    from flockwave.server.model import Client, FlockwaveMessage

__all__ = ("construct", "description", "schema")

#: default timeout for a single-parameter read/write transaction, in seconds
DEFAULT_PARAM_TIMEOUT = 5.0

#: default timeout for a full parameter list transaction, in seconds
DEFAULT_PARAM_LIST_TIMEOUT = 10.0

#: upper bound for client-supplied transaction timeouts, in seconds
MAX_PARAM_TIMEOUT = 60.0

#: how often OTA progress notifications are broadcast at most, in seconds
OTA_PROGRESS_INTERVAL = 0.5

#: UWB_ROLE wire values -> artifact species. Since the firmware's
#: tag/anchor application split the image IS the role (UWB_ROLE carries
#: image-pinned bounds: tag 1..1, anchor 2..3), so OTA must ship the
#: role-matched artifact; see rtls-link-zephyr PR #29.
ROLE_SPECIES = {1: "tag", 2: "anchor", 3: "anchor"}

#: how often per-device stats notifications are broadcast at most, in seconds
STATS_INTERVAL = 1.0


class RtlsExtension(Extension):
    """Manages rtls-link UWB positioning devices on the show network."""

    def __init__(self):
        super().__init__()
        self._protocol: Optional[RtlsProtocol] = None
        self._sock = None
        self._nursery: Optional[trio.Nursery] = None
        self._watchers: list[trio.MemorySendChannel] = []
        self._ota_jobs: dict[int, dict[str, Any]] = {}
        #: latest health-telemetry snapshot per device (server body shape)
        self._stats: dict[int, dict[str, Any]] = {}
        #: monotonic timestamp of the last stats broadcast per device, for
        #: the broadcast throttle
        self._last_stats_broadcast: dict[int, float] = {}
        #: snapshot body last broadcast per device; lets the periodic flush
        #: in the protocol loop tell whether a newer (throttled) update is
        #: still waiting to be sent
        self._last_stats_sent: dict[int, dict[str, Any]] = {}
        #: test hook; ``None`` means "use ota.upgrade"
        self._ota_upgrade = None

    async def run(self, app, configuration, logger):
        # Fail fast on the classic standalone-harness mistake: the server
        # framework sets ``self.app`` before calling run(), and every
        # broadcast path (stats, OTA progress) silently no-ops without it.
        # A test rig that constructs the extension directly must do the
        # same, or it would "work" while dropping all notifications.
        if self.app is None:
            raise RuntimeError(
                "rtls extension started without an app: set ext.app to the "
                "application (the server framework does this before run(); "
                "a standalone harness must too, or stats/OTA broadcasts "
                "are silently dropped)"
            )
        port = int(configuration.get("port", 3333))
        targets = [(host, port) for host in configuration.get("devices", [])]
        broadcast = [
            (host, port) for host in configuration.get("broadcast", ["255.255.255.255"])
        ]

        dialect = load_dialect()
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
                        "X-RTLS-STATS": self._handle_RTLS_STATS,
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
                self._handle_lost(event)

            # trailing-edge flush: a complete stats update that arrived inside
            # the throttle window is cached but not yet broadcast; push it once
            # the window has elapsed so clients see the latest snapshot.
            await self._flush_pending_stats(now)

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
            elif event.kind == "stats":
                # health telemetry arrives unsolicited; cache the latest
                # snapshot and broadcast it (throttled) so live GCS clients
                # see device health without polling.
                await self._on_stats(event.system_id, event.data, now)
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

    async def verify_species(
        self, system_id: int, role: str, *, timeout: float = DEFAULT_PARAM_TIMEOUT
    ) -> None:
        """Refuses a wrong-species OTA: reads the device's UWB_ROLE and
        raises ValueError unless it matches the artifact's declared
        species (``"tag"`` or ``"anchor"``). The role is image-pinned on
        the device, so uploading the wrong image silently strips the
        device of its function until it is re-flashed — this is the seam
        that makes that mistake loud.

        Raises KeyError for unknown devices and trio.TooSlowError when
        the device does not answer the role read in time."""
        result = await self.get_param(system_id, "UWB_ROLE", timeout=timeout)
        species = ROLE_SPECIES.get(int(result["value"]))
        if species != role:
            raise ValueError(
                f"Device {system_id} is {species or 'of unknown role'}; "
                f"refusing to upload a {role} image (the role is "
                "image-pinned — OTA the matching artifact)"
            )

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
            from rtlslink.ota import upgrade

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
            self._warn_no_app()
            return
        hub = self.app.message_hub
        body = {"type": "X-RTLS-OTA", "id": job["id"], "job": dict(job)}
        await hub.broadcast_message(hub.create_notification(body))

    async def _on_stats(
        self, system_id: int, data: dict[str, Any], now: float
    ) -> None:
        """Cache the latest health-telemetry snapshot for a device and
        broadcast it to clients, throttled to at most one per
        ``STATS_INTERVAL`` per device.

        The firmware emits one NAMED_VALUE_FLOAT per stat, so the SDK's
        accumulated ``data`` is only complete once a full cycle has arrived.
        We cache every update (so the X-RTLS-STATS query always has the
        latest), but only broadcast once the full stat set is present --
        otherwise the first notification after discovery would carry a
        half-populated snapshot.

        On the leading edge of the throttle window we broadcast immediately;
        a complete update that lands inside the window is only cached here --
        the periodic flush in :meth:`_run_protocol_loop` pushes the latest
        snapshot once the window elapses, so newer values are never dropped."""
        self._stats[system_id] = _stats_json(system_id, data)
        if not all(field in data for field in STATS_FIELDS):
            return
        last = self._last_stats_broadcast.get(system_id)
        if last is not None and now - last < STATS_INTERVAL:
            return
        await self._broadcast_stats(system_id, now)

    async def _flush_pending_stats(self, now: float) -> None:
        """Broadcast any cached stats snapshot that is newer than the last
        one sent and whose throttle window has elapsed (trailing edge)."""
        for system_id, stats in list(self._stats.items()):
            if stats == self._last_stats_sent.get(system_id):
                continue
            last = self._last_stats_broadcast.get(system_id)
            if last is not None and now - last < STATS_INTERVAL:
                continue
            await self._broadcast_stats(system_id, now)

    def _handle_lost(self, event: ProtocolEvent) -> None:
        """React to a device-``lost`` event: drop its cached stats so the
        X-RTLS-STATS query stops reporting it (mirroring X-RTLS-INF), then
        forward the event to subscribers."""
        self._prune_stats(event.system_id)
        self._dispatch_event(event)

    def _prune_stats(self, system_id: int) -> None:
        """Drop all cached stats state for a device (e.g. on ``lost``)."""
        self._stats.pop(system_id, None)
        self._last_stats_broadcast.pop(system_id, None)
        self._last_stats_sent.pop(system_id, None)

    async def _broadcast_stats(self, system_id: int, now: float) -> None:
        stats = self._stats.get(system_id)
        if stats is None:
            return
        self._last_stats_broadcast[system_id] = now
        self._last_stats_sent[system_id] = dict(stats)
        if self.app is None:
            self._warn_no_app()
            return
        hub = self.app.message_hub
        body = {"type": "X-RTLS-STATS", "stats": {str(system_id): dict(stats)}}
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

        # Optional species guardrail: the client declares which role the
        # artifact was built for and the server verifies the device
        # against it before uploading (the tag/anchor split made the
        # role image-pinned).
        role = message.body.get("role")
        if role is not None:
            if role not in ("tag", "anchor"):
                return hub.reject(
                    message, reason=f"Invalid role: {role!r} (expected tag or anchor)"
                )
            try:
                await self.verify_species(system_id, role)
            except KeyError:
                return hub.reject(message, reason=f"No such device: {system_id}")
            except ValueError as ex:
                return hub.reject(message, reason=str(ex))
            except trio.TooSlowError:
                return hub.reject(
                    message,
                    reason=f"Timeout while reading UWB_ROLE of device "
                    f"{system_id} for the species check",
                )

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

    async def _handle_RTLS_STATS(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        # Optional ``id`` narrows the snapshot to one device; without it the
        # latest stats for every known device are returned.
        device_id = message.body.get("id")
        if device_id is not None:
            try:
                system_id = _get_device_id(message)
            except ValueError as ex:
                return hub.reject(message, reason=str(ex))
            stats = self._stats.get(system_id)
            snapshot = {str(system_id): dict(stats)} if stats is not None else {}
        else:
            snapshot = {
                str(sysid): dict(stats) for sysid, stats in self._stats.items()
            }

        return hub.create_response_or_notification(
            body={"type": "X-RTLS-STATS", "stats": snapshot},
            in_response_to=message,
        )

    # ---- helpers / exports ----

    def _warn_no_app(self) -> None:
        """Once-per-instance warning for broadcasts dropped because
        ``self.app`` was never set (direct-handler use in a harness that
        bypassed run()'s guard)."""
        if getattr(self, "_no_app_warned", False):
            return
        self._no_app_warned = True
        if self.log:
            self.log.warning(
                "rtls: dropping broadcasts — extension has no app "
                "(set ext.app when driving the extension outside the server)"
            )

    def _require_protocol(self) -> RtlsProtocol:
        if self._protocol is None:
            raise RuntimeError("rtls protocol is not running")
        return self._protocol

    def _device_json(self, device, now: float) -> dict[str, Any]:
        job = self._ota_jobs.get(device.system_id)
        return {
            "id": device.system_id,
            "address": list(device.address),
            "age": round(now - device.last_seen, 3),
            "firmwareVersion": firmware_version(device),
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
            "verify_species": self.verify_species,
        }

    def _get_devices(self):
        if self._protocol is None:
            return {}
        return dict(self._protocol.devices)


def _stats_json(system_id: int, data: dict[str, Any]) -> dict[str, Any]:
    """Map the SDK's NAMED_VALUE_FLOAT stat names to the UI body shape.

    ``anc``/``agems``/``ancmask`` are integer-semantic floats over the wire
    (anchor count, milliseconds, bitmask); cast them to int for the UI."""
    return {
        "id": int(system_id),
        "solveRateHz": float(data.get("rate", 0.0)),
        "solvePct": float(data.get("solvepct", 0.0)),
        "anchorsSeen": int(data.get("anc", 0)),
        "fixAgeMs": int(data.get("agems", 0)),
        "clockPpm": float(data.get("ppm", 0.0)),
        "anchorMask": int(data.get("ancmask", 0)),
    }


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
