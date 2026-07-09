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
from flockwave.gps.vectors import GPSCoordinate
from rtlslink import (
    STATS_FIELDS,
    ProtocolEvent,
    RtlsProtocol,
    decode_param_value,
    encode_param_value,
    firmware_version,
    param_type_from_name,
    param_type_to_name,
    parse_address,
)
from rtlslink.dialect import load_dialect
from rtlslink.protocol import PARAM_ACK_ACCEPTED, PARAM_ACK_IN_PROGRESS

try:
    from rtlslink import SLEEP_PARAM
except ImportError:  # pragma: no cover
    # An SDK that predates sleep mode must not take the whole extension
    # down with an ImportError (the extension manager would silently drop
    # discovery/params/OTA too). The constant is just the parameter name.
    SLEEP_PARAM = "SLEEP"

from flockwave.server.ext.base import Extension
from flockwave.server.registries.errors import RegistryFull

from .cell_compat import cell_from_params, ned_to_global_e7, role_from_params
from .show_clock import ShowClockPinManager

if TYPE_CHECKING:
    from flockwave.server.message_hub import MessageHub
    from flockwave.server.model import Client, FlockwaveMessage

__all__ = ("construct", "description", "schema")

#: default timeout for a single-parameter read/write transaction, in seconds
DEFAULT_PARAM_TIMEOUT = 5.0

#: default timeout for a full parameter list transaction, in seconds
DEFAULT_PARAM_LIST_TIMEOUT = 10.0

#: default timeout for a sleep/wake transaction, in seconds (the request
#: itself is one parameter write; see also DEFAULT_SLEEP_SETTLE)
DEFAULT_SLEEP_TIMEOUT = 10.0

#: seconds to wait after an accepted sleep request before reading the
#: SLEEP parameter back: the firmware's arming gate runs asynchronously
#: and may wait up to its confirmation window (2 s by default) for a
#: disarmed heartbeat that postdates the request, refusing by flipping
#: the parameter to 0 -- so the settle must exceed that window
DEFAULT_SLEEP_SETTLE = 3.0

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
        #: beacon-registry API (``app.import_api("beacon")``); ``None`` disables
        #: anchor-beacon registration (config ``register_beacons: false`` or no
        #: beacon extension loaded)
        self._beacon_api = None
        #: live anchor beacons we own, keyed by stable beacon id
        #: (``rtls::<cell>::anchor_<i>``) -> (beacon, ExitStack-less context)
        self._anchor_beacons: dict[str, tuple[Any, Any]] = {}
        #: which device (a tag) currently supplies the cell geometry for a
        #: given cell id; lets us re-home a cell onto another live tag when the
        #: source tag is lost
        self._anchor_cell_sources: dict[str, int] = {}
        #: cluster->GPS pin lifecycle for the UWB-timebase show clock
        #: (``None`` when disabled via config ``show_clock_pin: false``)
        self._show_clock: Optional[ShowClockPinManager] = None
        #: latest inter-anchor TWR telemetry per device:
        #: system_id -> {peer_mac: (measured_distance_m, harvest_monotonic_s)}.
        #: Surfaced in X-RTLS-INF (with a derived age); pruned on device loss.
        self._twr: dict[int, dict[int, tuple[float, float]]] = {}

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
        targets = _configured_addresses(configuration.get("devices", []), port)
        broadcast = _configured_addresses(
            configuration.get("broadcast", ["255.255.255.255"]), port
        )
        register_beacons = bool(configuration.get("register_beacons", True))
        if bool(configuration.get("show_clock_pin", True)):
            self._show_clock = ShowClockPinManager(self)
        self._beacon_api = app.import_api("beacon") if register_beacons else None

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
                        "X-RTLS-SLEEP": self._handle_RTLS_SLEEP,
                        "X-RTLS-OTA": self._handle_RTLS_OTA,
                        "X-RTLS-STATS": self._handle_RTLS_STATS,
                    }
                ):
                    await self._run_protocol_loop(protocol, sock, logger)
        finally:
            self._nursery = None
            self._sock = None
            self._beacon_api = None
            self._clear_anchor_beacons()

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
            elif event.kind == "twr":
                # inter-anchor TWR health telemetry. The SDK surfaces it from
                # the same feed() pass as a dedicated event (peer MAC decoded
                # from the wire name); cache it per peer with a harvest stamp so
                # X-RTLS-INF can report its age.
                self._on_twr(event.system_id, event.data, now)
            elif event.kind == "param_value":
                # a freshly learned parameter may complete a tag's cell or
                # change an anchor's MAC/role; resync the anchor beacons
                self._sync_anchor_beacons_for_system(event.system_id)
                self._refresh_anchor_cells()
            self._dispatch_event(event)

    def _on_twr(self, system_id: int, data: dict[str, Any], now: float) -> None:
        """Cache one inter-anchor TWR sample (decoded by the SDK from the
        ``twr<peer-mac-hex>`` NAMED_VALUE_FLOAT) keyed by peer MAC, with the
        harvest time so X-RTLS-INF can report the measurement's age."""
        peer_mac = data.get("peerMac")
        distance_m = data.get("distanceM")
        if peer_mac is None or distance_m is None:
            return
        self._twr.setdefault(system_id, {})[int(peer_mac)] = (
            float(distance_m),
            now,
        )

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
                    if result == PARAM_ACK_ACCEPTED:
                        # mirror the acknowledged value into the device cache so
                        # the anchor display reflects the server's own push
                        # without waiting for the device to re-advertise it
                        device.params[name] = event.data["value"]
                        device.param_types[name] = param_type
                        self._sync_anchor_beacons(
                            device, _decoded_device_params(device)
                        )
                        self._refresh_anchor_cells()
                    return {
                        "value": decode_param_value(event.data["value"], param_type),
                        "type": param_type_to_name(param_type),
                        "result": result,
                        "accepted": result == PARAM_ACK_ACCEPTED,
                    }

    async def set_sleep(
        self,
        system_id: int,
        sleeping: bool,
        *,
        settle: float = DEFAULT_SLEEP_SETTLE,
        timeout: float = DEFAULT_PARAM_TIMEOUT,
    ) -> dict[str, Any]:
        """Puts a device to sleep (``sleeping=True``) or wakes it, and
        verifies the outcome. Sleep is commanded through the ``SLEEP``
        parameter; the firmware refuses the request while its flight
        controller is armed (or not confirmed disarmed, in strict gate
        mode) by flipping the parameter back to 0 shortly after the ack,
        so a sleep request is read back after ``settle`` seconds.

        Returns ``{"requested", "accepted", "sleeping", "detail"}``.
        Raises KeyError for unknown devices and trio.TooSlowError on
        timeout. Note that on hardware a woken device reboots shortly
        after the ack (to re-initialize the power-cycled UWB module) and
        drops off the network for a few seconds."""
        request = 1 if sleeping else 0
        # `timeout` bounds the whole transaction (write + settle + readback),
        # not each parameter operation separately, so a caller's deadline
        # holds.
        with trio.fail_after(timeout + settle):
            ack = await self.set_param(
                system_id, SLEEP_PARAM, request, "uint8", timeout=timeout
            )
            if not ack["accepted"]:
                return {
                    "requested": sleeping,
                    "accepted": False,
                    "sleeping": None,
                    "detail": f"parameter write rejected (code {ack['result']})",
                }
            if not sleeping:
                return {
                    "requested": False,
                    "accepted": True,
                    "sleeping": False,
                    "detail": "awake",
                }

            await trio.sleep(settle)
            try:
                state = await self.get_param(
                    system_id, SLEEP_PARAM, timeout=timeout
                )
            except (KeyError, trio.TooSlowError):
                # The write was acknowledged, so the device is most likely
                # asleep -- report the verification gap instead of a plain
                # timeout that reads like the command failed.
                return {
                    "requested": True,
                    "accepted": False,
                    "sleeping": None,
                    "detail": "sleep acknowledged but verification failed "
                    "(device state unknown)",
                }
        asleep = bool(state["value"])
        return {
            "requested": True,
            "accepted": asleep,
            "sleeping": asleep,
            "detail": (
                "asleep"
                if asleep
                else "refused by device (arming gate: vehicle armed or "
                "flight controller not confirmed disarmed)"
            ),
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
        # the show-clock pin rides the same stats feed: fresh cluster time
        # (clkok) mints/verifies the pin and unpinned devices get a push
        if self._show_clock is not None and self._nursery is not None:
            self._show_clock.on_stats(system_id, data, self._nursery)
        # STATS_FIELDS is the SDK's required legacy set; newer stats
        # (``slp`` sleep state, the cluster clock) are optional and must
        # not gate the broadcast, or firmware without them would never
        # get a health snapshot out.
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
        X-RTLS-STATS query stops reporting it (mirroring X-RTLS-INF), drop its
        TWR telemetry, re-home any cell it sourced, then forward the event to
        subscribers."""
        self._prune_stats(event.system_id)
        self._twr.pop(event.system_id, None)
        if self._show_clock is not None:
            self._show_clock.forget_device(event.system_id)
        self._drop_anchor_cells_for_source(event.system_id)
        self._refresh_anchor_cells()
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
            body={
                "type": "X-RTLS-INF",
                "status": status,
                "anchors": self._site_anchors(),
            },
            in_response_to=message,
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

    async def _handle_RTLS_SLEEP(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        body = message.body
        ids = body.get("ids")
        if ids is None:
            try:
                ids = [_get_device_id(message)]
            except ValueError as ex:
                return hub.reject(message, reason=str(ex))
        if (
            not isinstance(ids, list)
            or not ids
            or len(ids) > 256
            or not all(
                # bool is an int subclass: JSON `true` would otherwise pass
                # and target sysid 1
                isinstance(i, int) and not isinstance(i, bool) and 1 <= i <= 255
                for i in ids
            )
        ):
            return hub.reject(
                message, reason="Missing or invalid device ids ('ids' or 'id')"
            )
        sleeping = body.get("sleeping")
        if not isinstance(sleeping, bool):
            return hub.reject(message, reason="Missing 'sleeping' boolean")
        try:
            timeout = _get_timeout(message, DEFAULT_SLEEP_TIMEOUT)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        results: dict[str, dict[str, Any]] = {}

        async def run_one(system_id: int) -> None:
            key = str(system_id)
            try:
                results[key] = await self.set_sleep(
                    system_id, sleeping, timeout=timeout
                )
            except KeyError:
                results[key] = _sleep_error(sleeping, f"No such device: {system_id}")
            except ValueError as ex:
                results[key] = _sleep_error(sleeping, str(ex))
            except trio.TooSlowError:
                results[key] = _sleep_error(
                    sleeping,
                    f"Timeout while waiting for device {system_id}",
                )
            except Exception as ex:  # noqa: BLE001
                # never let one device's failure cancel the siblings
                results[key] = _sleep_error(sleeping, str(ex))

        # one request per device, concurrently: the sleep verification
        # includes a settle delay, so a serial loop over a fleet would
        # multiply it
        async with trio.open_nursery() as nursery:
            for system_id in dict.fromkeys(ids):
                nursery.start_soon(run_one, system_id)

        return hub.create_response_or_notification(
            body={
                "type": "X-RTLS-SLEEP",
                "sleeping": sleeping,
                "result": results,
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
        params = _decoded_device_params(device)
        role = role_from_params(params)

        body = {
            "id": device.system_id,
            "address": list(device.address),
            "age": round(now - device.last_seen, 3),
            "firmwareVersion": firmware_version(device),
            "paramCount": device.param_count,
            "otaStatus": job["status"] if job is not None else None,
            # getattr: tolerate an SDK that predates sleep mode
            "sleeping": bool(getattr(device, "sleeping", False)),
        }
        if role is not None:
            body["role"] = role
        name = _device_name(device.system_id, params, role)
        if name is not None:
            body["name"] = name
        twr = self._twr.get(device.system_id)
        if twr:
            # one row per heard peer, freshest first, with the age derived from
            # the harvest stamp so the client can flag stale (peer-quiet) rows
            body["twr"] = [
                {
                    "peerMac": peer_mac,
                    "distanceM": round(distance_m, 3),
                    "ageMs": round(max(0.0, now - harvested) * 1000.0),
                }
                for peer_mac, (distance_m, harvested) in sorted(
                    twr.items(), key=lambda kv: kv[1][1], reverse=True
                )
            ]
        return body

    # ---- site / cell / anchor beacons ----

    def _site_anchors(self) -> list[dict[str, Any]]:
        """The site-level anchor list for X-RTLS-INF: one entry per anchor in
        every known cell, with its stable beacon id, GPS position (origin+NED)
        and ``active`` flag (true iff a matching live anchor device is online).
        Derived from the same cell the anchor beacons render."""
        out: list[dict[str, Any]] = []
        if self._protocol is None:
            return out
        for cell_id, source in sorted(self._anchor_cell_sources.items()):
            device = self._protocol.devices.get(source)
            if device is None:
                continue
            try:
                cell = cell_from_params(
                    _decoded_device_params(device), cell_id=cell_id
                )
            except (KeyError, TypeError, ValueError):
                continue
            for anchor in cell.anchors:
                lat_e7, lon_e7, alt_mm = ned_to_global_e7(
                    cell.origin, anchor.north_m, anchor.east_m, anchor.down_m
                )
                out.append(
                    {
                        "id": _anchor_beacon_id(cell_id, anchor.index),
                        "cell": cell_id,
                        "index": anchor.index,
                        "mac": anchor.mac,
                        "position": {
                            "lat": round(lat_e7 * 1e-7, 7),
                            "lon": round(lon_e7 * 1e-7, 7),
                            "amsl": round(alt_mm * 1e-3, 3),
                        },
                        "active": self._anchor_device_online(anchor.mac),
                    }
                )
        return out

    def _sync_anchor_beacons_for_system(self, system_id: int) -> None:
        if self._protocol is None:
            return
        device = self._protocol.devices.get(system_id)
        if device is not None:
            self._sync_anchor_beacons(device, _decoded_device_params(device))

    def _sync_anchor_beacons(self, device, params: dict[str, Any]) -> None:
        """Reconcile the cell that ``device`` sources, and (when the beacon API
        is available) the anchor beacons rendered from it. Only a tag carries
        the cell geometry; any other role (or an invalid role) drops the cell
        this device used to source.

        Cell-source bookkeeping (``_anchor_cell_sources``) is maintained
        unconditionally so the X-RTLS-INF site anchors list reflects the
        advertised geometry even with ``register_beacons: false``; only the
        per-anchor beacon create/update/drop is gated on the beacon API."""
        role = _cell_source_role(device, params)
        if role is None:
            if "UWB_ROLE" in params:
                self._drop_anchor_cells_for_source(device.system_id)
            return
        if role != "tag":
            self._drop_anchor_cells_for_source(device.system_id)
            return

        try:
            cell = cell_from_params(params, cell_id=_cell_id_from_params(params))
        except (KeyError, TypeError, ValueError):
            return

        self._anchor_cell_sources[cell.cell_id] = device.system_id

        if self._beacon_api is None:
            return

        active_ids = set()
        for anchor in cell.anchors:
            beacon_id = _anchor_beacon_id(cell.cell_id, anchor.index)
            active_ids.add(beacon_id)
            beacon = self._get_or_create_anchor_beacon(beacon_id)
            if beacon is None:
                continue

            lat_e7, lon_e7, alt_mm = ned_to_global_e7(
                cell.origin, anchor.north_m, anchor.east_m, anchor.down_m
            )
            beacon.basic_properties.name = f"RTLS A{anchor.index}"
            beacon.update_status(
                position=GPSCoordinate(
                    lat=lat_e7 * 1e-7,
                    lon=lon_e7 * 1e-7,
                    amsl=alt_mm * 1e-3,
                ),
                active=self._anchor_device_online(anchor.mac),
            )

        for beacon_id in list(self._anchor_beacons):
            if (
                beacon_id.startswith(f"rtls::{cell.cell_id}::")
                and beacon_id not in active_ids
            ):
                self._drop_anchor_beacon(beacon_id)

    def _drop_anchor_cells_for_source(self, system_id: int) -> None:
        for cell_id, source in list(self._anchor_cell_sources.items()):
            if source != system_id:
                continue
            replacement = self._find_anchor_cell_source(cell_id)
            if replacement is None:
                self._drop_anchor_cell(cell_id)
            else:
                device, params = replacement
                self._sync_anchor_beacons(device, params)

    def _refresh_anchor_cells(self) -> None:
        if self._protocol is None:
            return
        for cell_id, source in list(self._anchor_cell_sources.items()):
            device = self._protocol.devices.get(source)
            if device is None:
                replacement = self._find_anchor_cell_source(cell_id)
                if replacement is None:
                    self._drop_anchor_cell(cell_id)
                else:
                    device, params = replacement
                    self._sync_anchor_beacons(device, params)
            else:
                self._sync_anchor_beacons(device, _decoded_device_params(device))

    def _find_anchor_cell_source(
        self, cell_id: str
    ) -> tuple[Any, dict[str, Any]] | None:
        if self._protocol is None:
            return None
        for device in self._protocol.devices.values():
            params = _decoded_device_params(device)
            if _cell_source_role(device, params) != "tag":
                continue
            try:
                cell = cell_from_params(params, cell_id=_cell_id_from_params(params))
            except (KeyError, TypeError, ValueError):
                continue
            if cell.cell_id == cell_id:
                return device, params
        return None

    def _anchor_device_online(self, mac: int | None) -> bool:
        if mac is None or self._protocol is None:
            return False
        for device in self._protocol.devices.values():
            params = _decoded_device_params(device)
            if role_from_params(params) not in (
                "anchor-initiator",
                "anchor-responder",
            ):
                continue
            try:
                if int(params.get("UWB_MAC")) == int(mac):
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def _drop_anchor_cell(self, cell_id: str) -> None:
        self._anchor_cell_sources.pop(cell_id, None)
        prefix = f"rtls::{cell_id}::"
        for beacon_id in list(self._anchor_beacons):
            if beacon_id.startswith(prefix):
                self._drop_anchor_beacon(beacon_id)

    def _get_or_create_anchor_beacon(self, beacon_id: str):
        entry = self._anchor_beacons.get(beacon_id)
        if entry is not None:
            return entry[0]
        if self._beacon_api is None:
            return None

        context = self._beacon_api.use(beacon_id)
        try:
            beacon = context.__enter__()
        except RegistryFull:
            if self.app:
                self.app.handle_registry_full_error(self, "RTLS anchor beacon")
            return None

        self._anchor_beacons[beacon_id] = (beacon, context)
        return beacon

    def _drop_anchor_beacon(self, beacon_id: str) -> None:
        entry = self._anchor_beacons.pop(beacon_id, None)
        if entry is None:
            return
        _, context = entry
        context.__exit__(None, None, None)

    def _clear_anchor_beacons(self) -> None:
        for beacon_id in list(self._anchor_beacons):
            self._drop_anchor_beacon(beacon_id)
        self._anchor_cell_sources.clear()

    def exports(self):
        return {
            "devices": self._get_devices,
            "protocol": lambda: self._protocol,
            "get_param": self.get_param,
            "get_param_list": self.get_param_list,
            "set_param": self.set_param,
            "set_sleep": self.set_sleep,
            "start_ota": self.start_ota,
            "verify_species": self.verify_species,
        }

    def _get_devices(self):
        if self._protocol is None:
            return {}
        return dict(self._protocol.devices)


def _sleep_error(requested: bool, detail: str) -> dict[str, Any]:
    """Uniform per-device error entry for an X-RTLS-SLEEP result."""
    return {
        "requested": requested,
        "accepted": False,
        "sleeping": None,
        "detail": detail,
    }


def _configured_addresses(values, default_port: int) -> list[tuple[str, int]]:
    """Parse configured ``host`` / ``host:port`` strings into address tuples,
    defaulting the port to the extension's management port."""
    return [parse_address(value, default_port) for value in values]


def _decoded_device_params(device) -> dict[str, Any]:
    """Decode a device's cached raw params into plain Python values, keyed by
    name. Parameters whose type is not yet known are skipped."""
    params: dict[str, Any] = {}
    for name, value in device.params.items():
        param_type = device.param_types.get(name)
        if param_type is None:
            continue
        params[name] = decode_param_value(value, param_type)
    return params


def _cell_id_from_params(params: dict[str, Any]) -> str:
    value = params.get("RTLS_CELL_ID")
    return str(value) if value not in (None, "") else "default"


def _cell_source_role(device, params: dict[str, Any]) -> str | None:
    """The role to treat a device as for cell sourcing. A configured
    ``UWB_ROLE`` is authoritative; lacking it, a device that has finished
    listing and carries a full cell geometry is treated as a legacy tag (the
    'cell inferred from tag params' compatibility bridge)."""
    role = role_from_params(params)
    if role is not None:
        return role
    if "UWB_ROLE" in params:
        return None
    if (
        device.param_count is not None
        and len(device.params) >= device.param_count
        and _has_cell_geometry(params)
    ):
        return "tag"
    return None


def _has_cell_geometry(params: dict[str, Any]) -> bool:
    return all(
        name in params
        for name in ("ORIGIN_LAT_E7", "ORIGIN_LON_E7", "ORIGIN_ALT_MM", "UWB_AN_COUNT")
    )


def _anchor_beacon_id(cell_id: str, index: int) -> str:
    return f"rtls::{cell_id}::anchor_{index}"


def _device_name(
    system_id: int, params: dict[str, Any], role: str | None
) -> str | None:
    if role == "tag":
        return f"RTLS tag {system_id}"
    if role in ("anchor-initiator", "anchor-responder"):
        index = _anchor_index_for_device(params)
        if index is not None:
            return f"RTLS anchor A{index}"
        return f"RTLS anchor {system_id}"
    return None


def _anchor_index_for_device(params: dict[str, Any]) -> int | None:
    mac = params.get("UWB_MAC")
    if mac is None:
        return None
    try:
        mac = int(mac)
        count = int(params.get("UWB_AN_COUNT", 0))
    except (TypeError, ValueError):
        return None

    for index in range(max(0, count)):
        try:
            if int(params.get(f"UWB_AN{index}_MAC")) == mac:
                return index
        except (TypeError, ValueError):
            continue
    return None


def _stats_json(system_id: int, data: dict[str, Any]) -> dict[str, Any]:
    """Map the SDK's NAMED_VALUE_FLOAT stat names to the UI body shape.

    ``anc``/``agems``/``ancmask`` are integer-semantic floats over the wire
    (anchor count, milliseconds, bitmask); cast them to int for the UI."""
    body = {
        "id": int(system_id),
        "solveRateHz": float(data.get("rate", 0.0)),
        "solvePct": float(data.get("solvepct", 0.0)),
        "anchorsSeen": int(data.get("anc", 0)),
        "fixAgeMs": int(data.get("agems", 0)),
        "clockPpm": float(data.get("ppm", 0.0)),
        "anchorMask": int(data.get("ancmask", 0)),
    }
    if "slp" in data:
        # sleep state rides the stats stream too; omitted (not False) for
        # firmware that predates sleep mode, so the UI can tell "unknown"
        body["sleeping"] = bool(data["slp"])
    return body


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
dependencies = ("beacon",)
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
        "register_beacons": {
            "type": "boolean",
            "title": "Register configured RTLS anchors as map beacons",
            "default": True,
        },
    }
}
