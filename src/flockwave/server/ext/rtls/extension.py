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

import math
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
from rtlslink.protocol import (
    PARAM_ACK_ACCEPTED,
    PARAM_ACK_IN_PROGRESS,
    RTLS_COMPONENT_ID,
)

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

#: extra seconds a timed-out parameter write keeps holding its
#: per-(device, name) lock to catch a LATE final ack. PARAM_EXT acks
#: carry no transaction id, so an ack that straggles in after the
#: timeout would otherwise be consumed by — and complete — the NEXT
#: write of the same parameter; caught within the grace it is adopted
#: as the (late) outcome of its own transaction instead
LATE_ACK_GRACE = 1.0

#: how often OTA progress notifications are broadcast at most, in seconds
OTA_PROGRESS_INTERVAL = 0.5

#: UWB_ROLE wire values -> artifact species. Since the firmware's
#: tag/anchor application split the image IS the role (UWB_ROLE carries
#: image-pinned bounds: tag 1..1, anchor 2..3), so OTA must ship the
#: role-matched artifact; see rtls-link-zephyr PR #29.
ROLE_SPECIES = {1: "tag", 2: "anchor", 3: "anchor"}

#: how often per-device stats notifications are broadcast at most, in seconds
STATS_INTERVAL = 1.0

#: NAMED_VALUE_FLOAT names of the tag's position-estimate debug stream
#: (rtls-link-zephyr#14): the solved NED triple plus the reported NE sigma,
#: emitted per solve when the tag's ``POS_DBG_HZ`` parameter is nonzero
#: (off by default). All fields of one estimate share a ``time_boot_ms``
#: stamp, which is what groups a cycle on the receive side.
POS_DEBUG_FIELDS = frozenset(("pn", "pe", "pd", "psig"))

#: how often per-device position-debug notifications are broadcast at
#: most, in seconds (the firmware emits at up to 50 Hz; clients get a
#: throttled stream of the latest estimate)
POS_INTERVAL = 0.1

#: how far ``time_boot_ms`` must jump backwards to be treated as a device
#: reboot (clearing the quad-assembly state) rather than a late, reordered
#: datagram of an older cycle (which is dropped). Reordering is bounded by
#: the emit spacing (<= 1 s at the slowest POS_DBG_HZ=1), while a reboot
#: rewinds the stamp by the device's whole uptime; a device that manages to
#: reboot AND resume streaming within this window is re-accepted as soon as
#: its uptime passes the pre-reboot stamp.
POS_REBOOT_BACKSTEP_MS = 5000

#: byte patterns that gate the position-debug scan: a datagram that carries
#: none of the field names cannot contain the stream, so the second parse
#: is skipped (the stream is off by default -- steady-state stats/heartbeat
#: traffic must not pay for it). The names cannot be matched NUL-terminated:
#: ``name`` is the trailing payload field and MAVLink 2 truncates trailing
#: zero bytes, so a short name may end the payload without its padding.
#: False positives (the two bytes appearing elsewhere in a frame) just fall
#: through to the parse + exact name check.
_POS_DEBUG_MARKERS = (b"pn", b"pe", b"pd", b"psig")

#: how often the X-RTLS-INF device list is broadcast at most, in seconds;
#: gained/lost transitions inside the window coalesce into one trailing-edge
#: notification (during a show, drone tags flap in bursts)
INF_INTERVAL = 1.0

#: how often the X-RTLS-INF device list is rebroadcast without any
#: gained/lost transition, in seconds, so client-side ``age`` values keep
#: refreshing between transitions
INF_REFRESH_INTERVAL = 10.0

#: default UDP port of the firmware's phone-home / state-advertisement
#: datagrams (CONFIG_RTLS_LINK_NET_PHONE_HOME_PORT; mirrors
#: rtlslink.advertisement.ADVERTISEMENT_PORT without requiring the module,
#: which pre-advertisement SDK pins do not ship)
DEFAULT_ADVERTISEMENT_PORT = 3343

#: active-mode presence defaults: the classic 2 s heartbeat probe with a
#: 6 s liveness timeout (three missed probes)
DEFAULT_HEARTBEAT_INTERVAL = 2.0
DEFAULT_DEVICE_TIMEOUT = 6.0

#: passive-mode presence defaults: a slow 60 s "hello" (boards still need
#: to learn the server address for their unicast telemetry, and legacy
#: boards without advertisement firmware still need an occasional probe)
#: and a 30 s timeout = 3x the firmware's 10 s advertisement period
DEFAULT_HELLO_INTERVAL = 60.0
DEFAULT_PASSIVE_DEVICE_TIMEOUT = 30.0

#: heartbeat system_status values the sleep latch derives from (STANDBY =
#: sleeping, ACTIVE = awake); numeric MAVLink enum values mirrored here --
#: like rtlslink.protocol mirrors them -- so the optimistic sleep/wake
#: update needs neither the dialect module nor a post-sleep SDK pin
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4

#: seconds after discovery before a device's param snapshot is first
#: checked for identity holes: the discovery-triggered full dump (~80
#: params) plus retransmission slack must have had time to land, or the
#: refill would race its own trigger and duplicate the dump
REFILL_INITIAL_DELAY = 15.0

#: minimum spacing between identity-refill retry rounds, in seconds; in
#: passive mode the (longer) hello interval paces the rounds instead
REFILL_MIN_INTERVAL = 10.0

#: targeted re-request rounds before the refill gives up on a device
#: that never answers, until it is rediscovered
REFILL_MAX_ATTEMPTS = 5

#: spacing between successive targeted read requests of one refill
#: round, in seconds. The firmware paces its own param dump (a few
#: values per tick) because the management TX queue cannot absorb a
#: burst of replies; an unpaced read burst would recreate the very
#: response loss the refill exists to repair.
REFILL_READ_SPACING = 0.01

#: upper bound on the anchor-table size considered by the refill (the
#: firmware caps UWB_AN_COUNT at 8; a corrupt count must not fan out
#: into an unbounded burst of read requests)
REFILL_MAX_ANCHOR_TABLE = 16

#: identity params a tag must carry for its cell geometry to render
#: (see :func:`_has_cell_geometry`) — plus the geometry-consistency
#: params: X-RTLS-GEO refuses to trust a snapshot whose optional
#: geometry params may be dump-loss holes, so the refill must repair
#: those holes too. A name genuinely absent from an older registry
#: costs the capped retry rounds and is then left alone.
TAG_IDENTITY_PARAMS = (
    "ORIGIN_LAT_E7",
    "ORIGIN_LON_E7",
    "ORIGIN_ALT_MM",
    "UWB_AN_COUNT",
    "POS_YAW_DEG",
    "CELL_ID",
)

#: how long an accepted sleep/wake transaction's outcome overrides
#: contradicting heartbeats, in seconds. The firmware acks the SLEEP write
#: before its power task cuts over, so in-flight pre-transition heartbeats
#: -- and, for a wake, the whole reboot-off-the-network window plus
#: rediscovery -- would otherwise revert the optimistic state right after
#: the push and recreate the stale-state bug (control#16). A heartbeat
#: CONFIRMING the expected state clears the pin early; past the deadline
#: the device's own reports win again.
SLEEP_PIN_TIMEOUT = 30.0


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
        #: feed-clock harvest time of each stats snapshot: consumers that
        #: judge "is this tag healthy NOW" (fleet verify) must not trust
        #: an indefinitely cached snapshot from a stream that went silent
        self._stats_at: dict[int, float] = {}
        #: monotonic timestamp of the last stats broadcast per device, for
        #: the broadcast throttle
        self._last_stats_broadcast: dict[int, float] = {}
        #: snapshot body last broadcast per device; lets the periodic flush
        #: in the protocol loop tell whether a newer (throttled) update is
        #: still waiting to be sent
        self._last_stats_sent: dict[int, dict[str, Any]] = {}
        #: True while a device gained/lost transition still awaits an
        #: X-RTLS-INF broadcast (set on the transition, cleared by the
        #: broadcast); the periodic flush in the protocol loop delivers it
        self._inf_pending = False
        #: monotonic timestamp of the last X-RTLS-INF broadcast, for the
        #: broadcast throttle and the age-refresh rebroadcast
        self._last_inf_broadcast: Optional[float] = None
        #: test hook; ``None`` means "use ota.upgrade"
        self._ota_upgrade = None
        #: test hook for the geometry-sync device reset; ``None`` means
        #: "use the SMP os-reset" (see geometry._default_reset)
        self._geo_reset = None
        #: True while an X-RTLS-GEO sync runs; a second concurrent sync
        #: is refused, and the cell-source bookkeeping refuses to move
        #: the source off the pinned reference while targets are being
        #: rewritten (see :meth:`_sync_anchor_beacons`)
        self._geo_sync_running = False
        #: True while an X-RTLS-VERIFY run is in flight; a second
        #: concurrent run is refused (fleet-wide MAVLink param reads
        #: must not be hammered)
        self._verify_running = False
        #: complete rolling TWR summaries keyed by reporting system ID.
        #: Scalar ``twr`` events remain in ``_twr`` for X-RTLS-INF only;
        #: calibration consumes these capability-gated coherent generations.
        self._twr_summaries: dict[int, Any] = {}
        #: edge-triggered wakeup for fit requests waiting for a generation
        self._twr_summary_changed = trio.Event()
        #: strict fit + exact summary pinned for an optional refined fit
        self._geo_fit_session = None
        #: Process-local identifier for a distributed calibration capture.
        #: A restart clears the pinned session, so persistence is unnecessary.
        self._next_geo_capture_id = 1
        #: lazily loaded canonical-geometry store (see geometry.py);
        #: ``None`` = not loaded yet
        self._geo_canonical: Optional[dict[str, Any]] = None
        #: test seam: overrides the canonical store's file path
        self._geo_store_path = None
        #: per-(system id, name) write serialization: PARAM_EXT acks
        #: carry no transaction id — they are matched by device + name
        #: only — so two overlapping writes of the same parameter could
        #: each consume the FIRST ack and both report the first write's
        #: outcome as their own
        self._param_write_locks: dict[tuple[int, str], trio.Lock] = {}
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
        #: dedicated MAVLink parser for the position-estimate debug stream
        #: (lazily created; see :meth:`_scan_pos_debug`)
        self._pos_parser = None
        #: accumulated position-debug wire fields per device:
        #: system_id -> {name: (value, time_boot_ms)}; a ``None`` value
        #: marks a non-finite wire value (see _pos_json)
        self._pos_wire: dict[int, dict[str, tuple[Optional[float], int]]] = {}
        #: latest complete position-debug snapshot per device (body shape
        #: without the derived ``ageMs``), plus its harvest monotonic stamp
        self._pos: dict[int, dict[str, Any]] = {}
        self._pos_stamp: dict[int, float] = {}
        #: broadcast throttle bookkeeping for X-RTLS-POS, mirroring the
        #: stats throttle
        self._last_pos_broadcast: dict[int, float] = {}
        self._last_pos_sent: dict[int, dict[str, Any]] = {}
        #: ``rtlslink.advertisement.parse_advertisement`` once resolved (a
        #: seam: tests may pre-set a fake); ``None`` while unresolved or when
        #: the pinned SDK predates the advertisement module
        self._parse_advertisement = None
        #: passive-presence mode (config ``passive``): the active heartbeat
        #: slows to the hello cadence and ANY attributable inbound datagram
        #: refreshes device liveness, not just heartbeats
        self._passive = False
        #: management UDP port of the fleet (config ``port``); an
        #: advertisement's source IP is combined with it to form the
        #: device's real address (the datagram leaves a throwaway socket)
        self._management_port = 3333
        #: socket of the state-advertisement listener; ``None`` when the
        #: listener is disabled (config, or a pre-advertisement SDK pin)
        self._adv_sock = None
        #: latest advertisement-borne enrichment per device (keys among
        #: ``firmwareVersion``/``role``/``uptimeMs``); fresher than the
        #: param cache, pruned on device loss
        self._adv: dict[int, dict[str, Any]] = {}
        #: last-known sleep latch per device: system_id -> (sleeping, stamp)
        #: where the stamp is the monotonic time of the heartbeat (or
        #: authoritative sleep/wake transaction) that reported it; drives
        #: the X-RTLS-INF push on an ACTIVE<->STANDBY flip and the
        #: staleness cutoff in :meth:`_device_json`
        self._sleeping: dict[int, tuple[bool, float]] = {}
        #: device-liveness timeout from the presence config, mirrored here
        #: as the sleep-latch staleness cutoff
        self._device_timeout = DEFAULT_DEVICE_TIMEOUT
        #: authoritative sleep-state pins from accepted sleep/wake
        #: transactions: system_id -> (expected_sleeping, deadline); while
        #: pinned, contradicting heartbeats are overridden (see
        #: :meth:`_pinned_sleeping`). Deadlines are on the same monotonic
        #: clock the protocol loop feeds datagrams with. Deliberately NOT
        #: pruned on device loss: a woken device reboots off the network
        #: (lost + rediscovered) and the pin must span that window.
        self._sleep_pins: dict[int, tuple[bool, float]] = {}
        #: identity-param refill bookkeeping per device: system_id ->
        #: {"attempts", "next", "started"}. Seeded on discovery; the
        #: periodic poll re-requests identity params the discovery dump
        #: lost (rtls startup: ~14 boards dump ~80 params at once over
        #: UDP), and the entry is dropped once the snapshot is complete
        #: or the attempt cap is reached — until the next rediscovery
        self._refill: dict[int, dict[str, Any]] = {}
        #: spacing between refill retry rounds; recomputed in run() from
        #: the presence config (the hello interval paces passive mode)
        self._refill_interval = max(
            DEFAULT_HEARTBEAT_INTERVAL, REFILL_MIN_INTERVAL
        )
        #: lazy proxy of the mavlink extension's API (``None`` when the
        #: extension module is unavailable); source of the UAV source
        #: addresses the tag<->drone association joins against
        self._mavlink_api = None
        #: current device->UAV association: system_id -> flockwave UAV id.
        #: Derived state, recomputed continuously by :meth:`_poll_uav_map`
        #: from the live device and UAV source addresses -- never persisted
        self._uav_map: dict[int, str] = {}

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

        # Lazy import: the proxy resolves once the mavlink extension loads
        # (its ``loaded`` flag gates every use), so no load-order dependency
        # is introduced and the rtls extension keeps working without it --
        # devices simply stay unassociated.
        try:
            self._mavlink_api = app.import_api("mavlink")
        except (KeyError, RuntimeError):
            self._mavlink_api = None

        self._passive = bool(configuration.get("passive", False))
        self._management_port = port
        heartbeat_interval, device_timeout = _presence_config(configuration)
        self._device_timeout = device_timeout
        self._refill_interval = max(heartbeat_interval, REFILL_MIN_INTERVAL)

        dialect = load_dialect()
        self._protocol = protocol = RtlsProtocol(
            dialect,
            targets=targets,
            broadcast_targets=broadcast,
            heartbeat_interval=heartbeat_interval,
            device_timeout=device_timeout,
        )

        sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
        sock.setsockopt(trio.socket.SOL_SOCKET, trio.socket.SO_BROADCAST, 1)
        await sock.bind(("0.0.0.0", 0))
        self._sock = sock

        # Passive presence: listen for the firmware's autonomous state
        # advertisements (phone-home datagrams) next to the active
        # management conversation. Both a config of 0/None and a pinned
        # SDK without the advertisement parser disable the listener; the
        # rest of the extension is unaffected either way.
        advertisement_port = int(configuration.get(
            "advertisement_port", DEFAULT_ADVERTISEMENT_PORT
        ) or 0)
        if advertisement_port:
            if self._parse_advertisement is None:
                self._parse_advertisement = _load_advertisement_parser(logger)
            if self._parse_advertisement is not None:
                adv_sock = trio.socket.socket(
                    trio.socket.AF_INET, trio.socket.SOCK_DGRAM
                )
                try:
                    # deliberately NO SO_REUSEADDR: on Linux it would let a
                    # second reuse-enabled listener bind the same port and
                    # silently steal unicast delivery — a collision must
                    # fail loudly (EADDRINUSE) into the warning below
                    await adv_sock.bind(("0.0.0.0", advertisement_port))
                except OSError as ex:
                    adv_sock.close()
                    logger.warning(
                        f"rtls: cannot listen for state advertisements on "
                        f"UDP :{advertisement_port} ({ex}); passive "
                        f"presence disabled"
                    )
                else:
                    self._adv_sock = adv_sock

        logger.info(
            f"rtls: managing devices on UDP :{port} "
            f"({len(targets)} static, broadcast {len(broadcast)}, "
            f"{'passive' if self._passive else 'active'} presence, "
            f"advertisements "
            + (
                f"on UDP :{advertisement_port}"
                if self._adv_sock is not None
                else "off"
            )
            + ")"
        )

        try:
            async with trio.open_nursery() as nursery:
                self._nursery = nursery
                if self._adv_sock is not None:
                    nursery.start_soon(
                        self._run_advertisement_loop, self._adv_sock
                    )
                with app.message_hub.use_message_handlers(
                    {
                        "X-RTLS-INF": self._handle_RTLS_INF,
                        "X-RTLS-PARAM-LIST": self._handle_RTLS_PARAM_LIST,
                        "X-RTLS-PARAM-GET": self._handle_RTLS_PARAM_GET,
                        "X-RTLS-PARAM-SET": self._handle_RTLS_PARAM_SET,
                        "X-RTLS-SLEEP": self._handle_RTLS_SLEEP,
                        "X-RTLS-OTA": self._handle_RTLS_OTA,
                        "X-RTLS-STATS": self._handle_RTLS_STATS,
                        "X-RTLS-POS": self._handle_RTLS_POS,
                        "X-RTLS-GEO": self._handle_RTLS_GEO,
                        "X-RTLS-VERIFY": self._handle_RTLS_VERIFY,
                    }
                ):
                    await self._run_protocol_loop(protocol, sock, logger)
        finally:
            self._nursery = None
            self._sock = None
            if self._adv_sock is not None:
                self._adv_sock.close()
                self._adv_sock = None
            self._adv.clear()
            self._sleeping.clear()
            self._sleep_pins.clear()
            self._mavlink_api = None
            self._uav_map.clear()
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

            # re-join the tag<->drone association against the UAVs' current
            # source addresses (DHCP churn, UAVs appearing/disappearing);
            # a change pushes the (throttled) device list
            await self._poll_uav_map(now)

            # re-request identity params a lossy discovery dump left out
            # of a device's snapshot (backed off, capped; see
            # _poll_param_refill)
            await self._poll_param_refill(now)

            # trailing-edge flush: a complete stats update that arrived inside
            # the throttle window is cached but not yet broadcast; push it once
            # the window has elapsed so clients see the latest snapshot.
            await self._flush_pending_stats(now)
            self._flush_pending_pos(now)

            # same for the device list: coalesced gained/lost transitions are
            # pushed once their throttle window elapses, and the list is
            # rebroadcast on a slow heartbeat so client-side ages refresh.
            await self._flush_pending_inf(now)

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

    async def _run_advertisement_loop(self, sock) -> None:
        """Consumes the firmware's autonomous state advertisements
        (one datagram per board every ADV_PERIOD_S, plus a burst on DHCP
        bind) so presence tracking works without probing the boards."""
        while True:
            try:
                data, address = await sock.recvfrom(2048)
            except OSError:
                await trio.sleep(1)
                continue
            await self._process_advertisement(data, address, time.monotonic())

    async def _process_advertisement(
        self, data: bytes, address: tuple[str, int], now: float
    ) -> None:
        """One datagram from the advertisement socket: parse it, cache the
        enrichment (firmware version / role / uptime), then feed the
        SANITIZED frames through the same protocol seam as a
        management-channel datagram — the advertisement is HEARTBEAT +
        SYSTEM_TIME + PARAM_EXT_VALUE frames by design, so discovery,
        liveness refresh, the param cache and the X-RTLS-INF gained/lost
        broadcasts behave exactly as for active discovery. The datagram's
        source PORT is a throwaway sender the firmware never reads from;
        the device is recorded at (source IP, management port) instead.

        Only the advertiser's own validated frames reach the protocol
        (see :func:`_sanitized_advertisement_frames`): the raw datagram
        must never touch the protocol's shared parser."""
        if self._parse_advertisement is None or self._protocol is None:
            return
        try:
            advertisement = self._parse_advertisement(
                data, address, management_port=self._management_port
            )
        except Exception:
            # the port is open to the world; a garbage datagram must not
            # take the listener down
            return
        if advertisement is None:
            return
        mgmt_address = advertisement.address
        if mgmt_address is None:
            return
        entry: dict[str, Any] = {}
        if advertisement.firmware_version is not None:
            entry["firmwareVersion"] = advertisement.firmware_version
        kind = getattr(advertisement, "kind", None)
        if kind is not None:
            entry["role"] = kind
        if advertisement.uptime_ms is not None:
            entry["uptimeMs"] = int(advertisement.uptime_ms)
        self._adv[advertisement.system_id] = entry
        frames = _sanitized_advertisement_frames(data, advertisement.system_id)
        if frames:
            await self._process_datagram(frames, tuple(mgmt_address), now)

    async def _process_datagram(
        self, data: bytes, address: tuple[str, int], now: float
    ) -> None:
        if self._protocol is None:
            return
        self._scan_pos_debug(data, now)
        if self._passive:
            # In passive mode the boards are probed only every
            # hello_interval, so ANY inbound datagram from a device counts
            # as proof of life — content-independent, because real
            # firmware datagrams exist that yield no protocol event at all
            # (the pn/pe/pd position stats and SYSTEM_TIME are dropped by
            # the pinned SDK, which itself refreshes liveness on
            # heartbeats only), and without this a board whose heartbeats
            # are lost to a contended AP would flap on the slow hello
            # cadence. Attribution is by the MAVLink header system id,
            # NOT by source address: bench DHCP reassigns IPs across
            # power cycles, and an address-keyed refresh would keep a
            # ghost device alive forever on its successor's traffic. The
            # source IP is only a guard — on a mismatch (the device
            # moved) nothing is refreshed here and the normal heartbeat
            # seam migrates the recorded address instead.
            for system_id in _datagram_system_ids(data):
                device = self._protocol.devices.get(system_id)
                if device is not None and device.address[0] == address[0]:
                    device.last_seen = max(device.last_seen, now)
        for event in self._protocol.feed(data, address, now):
            if event.kind == "discovered":
                if self.log:
                    self.log.info(
                        f"rtls: device sysid {event.system_id} discovered "
                        f"at {event.data['address'][0]}"
                    )
                # seed the sleep latch so the first ACTIVE<->STANDBY flip
                # after discovery is recognized as a transition; the pin
                # filter covers a wake-rebooted device rediscovered off a
                # stale pre-transition heartbeat. This must happen BEFORE
                # the first await below: feed() already installed the raw
                # heartbeat state on the device, so a concurrent X-RTLS-INF
                # query at an await checkpoint would otherwise observe the
                # contradictory state before the pin re-asserts it.
                if "sleeping" in event.data:
                    self._sleeping[event.system_id] = (
                        self._pinned_sleeping(
                            event.system_id, bool(event.data["sleeping"]), now
                        ),
                        now,
                    )
                # (re)arm the identity-param refill: the dump requested
                # below is lossy under the all-boards-at-once startup
                # burst, so the snapshot is re-checked for holes once the
                # dump has had time to land (rediscovery resets the cap)
                self._refill[event.system_id] = {
                    "attempts": 0,
                    "next": now + REFILL_INITIAL_DELAY,
                    "started": False,
                }
                request = self._protocol.request_param_list(event.system_id)
                if request is not None:
                    await self._send(*request)
                # push the new device list so clients that only listen (the
                # GUI queries X-RTLS-INF once) see the device appear
                await self._on_inf_change(now)
            elif event.kind == "heartbeat":
                # heartbeats (including the sanitized state advertisements)
                # re-latch the device's sleep state; a flip must reach
                # clients now, not on the 10 s slow INF refresh
                await self._track_sleeping(
                    event.system_id, event.data.get("sleeping"), now
                )
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
            elif event.kind == "twr_summary":
                # The SDK emits this only after it has assembled a coherent
                # capability/header plus complete range/quality/count triples
                # from one device generation. Calibration later combines one
                # A0 spoke from each responder.
                from .fit import on_twr_summary

                on_twr_summary(self, event.system_id, event.data, now)
            elif event.kind == "param_value":
                # a freshly learned parameter may complete a tag's cell or
                # change an anchor's MAC/role; resync the anchor beacons
                self._sync_anchor_beacons_for_system(event.system_id)
                self._refresh_anchor_cells()
                # a reply to a refill round may have just plugged the last
                # hole: finish promptly instead of on the next poll round
                self._check_refill_completion(event.system_id)
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

    async def _track_sleeping(
        self, system_id: int, sleeping: Optional[bool], now: float
    ) -> None:
        """A heartbeat reported a device's sleep state: refresh the latch
        stamp and, when the state flipped since the last report, push the
        (throttled) X-RTLS-INF broadcast -- an ACTIVE<->STANDBY transition
        alone would otherwise only surface on the 10 s slow refresh, and a
        tag that slept inside that window renders green for its whole
        remainder."""
        if sleeping is None:
            return
        sleeping = self._pinned_sleeping(system_id, bool(sleeping), now)
        previous = self._sleeping.get(system_id)
        self._sleeping[system_id] = (sleeping, now)
        if previous is not None and previous[0] != sleeping:
            await self._on_inf_change(now)

    def _pinned_sleeping(self, system_id: int, sleeping: bool, now: float) -> bool:
        """Filter a heartbeat-reported sleep state through the transaction
        pin: while an accepted sleep/wake transaction's expected state is
        pinned, a CONTRADICTING report is overridden -- the firmware acks
        the SLEEP write before its power task cuts over, so an in-flight
        pre-transition heartbeat would otherwise revert the optimistic
        state right after the push. A CONFIRMING report, or the pin's
        deadline passing, clears the pin and resumes normal tracking (a
        device that never confirms is believed again after the deadline).

        Returns the effective state to track/report."""
        pin = self._sleep_pins.get(system_id)
        if pin is None:
            return sleeping
        expected, deadline = pin
        if now >= deadline or sleeping == expected:
            del self._sleep_pins[system_id]
            return sleeping
        # contradicting report inside the window: hold the expected state,
        # re-asserting the device's own latch (the SDK re-latched the wire
        # value before this event reached us)
        self._set_device_system_status(system_id, expected)
        return expected

    def _mark_sleeping(self, system_id: int, sleeping: bool) -> None:
        """Optimistically overwrite a device's passive sleep latch after an
        authoritative sleep/wake transaction: the last-heartbeat latch
        predates the outcome (a woken device even reboots off the network
        for a few seconds before its first ACTIVE heartbeat), so the
        device's ``system_status`` is set to what the transaction just
        established -- and pinned, so a contradicting in-flight heartbeat
        cannot revert it until the device confirms or the pin expires."""
        now = time.monotonic()
        self._sleeping[system_id] = (sleeping, now)
        self._sleep_pins[system_id] = (sleeping, now + SLEEP_PIN_TIMEOUT)
        self._set_device_system_status(system_id, sleeping)

    def _set_device_system_status(self, system_id: int, sleeping: bool) -> None:
        device = self._protocol.devices.get(system_id) if self._protocol else None
        # hasattr: tolerate an SDK that predates sleep mode
        if device is not None and hasattr(device, "system_status"):
            device.system_status = MAV_STATE_STANDBY if sleeping else MAV_STATE_ACTIVE

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

        async def wait_for_final_ack(events):
            while True:
                event = await events.receive()
                if (
                    event.kind != "param_ack"
                    or event.system_id != system_id
                    or event.data["name"] != name
                ):
                    continue
                if event.data["result"] == PARAM_ACK_IN_PROGRESS:
                    continue  # final ack still to come
                return event

        # Serialize writes of the same parameter of the same device:
        # PARAM_EXT acks carry no transaction id, so two overlapping
        # writes would each match the FIRST ack by (device, name) and
        # both report the first write's outcome as their own. The lock
        # spans send -> final ack, so the second write starts only after
        # the first one's ack has been consumed.
        key = (system_id, name)
        lock = self._param_write_locks.setdefault(key, trio.Lock())
        try:
            async with lock:
                # re-resolve the device: while this write queued behind
                # the lock, the device may have been lost and
                # rediscovered under the same system id — mirroring the
                # ack into the pre-lock object would leave the LIVE
                # cache stale
                device = protocol.devices.get(system_id)
                if device is None:
                    raise KeyError(system_id)
                with self._subscribed_events() as events:
                    request = protocol.set_param(
                        system_id, name, encoded, param_type
                    )
                    if request is None:
                        raise KeyError(system_id)
                    await self._send(*request)

                    try:
                        with trio.fail_after(timeout):
                            event = await wait_for_final_ack(events)
                    except trio.TooSlowError:
                        # keep holding the lock through a short drain:
                        # a LATE final ack caught here is adopted as
                        # this transaction's (late) outcome — released
                        # immediately, it would complete the NEXT write
                        # of the same parameter instead
                        event = None
                        with trio.move_on_after(LATE_ACK_GRACE):
                            event = await wait_for_final_ack(events)
                        if event is None:
                            raise

                    result = event.data["result"]
                    if result == PARAM_ACK_ACCEPTED:
                        # mirror the acknowledged value into the device
                        # cache so the anchor display reflects the
                        # server's own push without waiting for the
                        # device to re-advertise it — into the CURRENT
                        # object: the device may have been expired and
                        # rediscovered (a NEW object) at any await point
                        # of this transaction
                        live = protocol.devices.get(system_id)
                        if live is not None:
                            live.params[name] = event.data["value"]
                            live.param_types[name] = param_type
                            self._sync_anchor_beacons(
                                live, _decoded_device_params(live)
                            )
                            self._refresh_anchor_cells()
                    return {
                        "value": decode_param_value(
                            event.data["value"], param_type
                        ),
                        "type": param_type_to_name(param_type),
                        "result": result,
                        "accepted": result == PARAM_ACK_ACCEPTED,
                    }
        finally:
            # prune the lock once nobody holds or awaits it: parameter
            # names are client-supplied, so keeping every (device, name)
            # entry forever would grow without bound. Safe under trio's
            # cooperative scheduling: a task between setdefault and its
            # acquire checkpoint cannot interleave here, so an unheld,
            # waiter-less lock is truly unreferenced-for-serialization.
            stats = lock.statistics()
            if (
                not stats.locked
                and stats.tasks_waiting == 0
                and self._param_write_locks.get(key) is lock
            ):
                self._param_write_locks.pop(key, None)

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
                # Optimistic wake: the ack is authoritative, but the
                # passive sleep latch still holds the pre-wake STANDBY
                # heartbeat and the device is about to reboot off the
                # network for seconds -- flip the latch to ACTIVE and push
                # the device list, so clients see the state the server
                # just established instead of the stale latch.
                self._mark_sleeping(system_id, False)
                await self._on_inf_change(time.monotonic())
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
        if asleep:
            # mirror-optimism for a verified sleep: the readback confirmed
            # the device is asleep, so re-latch STANDBY and push before its
            # next (possibly missed) STANDBY heartbeat does
            self._mark_sleeping(system_id, True)
            await self._on_inf_change(time.monotonic())
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
        self._stats_at[system_id] = now
        # the show-clock pin rides the same stats feed: fresh cluster time
        # (clkok) mints/verifies the pin and unpinned devices get a push
        if self._show_clock is not None and self._nursery is not None:
            self._show_clock.on_stats(system_id, data, self._nursery)
        # STATS_FIELDS is the SDK's required legacy set; newer stats (``slp``
        # sleep state, ``vbat`` battery voltage, the cluster clock) are optional
        # and must not gate the broadcast, or firmware without them would never
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

    # ---- position-estimate debug stream (X-RTLS-POS) ----

    def _scan_pos_debug(self, data: bytes, now: float) -> None:
        """Harvest the tag's position-estimate debug stream from a raw
        management datagram (rtls-link-zephyr#14).

        When ``POS_DBG_HZ`` is nonzero the tag streams every solved NED
        estimate as NAMED_VALUE_FLOAT ``pn``/``pe``/``pd``/``psig`` on the
        same channel as the health stats -- but the SDK's feed() accepts
        only its known stat names, so these are dropped before the event
        stream. Until the SDK grows a dedicated event for them, scan the
        datagram with a second parser (the same transitional pattern the
        inter-anchor TWR telemetry used before it moved into the SDK).

        All fields of one estimate share a ``time_boot_ms`` stamp; a
        snapshot is complete once the whole quad agrees on it (see
        :func:`_pos_json`). Complete snapshots broadcast as X-RTLS-POS,
        throttled per device like the stats."""
        if not any(marker in data for marker in _POS_DEBUG_MARKERS):
            return
        parser = self._pos_parser
        if parser is None:
            dialect = load_dialect()
            parser = dialect.MAVLink(None)
            parser.robust_parsing = True
            self._pos_parser = parser
        for message in parser.parse_buffer(data) or []:
            if message.get_type() != "NAMED_VALUE_FLOAT":
                continue
            name = _named_value_name(message.name)
            if name not in POS_DEBUG_FIELDS:
                continue
            system_id = message.get_srcSystem()
            if (
                self._protocol is None
                or system_id not in self._protocol.devices
            ):
                # Only known (heartbeating) devices may populate the caches:
                # an unknown sysid has no ``lost`` path, so accepting it here
                # would grow the pos dicts unboundedly (spoofed traffic) or
                # resurrect a device right after ``_prune_pos``.
                continue
            value = float(message.value)
            stamp = int(message.time_boot_ms)
            wire = self._pos_wire.setdefault(system_id, {})
            if wire:
                # The assembly only ever holds fields of ONE cycle (they all
                # share a stamp). A newer stamp starts a new cycle; an older
                # one is either a late replay of a previous cycle (dropped,
                # so it cannot regress a field of the current assembly) or a
                # device reboot rewinding time_boot_ms (assembly cleared, or
                # post-reboot fields would pair with pre-reboot leftovers
                # under a recurring stamp).
                current = next(iter(wire.values()))[1]
                if stamp != current:
                    if (
                        stamp > current
                        or current - stamp > POS_REBOOT_BACKSTEP_MS
                    ):
                        wire.clear()
                    else:
                        continue
            # A non-finite value invalidates its field but still counts for
            # the cycle grouping: dropping the field entirely would freeze
            # its stamp and silently blackhole every later cycle (e.g. a
            # covariance blow-up producing inf sigma forever -- exactly the
            # regime a debug stream exists for).
            wire[name] = (value if math.isfinite(value) else None, stamp)
            snapshot = _pos_json(system_id, wire)
            if snapshot is None:
                continue
            self._pos[system_id] = snapshot
            self._pos_stamp[system_id] = now
            last = self._last_pos_broadcast.get(system_id)
            if last is not None and now - last < POS_INTERVAL:
                continue
            self._broadcast_pos(system_id, now)

    def _flush_pending_pos(self, now: float) -> None:
        """Trailing-edge flush for X-RTLS-POS, mirroring the stats flush:
        pushes a cached snapshot that is newer than the last one sent once
        its throttle window has elapsed (so the final estimate of a burst
        is never lost to the throttle)."""
        for system_id, snapshot in list(self._pos.items()):
            if snapshot == self._last_pos_sent.get(system_id):
                continue
            last = self._last_pos_broadcast.get(system_id)
            if last is not None and now - last < POS_INTERVAL:
                continue
            self._broadcast_pos(system_id, now)

    def _broadcast_pos(self, system_id: int, now: float) -> None:
        """Enqueues an X-RTLS-POS notification for a device.

        Deliberately non-blocking: this is called from the extension's
        single receive/expiry loop, so awaiting the hub's bounded TX queue
        (``broadcast_message``) would let outbound backpressure stall
        datagram processing and device-loss handling for the whole
        extension. ``enqueue_broadcast_message`` drops the notification on
        overflow instead, which is the right trade for this stream: the
        next estimate supersedes it and the trailing-edge flush re-sends
        the latest snapshot once the queue drains. (The pre-existing stats
        broadcast path still awaits the hub and shares the exposure at its
        much lower 1 Hz rate; fixing it is out of scope here.)"""
        snapshot = self._pos.get(system_id)
        if snapshot is None:
            return
        self._last_pos_broadcast[system_id] = now
        self._last_pos_sent[system_id] = dict(snapshot)
        if self.app is None:
            self._warn_no_app()
            return
        hub = self.app.message_hub
        body = {
            "type": "X-RTLS-POS",
            "positions": {str(system_id): self._pos_entry(system_id, now)},
        }
        hub.enqueue_broadcast_message(hub.create_notification(body))

    def _pos_entry(self, system_id: int, now: float) -> dict[str, Any]:
        """The client-facing X-RTLS-POS entry for a device: the cached
        snapshot plus the derived ``ageMs`` (time since the estimate was
        harvested, for client-side staleness fading)."""
        entry = dict(self._pos[system_id])
        harvested = self._pos_stamp.get(system_id, now)
        entry["ageMs"] = round(max(0.0, now - harvested) * 1000.0)
        return entry

    def _prune_pos(self, system_id: int) -> None:
        """Drop all cached position-debug state for a device (on ``lost``)."""
        self._pos_wire.pop(system_id, None)
        self._pos.pop(system_id, None)
        self._pos_stamp.pop(system_id, None)
        self._last_pos_broadcast.pop(system_id, None)
        self._last_pos_sent.pop(system_id, None)

    def _handle_lost(self, event: ProtocolEvent) -> None:
        """React to a device-``lost`` event: drop its cached stats so the
        X-RTLS-STATS query stops reporting it (mirroring X-RTLS-INF), drop its
        TWR telemetry, re-home any cell it sourced, then forward the event to
        subscribers. Also marks the device list dirty; the periodic flush in
        the protocol loop broadcasts the updated X-RTLS-INF (this handler is
        synchronous, so it cannot broadcast itself)."""
        self._prune_stats(event.system_id)
        self._prune_pos(event.system_id)
        self._refill.pop(event.system_id, None)
        self._twr.pop(event.system_id, None)
        self._twr_summaries.pop(event.system_id, None)
        if (
            self._geo_fit_session is not None
            and event.system_id
            in self._geo_fit_session.capture.participant_system_ids
        ):
            self._geo_fit_session = None
        self._adv.pop(event.system_id, None)
        self._sleeping.pop(event.system_id, None)
        # _sleep_pins deliberately survives loss: a woken device reboots off
        # the network (lost + rediscovered) inside its pin window
        if self._show_clock is not None:
            self._show_clock.forget_device(event.system_id)
        self._drop_anchor_cells_for_source(event.system_id)
        self._refresh_anchor_cells()
        self._inf_pending = True
        self._dispatch_event(event)

    def _prune_stats(self, system_id: int) -> None:
        """Drop all cached stats state for a device (e.g. on ``lost``)."""
        self._stats.pop(system_id, None)
        self._stats_at.pop(system_id, None)
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

    async def _on_inf_change(self, now: float) -> None:
        """A device was gained (or lost, via :meth:`_handle_lost` and the
        periodic flush): broadcast the full X-RTLS-INF device list, throttled
        to at most one per ``INF_INTERVAL``.

        On the leading edge of the throttle window the list is broadcast
        immediately; transitions inside the window only mark the list dirty --
        the periodic flush in :meth:`_run_protocol_loop` pushes the coalesced
        result once the window elapses, so no transition is ever dropped."""
        self._inf_pending = True
        last = self._last_inf_broadcast
        if last is not None and now - last < INF_INTERVAL:
            return
        await self._broadcast_inf(now)

    async def _flush_pending_inf(self, now: float) -> None:
        """Broadcast the device list when a gained/lost transition is still
        pending and its throttle window has elapsed (trailing edge); with no
        transition pending, rebroadcast every ``INF_REFRESH_INTERVAL`` while
        devices are known so client-side ``age`` values keep refreshing."""
        last = self._last_inf_broadcast
        if self._inf_pending:
            if last is None or now - last >= INF_INTERVAL:
                await self._broadcast_inf(now)
        elif (
            last is not None
            and now - last >= INF_REFRESH_INTERVAL
            and self._protocol is not None
            and self._protocol.devices
        ):
            await self._broadcast_inf(now)

    async def _broadcast_inf(self, now: float) -> None:
        """Broadcast the current X-RTLS-INF body (same shape as the query
        response: the full status map plus the site anchors list, so clients
        can apply a wholesale replace)."""
        self._inf_pending = False
        self._last_inf_broadcast = now
        if self.app is None:
            self._warn_no_app()
            return
        hub = self.app.message_hub
        body = {
            "type": "X-RTLS-INF",
            "status": self._inf_status(now),
            "anchors": self._site_anchors(),
        }
        await hub.broadcast_message(hub.create_notification(body))

    # ---- tag<->drone association ----

    def _uav_source_addresses(self) -> dict[str, tuple[str, int]]:
        """The connected MAVLink UAVs' last-heard-from source addresses,
        keyed by flockwave UAV id, from the mavlink extension's API (empty
        while the extension is not loaded). Tests override this seam."""
        api = self._mavlink_api
        if api is None or not getattr(api, "loaded", False):
            return {}
        try:
            return api.get_uav_source_addresses()
        except Exception:
            return {}

    def _refresh_uav_map(self) -> bool:
        """Recompute the device->UAV association and return whether it
        changed.

        A drone's flight controller reaches the server through its tag's
        WiFi-UART bridge, so the UAV's UDP source IP equals the tag's
        management IP -- the join is on that IP. Purely derived state:
        a device or UAV that disappears (or moves to another IP on a DHCP
        renewal) drops out of the mapping on the next recompute. An IP
        claimed by more than one UAV yields no mapping for its devices
        (better unmapped than mis-attributed -- mis-attribution is the
        operator incident this exists to prevent)."""
        mapping: dict[int, str] = {}
        devices = self._protocol.devices if self._protocol else {}
        if devices:
            uav_by_ip: dict[str, Optional[str]] = {}
            for uav_id, address in self._uav_source_addresses().items():
                ip = address[0]
                # None marks an ambiguous IP (multiple UAVs behind it)
                uav_by_ip[ip] = uav_id if ip not in uav_by_ip else None
            for system_id, device in devices.items():
                uav_id = uav_by_ip.get(device.address[0])
                if uav_id is not None:
                    mapping[system_id] = uav_id
        if mapping == self._uav_map:
            return False
        self._uav_map = mapping
        return True

    async def _poll_uav_map(self, now: float) -> None:
        """One pass of the association loop: recompute the mapping and, when
        it changed, push the (throttled) X-RTLS-INF device list so clients
        re-render the tag<->drone pairing without polling."""
        if self._refresh_uav_map():
            await self._on_inf_change(now)

    # ---- identity-param snapshot refill ----

    async def _poll_param_refill(self, now: float) -> None:
        """One pass of the identity-param refill: for every device whose
        refill round is due, re-request exactly the role-relevant identity
        params still missing from its snapshot (the startup all-boards
        dump loses datagrams; without this a device keeps a generic name
        and missing anchor beacons until it reboots).

        Rounds are paced by the refill interval (the hello cadence in
        passive mode) and capped at ``REFILL_MAX_ATTEMPTS``; a device
        whose snapshot is complete — including the complete-but-unmatched
        anchor case — sheds its refill entry and is never polled again
        until rediscovery."""
        if self._protocol is None or not self._refill:
            return
        for system_id, state in list(self._refill.items()):
            if now < state["next"]:
                continue
            device = self._protocol.devices.get(system_id)
            if device is None:
                self._refill.pop(system_id, None)
                continue
            missing = self._missing_identity_params(device)
            if not missing:
                self._finish_refill(system_id, state, "complete")
                continue
            if state["attempts"] >= REFILL_MAX_ATTEMPTS:
                self._finish_refill(
                    system_id,
                    state,
                    f"gave up after {state['attempts']} attempts "
                    f"({len(missing)} params still missing)",
                )
                continue
            if not state["started"]:
                state["started"] = True
                if self.log:
                    self.log.info(
                        f"rtls: sysid {system_id} param snapshot is "
                        f"missing {len(missing)} identity params after "
                        f"the dump; re-requesting"
                    )
            state["attempts"] += 1
            state["next"] = now + self._refill_interval
            if "UWB_ROLE" in missing:
                # the device cannot be classified at all (UWB_ROLE lost
                # with the dump, or a legacy tag that genuinely lacks it
                # and is only recognized once its snapshot is
                # count-complete): targeted reads cannot enumerate holes
                # whose names were never listed, so re-request the full,
                # firmware-paced dump instead
                request = self._protocol.request_param_list(system_id)
                if request is not None:
                    await self._send(*request)
            elif self._nursery is not None:
                # the paced reads sleep between datagrams; run them off
                # the receive loop so a large round cannot stall
                # heartbeat consumption into false device expiry
                self._nursery.start_soon(
                    self._send_refill_reads, system_id, missing
                )
            else:
                await self._send_refill_reads(system_id, missing)

    async def _send_refill_reads(
        self, system_id: int, names: list[str]
    ) -> None:
        """Send one targeted PARAM_EXT_REQUEST_READ per name, paced: each
        read is answered immediately by the firmware, whose management TX
        queue cannot absorb a burst of replies (the reason its own dump
        paces itself) — blasting the whole list at once would recreate
        the very loss the refill exists to repair. Stops early when the
        device (or the protocol) goes away mid-round."""
        for offset, name in enumerate(names):
            if offset:
                await trio.sleep(REFILL_READ_SPACING)
            protocol = self._protocol
            if protocol is None:
                return
            request = protocol.request_param_read(system_id, name)
            if request is None:
                return
            await self._send(*request)

    def _check_refill_completion(self, system_id: int) -> None:
        """Finish an in-progress refill as soon as a param reply plugs the
        last hole (called from the ``param_value`` seam), so completion is
        logged and further rounds stop without waiting a whole interval.
        Snapshots still inside the initial post-discovery window (refill
        not started) are left alone — the dump is still streaming in."""
        state = self._refill.get(system_id)
        if state is None or not state["started"] or self._protocol is None:
            return
        device = self._protocol.devices.get(system_id)
        if device is not None and not self._missing_identity_params(device):
            self._finish_refill(system_id, state, "complete")

    def _finish_refill(
        self, system_id: int, state: dict[str, Any], outcome: str
    ) -> None:
        self._refill.pop(system_id, None)
        if not state["started"]:
            return  # snapshot was complete all along: nothing to log
        if self.log:
            self.log.info(
                f"rtls: identity param refill for sysid {system_id} {outcome}"
            )
        # a completed refill usually changes the device's derived name /
        # cell; mark the device list dirty so clients need not wait for
        # the slow INF refresh
        self._inf_pending = True

    def _missing_identity_params(self, device) -> list[str]:
        """The role-relevant identity params missing from a device's
        snapshot (see :func:`_missing_identity_param_names`); the role is
        resolved like :meth:`_device_json` does, preferring the fresher
        state advertisement over the param cache."""
        if (
            device.param_count is not None
            and len(device.params) >= device.param_count
        ):
            # the snapshot is count-complete, so the dump lost nothing:
            # an identity param absent from it is genuinely absent on the
            # device and must not be re-polled
            return []
        params = _decoded_device_params(device)
        role = (
            self._adv.get(device.system_id, {}).get("role")
            or role_from_params(params)
        )
        return _missing_identity_param_names(params, role)

    # ---- client message handlers ----

    async def _handle_RTLS_INF(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        return hub.create_response_or_notification(
            body={
                "type": "X-RTLS-INF",
                "status": self._inf_status(time.monotonic()),
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

    async def _handle_RTLS_POS(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        # Optional ``id`` narrows the snapshot to one device; without it the
        # latest position estimate of every reporting device is returned.
        now = time.monotonic()
        device_id = message.body.get("id")
        if device_id is not None:
            try:
                system_id = _get_device_id(message)
            except ValueError as ex:
                return hub.reject(message, reason=str(ex))
            positions = (
                {str(system_id): self._pos_entry(system_id, now)}
                if system_id in self._pos
                else {}
            )
        else:
            positions = {
                str(system_id): self._pos_entry(system_id, now)
                for system_id in self._pos
            }

        return hub.create_response_or_notification(
            body={"type": "X-RTLS-POS", "positions": positions},
            in_response_to=message,
        )

    async def _handle_RTLS_GEO(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        # Lazy import: geometry.py imports helpers from this module, so
        # importing it at module load would be a cycle.
        from .geometry import (
            DEFAULT_FLOAT_TOLERANCE,
            run_adopt,
            run_check,
            run_sync,
        )

        body = message.body
        op = body.get("op")
        if op == "fit":
            from .fit import SUMMARY_WAIT_TIMEOUT_S, run_fit

            try:
                mode = body.get("mode", "strict")
                if not isinstance(mode, str):
                    raise ValueError(f"Invalid fit mode: {mode!r}")
                capture_id = body.get("captureId")
                if capture_id is not None:
                    try:
                        capture_id = int(capture_id)
                    except (TypeError, ValueError):
                        raise ValueError(
                            f"Invalid captureId: {capture_id!r}"
                        ) from None
                timeout = body.get("timeout", SUMMARY_WAIT_TIMEOUT_S)
                try:
                    timeout = float(timeout)
                except (TypeError, ValueError):
                    raise ValueError(f"Invalid timeout: {timeout!r}") from None
                cell = body.get("cell")
                if cell is not None and not isinstance(cell, str):
                    raise ValueError(f"Invalid cell: {cell!r}")
                result = await run_fit(
                    self,
                    mode=mode,
                    cell=cell,
                    capture_id=capture_id,
                    timeout=timeout,
                )
            except ValueError as ex:
                return hub.reject(message, reason=str(ex))
            return hub.create_response_or_notification(
                body=result, in_response_to=message
            )
        if op not in ("adopt", "check", "sync"):
            return hub.reject(
                message,
                reason=f"Invalid op: {op!r} (expected 'adopt', 'check', "
                "'sync' or 'fit')",
            )
        try:
            reference = _get_optional_device_id(body, "reference")
            ids = _get_optional_device_ids(body)
            cell = body.get("cell")
            if cell is not None and not isinstance(cell, str):
                raise ValueError(f"Invalid cell: {cell!r}")
            tolerance = body.get("tolerance", DEFAULT_FLOAT_TOLERANCE)
            try:
                tolerance = float(tolerance)
            except (TypeError, ValueError):
                raise ValueError(f"Invalid tolerance: {tolerance!r}") from None
            if not 0 <= tolerance <= 1:
                raise ValueError("Tolerance must be between 0 and 1")
            timeout = _get_timeout(message, DEFAULT_PARAM_TIMEOUT)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        try:
            if op == "adopt":
                result = await run_adopt(
                    self, reference=reference, tolerance=tolerance
                )
            elif op == "check":
                result = await run_check(
                    self, cell=cell, ids=ids, tolerance=tolerance
                )
            else:
                geometry = body.get("geometry")
                if geometry is not None and not isinstance(geometry, dict):
                    return hub.reject(
                        message,
                        reason="'geometry' must be an object of "
                        "parameter name -> value",
                    )
                result = await run_sync(
                    self,
                    cell=cell,
                    ids=ids,
                    geometry=geometry,
                    tolerance=tolerance,
                    reboot=bool(body.get("reboot", True)),
                    timeout=timeout,
                )
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))

        return hub.create_response_or_notification(
            body=result, in_response_to=message
        )

    async def _handle_RTLS_VERIFY(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        # Lazy import: verify.py imports helpers from this module.
        from .verify import run_verify

        in_depth = bool(message.body.get("inDepth", False))
        try:
            result = await run_verify(self, in_depth=in_depth)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        return hub.create_response_or_notification(
            body=result, in_response_to=message
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

    def _inf_status(self, now: float) -> dict[str, dict[str, Any]]:
        """The full X-RTLS-INF status map: one entry per live device, keyed
        by system id (as string). Shared by the query response and the
        gained/lost broadcast so both carry the same body shape."""
        devices = self._protocol.devices if self._protocol else {}
        return {
            str(device.system_id): self._device_json(device, now)
            for device in devices.values()
        }

    def _device_json(self, device, now: float) -> dict[str, Any]:
        job = self._ota_jobs.get(device.system_id)
        params = _decoded_device_params(device)
        # the state advertisement re-announces version/role every
        # ADV_PERIOD_S, so it is a fresher source than the param cache
        # (filled on discovery / explicit reads); the param path stays the
        # fallback for boards without advertisement firmware
        adv = self._adv.get(device.system_id, {})
        role = adv.get("role") or role_from_params(params)

        body = {
            "id": device.system_id,
            "address": list(device.address),
            "age": round(now - device.last_seen, 3),
            "firmwareVersion": adv.get("firmwareVersion")
            or firmware_version(device),
            "paramCount": device.param_count,
            "otaStatus": job["status"] if job is not None else None,
        }
        # The sleep flag is a passive latch of the last heartbeat's
        # MAV_STATE; when the device has stayed alive past the device
        # timeout without a heartbeat re-latching it (possible in passive
        # mode, where ANY traffic refreshes liveness) the latched value is
        # a guess -- omit the key so clients render "unknown" instead of a
        # stale definite state. Both stamps come from the feed clock, so
        # the comparison is immune to the query-time clock.
        latch = self._sleeping.get(device.system_id)
        if latch is None or device.last_seen - latch[1] <= self._device_timeout:
            # getattr: tolerate an SDK that predates sleep mode
            body["sleeping"] = bool(getattr(device, "sleeping", False))
        if "uptimeMs" in adv:
            body["uptimeMs"] = adv["uptimeMs"]
        # the drone whose flight controller talks through this device's
        # WiFi-UART bridge (source-IP join, see _refresh_uav_map); absent
        # for unassociated devices (anchors, spares, bridge-less tags)
        uav_id = self._uav_map.get(device.system_id)
        if uav_id is not None:
            body["uav"] = uav_id
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
                        # native cell-frame NED (meters) alongside the derived
                        # GPS position: the position-debug view plots anchors
                        # and the X-RTLS-POS estimates in the same frame
                        # without a lossy GPS round-trip
                        "ned": {
                            "north": round(anchor.north_m, 3),
                            "east": round(anchor.east_m, 3),
                            "down": round(anchor.down_m, 3),
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
        # a CELL_ID change re-homes the device: without this, the old
        # cell id would keep pointing at it and render a phantom cell
        # with the NEW cell's geometry
        self._drop_stale_anchor_cells_for_source(
            device.system_id, keep_cell_id=cell.cell_id
        )

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

    def _drop_stale_anchor_cells_for_source(
        self, system_id: int, *, keep_cell_id: str
    ) -> None:
        """Drops (or re-homes onto another live tag) every cell mapping
        this device sources under a cell id OTHER than ``keep_cell_id``
        — its current one. The recursion through ``_sync_anchor_beacons``
        terminates: a replacement source found for a cell id advertises
        exactly that cell id, so its own resync keeps it."""
        for cell_id, source in list(self._anchor_cell_sources.items()):
            if source != system_id or cell_id == keep_cell_id:
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


def _presence_config(configuration) -> tuple[float, float]:
    """Resolve the presence cadence from the extension configuration:
    returns ``(heartbeat_interval, device_timeout)`` in seconds.

    Active mode (default) keeps the classic 2 s probe / 6 s timeout.
    Passive mode (``passive: true``) slows the probe to ``hello_interval``
    (default 60 s) and defaults the timeout to 30 s — 3x the firmware's
    10 s advertisement period — unless ``device_timeout`` is configured
    explicitly."""
    if configuration.get("passive", False):
        heartbeat_interval = float(
            configuration.get("hello_interval", DEFAULT_HELLO_INTERVAL)
        )
        device_timeout = float(
            configuration.get("device_timeout", DEFAULT_PASSIVE_DEVICE_TIMEOUT)
        )
    else:
        heartbeat_interval = float(
            configuration.get("heartbeat_interval", DEFAULT_HEARTBEAT_INTERVAL)
        )
        device_timeout = float(
            configuration.get("device_timeout", DEFAULT_DEVICE_TIMEOUT)
        )
    return heartbeat_interval, device_timeout


def _datagram_system_ids(data: bytes) -> set[int]:
    """The system ids carried in the MAVLink frame headers of one
    datagram, extracted with a fresh throwaway parser (never the
    protocol's shared stateful one). Undecodable bytes yield nothing."""
    parser = load_dialect().MAVLink(None)
    parser.robust_parsing = True
    system_ids: set[int] = set()
    for message in parser.parse_buffer(bytes(data)) or []:
        if message.get_type() == "BAD_DATA":
            continue
        system_ids.add(message.get_srcSystem())
    return system_ids


def _sanitized_advertisement_frames(data: bytes, system_id: int) -> bytes:
    """Re-serializes exactly the frames of one advertisement datagram that
    belong to the advertising device: its HEARTBEAT(s) and its
    PARAM_EXT_VALUE frames (component id 197, same system id as the
    validated advertisement). Everything else is dropped.

    The raw datagram must never be fed to the protocol core:

    - its shared parser is STATEFUL across datagrams, so a truncated
      trailing frame would leave it mid-frame and eat the first frame of
      the next management datagram (cross-socket parser pollution);
      the fresh per-datagram parser here provides isolation, and only
      complete, re-serialized frames come out;
    - frames piggybacked after a valid heartbeat could otherwise
      fabricate other devices, refresh their liveness or overwrite their
      cached parameters from a single spoofed datagram.
    """
    parser = load_dialect().MAVLink(None)
    parser.robust_parsing = True
    frames: list[bytes] = []
    for message in parser.parse_buffer(bytes(data)) or []:
        if (
            message.get_srcSystem() != system_id
            or message.get_srcComponent() != RTLS_COMPONENT_ID
            or message.get_type() not in ("HEARTBEAT", "PARAM_EXT_VALUE")
        ):
            continue
        frames.append(bytes(message.get_msgbuf()))
    return b"".join(frames)


def _load_advertisement_parser(log):
    """Resolves ``rtlslink.advertisement.parse_advertisement``, or returns
    ``None`` (after one warning) when the pinned rtls-link SDK predates
    the advertisement module — the listener is then disabled and the rest
    of the extension works as before."""
    try:
        from rtlslink.advertisement import parse_advertisement
    except ImportError:
        if log:
            log.warning(
                "rtls: the pinned rtls-link SDK has no advertisement "
                "parser (rtlslink.advertisement); state advertisements "
                "will not be consumed — bump the rtls-link pin to enable "
                "passive presence"
            )
        return None
    return parse_advertisement


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
    # CELL_ID is the firmware's operator-facing cell label; RTLS_CELL_ID is
    # the older speculative name kept as a fallback.
    value = params.get("CELL_ID") or params.get("RTLS_CELL_ID")
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


def _missing_identity_param_names(
    params: dict[str, Any], role: str | None
) -> list[str]:
    """The identity params a device's snapshot must gain before its name
    (anchor A-index) or cell geometry (tag) can resolve — an empty list
    when nothing needs re-requesting.

    Missing means absent from the snapshot, never present-but-mismatched:
    an anchor with a complete MAC table that simply does not contain its
    own UWB_MAC yields an empty list (a real configuration state that must
    not be re-polled forever)."""
    if role in ("anchor-initiator", "anchor-responder"):
        if _anchor_index_for_device(params) is not None:
            return []  # A-index already resolves
        missing = [
            name for name in ("UWB_MAC", "UWB_AN_COUNT") if name not in params
        ]
        missing.extend(_missing_anchor_table_names(params, ("MAC",)))
        return missing
    if role == "tag":
        missing = [name for name in TAG_IDENTITY_PARAMS if name not in params]
        # cell_from_params() also needs every counted anchor's NED triple
        # (and the MAC drives the beacon's live/active matching); a hole
        # there keeps the whole cell — hence the map beacons — unrendered
        missing.extend(
            _missing_anchor_table_names(
                params, ("X", "Y", "Z", "MAC", "BIAS_M")
            )
        )
        return missing
    if role is None and "UWB_ROLE" not in params:
        # the device cannot be classified (no advertisement, UWB_ROLE
        # lost with the dump — or genuinely absent on a legacy tag, the
        # _cell_source_role compatibility bridge, which is recognized
        # only once the snapshot is count-complete). The marker makes the
        # poll re-request the whole dump: holes whose names were never
        # listed cannot be enumerated by targeted reads.
        return ["UWB_ROLE"]
    return []


def _missing_anchor_table_names(
    params: dict[str, Any], suffixes: tuple[str, ...]
) -> list[str]:
    """The absent ``UWB_AN{i}_{suffix}`` names for every anchor index the
    snapshot's ``UWB_AN_COUNT`` declares — empty when the count itself is
    absent (it is requested first) or undecodable."""
    if "UWB_AN_COUNT" not in params:
        return []
    try:
        count = int(params["UWB_AN_COUNT"])
    except (TypeError, ValueError):
        return []
    count = max(0, min(count, REFILL_MAX_ANCHOR_TABLE))
    return [
        name
        for index in range(count)
        for name in (f"UWB_AN{index}_{suffix}" for suffix in suffixes)
        if name not in params
    ]


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
    if "vbat" in data:
        # Battery voltage is optional because only newer boards can measure it
        # while the flight-controller rail is off.
        body["batteryVoltage"] = round(float(data["vbat"]), 3)
    if "clkok" in data:
        # cluster-clock sync freshness: the fleet-verify clock rule and the
        # UI both need it; previously only the show-clock pin manager
        # consumed it internally
        body["clockSyncOk"] = bool(data["clkok"])
    return body


def _named_value_name(value) -> str:
    """NUL-trimmed string from a pymavlink NAMED_VALUE_FLOAT ``name``
    (a fixed char[10], not NUL-terminated when full)."""
    if isinstance(value, bytes):
        return value.split(b"\x00", 1)[0].decode(errors="replace")
    return str(value).split("\x00", 1)[0]


def _pos_json(
    system_id: int, wire: dict[str, tuple[Optional[float], int]]
) -> Optional[dict[str, Any]]:
    """Assemble the client-facing position snapshot from the accumulated
    debug-stream wire fields, or ``None`` while the current cycle is
    incomplete.

    The firmware emits the full ``pn``/``pe``/``pd``/``psig`` quad per
    estimate, all stamped with one ``time_boot_ms``, with ``psig``
    trailing -- so a cycle is complete exactly when all four agree on
    the stamp (completing on the NED triple alone would broadcast every
    snapshot one message before its sigma). A cycle that loses a
    datagram is skipped; the next one regroups on its fresh stamp.

    A ``None`` field value marks a non-finite wire value: it satisfies
    the grouping but poisons the NED triple (the cycle yields no
    snapshot), while a poisoned ``sigma`` merely omits the key. ``sigma``
    is also omitted when negative -- the firmware sends ``-1`` for "no
    covariance available"."""
    try:
        north, stamp = wire["pn"]
        east, stamp_e = wire["pe"]
        down, stamp_d = wire["pd"]
        sigma, stamp_s = wire["psig"]
    except KeyError:
        return None
    if not (stamp == stamp_e == stamp_d == stamp_s):
        return None
    if north is None or east is None or down is None:
        return None
    body: dict[str, Any] = {
        "id": int(system_id),
        "north": round(north, 3),
        "east": round(east, 3),
        "down": round(down, 3),
        "timeBootMs": stamp,
    }
    if sigma is not None and sigma >= 0.0:
        body["sigma"] = round(sigma, 3)
    return body


def _get_device_id(message: "FlockwaveMessage") -> int:
    value = message.body.get("id")
    if value is None:
        raise ValueError("Missing device ID")
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ValueError(f"Invalid device ID: {value!r}") from None


def _get_optional_device_id(body: dict, key: str) -> Optional[int]:
    """An optional device system id under ``key`` of a message body:
    ``None`` when absent, the id as int otherwise (numeric strings are
    accepted, like everywhere else in this API)."""
    value = body.get(key)
    if value is None:
        return None
    if isinstance(value, bool):
        # bool is an int subclass: JSON `true` would otherwise pass as 1
        raise ValueError(f"Invalid device ID in {key!r}: {value!r}")
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ValueError(f"Invalid device ID in {key!r}: {value!r}") from None


def _get_optional_device_ids(body: dict) -> Optional[list[int]]:
    """The optional ``ids`` list of a message body, validated like the
    sleep handler validates its targets; ``None`` when absent."""
    ids = body.get("ids")
    if ids is None:
        return None
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
        raise ValueError("Invalid device ids ('ids' must be a list of system ids)")
    return ids


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
        "advertisement_port": {
            "type": "integer",
            "title": "UDP port for the devices' state advertisements "
            "(0 disables the listener)",
            "default": 3343,
        },
        "passive": {
            "type": "boolean",
            "title": "Passive presence: track devices from their state "
            "advertisements and slow active probing to the hello interval "
            "(requires advertisement-capable firmware on the fleet)",
            "default": False,
        },
        "hello_interval": {
            "type": "number",
            "title": "Interval of the active hello probe in passive mode, "
            "in seconds",
            "default": 60,
        },
    }
}
