"""Trio/flockwave glue between the server's message hub and the
``rtlslink`` SDK (the standalone Python package shipped with the
rtls-link firmware repo).

The sans-IO protocol core (:class:`rtlslink.RtlsProtocol`) and the
MCUmgr/SMP OTA helper (:func:`rtlslink.ota.upgrade`) live in the SDK;
this extension only runs the discovery/keepalive loop on the server's
Trio socket and exposes the managed devices to Axio clients through
experimental (``X-`` prefixed) message types on the message hub; see
``README.md`` in this directory for the message API.
"""

from __future__ import annotations

import json
import math
import time
from contextlib import contextmanager, suppress
from functools import partial
from typing import TYPE_CHECKING, Any, Callable, Iterator, Optional

import trio
import trio.socket
from flockwave.gps.vectors import GPSCoordinate
from rtlslink import (
    STATS_FIELDS,
    Origin,
    ProtocolEvent,
    RtlsDevice,
    RtlsProtocol,
    anchor_location,
    decode_param_value,
    encode_param_value,
    firmware_version,
    ned_to_global_e7,
    param_type_from_name,
    param_type_to_name,
    parse_address,
    role_from_params,
)
from rtlslink.dialect import load_dialect
from rtlslink.flash import read_receipt
from rtlslink.protocol import (
    ANCHOR_CHARGE_BLOCKS,
    ANCHOR_CHARGE_PHASES,
    ANCHOR_LOW_BATTERY,
    ANCHOR_POWER_INPUTS,
    ANCHOR_POWER_STATES,
    PARAM_ACK_ACCEPTED,
    PARAM_ACK_IN_PROGRESS,
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

from .anchor_lock import (
    anchor_views,
    handle_lock_request,
    lock_json,
    locked_detail,
    show_gate,
)
from .anchor_sleep import (
    ANCHOR_WRITE_TIMEOUT,
    lacks_param,
    set_anchor_sleep,
    sleep_error,
)
from .cell_settings import cell_view, handle_cell_request
from .cluster_clock import ClusterClock
from .fc_identity import FlightController, flight_controller_claims, uav_ids_for_claims
from .survey import Survey, handle_survey_request
from .survey_view import (
    location_from_params,
    stored_location,
    survey_inputs,
    survey_json,
)
from .tag_table import TABLE_STAT, TAG_SURVEY_STATS, AnchorTable

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

#: seconds without any stats report after which a live device's snapshot is
#: marked stale: three periods of the slowest report (the 10 s status one).
#: Single stats the device stops sending are aged out by the SDK, and the
#: snapshot is rebuilt from the SDK's on every report.
STATS_STALE_AFTER = 30.0

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

#: UDP port of the management socket: the boards stream their telemetry
#: to MGMT_HOST:MGMT_PORT, and this is the firmware's MGMT_PORT default
DEFAULT_LISTEN_PORT = 3343

#: presence defaults: a 2 s heartbeat probe with a 6 s liveness timeout
#: (three missed probes)
DEFAULT_HEARTBEAT_INTERVAL = 2.0
DEFAULT_DEVICE_TIMEOUT = 6.0

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

#: minimum spacing between identity-refill retry rounds, in seconds
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

#: what places a tag's cell frame on the map; the lowest online tag of a cell
#: gives its anchors their map positions
ORIGIN_PARAMS = ("ORIGIN_LAT_E7", "ORIGIN_LON_E7", "ORIGIN_ALT_MM")

#: identity params of a tag: the origin, ``POS_YAW_DEG`` (the fleet
#: verification compares both across a cell) and ``CELL_ID`` (cell naming),
#: which are optional on older registries: the refill repairs dump-loss
#: holes in them. A name genuinely absent from an older registry costs the
#: capped retry rounds and is then left alone.
TAG_IDENTITY_PARAMS = (*ORIGIN_PARAMS, "POS_YAW_DEG", "CELL_ID")

#: how often an anchor's ``UWB_LOC`` is read again at most, in seconds, while
#: it advertises another survey ID than the cached value holds
LOCATION_READ_INTERVAL = 5.0

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
        #: latest health-telemetry snapshot per device (server body shape);
        #: a lost device's last snapshot stays, marked ``stale``
        self._stats: dict[int, dict[str, Any]] = {}
        #: devices that went silent, as last seen (the SDK's device object):
        #: listed offline in X-RTLS-INF until an operator forgets them
        #: (X-RTLS-FORGET) or the extension stops. Never consulted as live.
        self._lost: dict[int, RtlsDevice] = {}
        #: decoded parameters of a lost device that came back, standing in
        #: for its role and name until its new snapshot is complete (the
        #: SDK starts a rediscovered device with an empty cache)
        self._retained_params: dict[int, dict[str, Any]] = {}
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
        #: SMP (MCUmgr) port per configured management address. Hardware is
        #: always :1337; simulated devices share one loopback host and each
        #: publish their own port in the sim manifest, which the generated
        #: config forwards (rtlslink.sim.gcs_config_from_manifest).
        self._smp_ports: dict[tuple[str, int], int] = {}
        #: an X-RTLS-CELL apply is running (cell_settings.py: one at a time)
        self._cell_applying = False
        #: the anchor survey (survey.py: one at a time)
        self._survey = Survey(self)
        #: True while an X-RTLS-VERIFY run is in flight; a second
        #: concurrent run is refused (fleet-wide MAVLink param reads
        #: must not be hammered)
        self._verify_running = False
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
        #: live anchor beacons we own, keyed by the anchor's X-RTLS-INF id
        #: -> (beacon, ExitStack-less context)
        self._anchor_beacons: dict[str, tuple[Any, Any]] = {}
        #: each tag's anchor table, assembled over the rotation of its
        #: ``antbl`` stat
        self._anchor_tables: dict[int, AnchorTable] = {}
        #: the advertised survey ID an anchor's ``UWB_LOC`` was last read
        #: again for, and when (see :meth:`_follow_advertised_survey`)
        self._location_asked: dict[int, tuple[float, float]] = {}
        #: wall-clock estimate of the UWB cluster clock that the drones count
        #: the show start in
        self._cluster_clock = ClusterClock()
        #: latest inter-anchor TWR telemetry per device:
        #: system_id -> {peer_mac: (measured_distance_m, harvest_monotonic_s)}.
        #: Surfaced in X-RTLS-INF (with a derived age); pruned on device loss.
        self._twr: dict[int, dict[int, tuple[float, float]]] = {}
        #: latest complete inter-anchor TWR summary generation per device:
        #: system_id -> {peer_mac: (mad_m, count, harvest_monotonic_s)}
        self._twr_summary: dict[int, dict[int, tuple[float, int, float]]] = {}
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
        #: latest uptime per device from its periodic SYSTEM_TIME, in ms;
        #: pruned on device loss
        self._uptime_ms: dict[int, int] = {}
        #: last-known sleep latch per device: system_id -> (sleeping, stamp)
        #: where the stamp is the monotonic time of the heartbeat (or
        #: authoritative sleep/wake transaction) that reported it; drives
        #: the X-RTLS-INF push on an ACTIVE<->STANDBY flip
        self._sleeping: dict[int, tuple[bool, float]] = {}
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
        #: the heartbeat interval
        self._refill_interval = max(DEFAULT_HEARTBEAT_INTERVAL, REFILL_MIN_INTERVAL)
        #: lazy proxy of the mavlink extension's API (``None`` when the
        #: extension module is unavailable); resolves the MAVLink system id
        #: each tag reports for its flight controller to a server UAV id
        self._mavlink_api = None
        #: each device's flight-controller claim and the server UAV it names
        #: (system_id -> flockwave UAV id), recomputed continuously by
        #: :meth:`_poll_uav_map` -- never persisted
        self._fc_claims: dict[int, FlightController] = {}
        self._uav_map: dict[int, str] = {}
        #: whether a flight may be authorized only with the anchors locked
        self._lock_required_for_show = True

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
        targets, self._smp_ports = _configured_devices(
            configuration.get("devices", []), port
        )
        listen_port = int(configuration.get("listen_port", DEFAULT_LISTEN_PORT))
        register_beacons = bool(configuration.get("register_beacons", True))
        self._lock_required_for_show = bool(
            configuration.get("lock_required_for_show", True)
        )
        self._cluster_clock.log = self.log
        self._beacon_api = app.import_api("beacon") if register_beacons else None

        # Lazy import: the proxy resolves once the mavlink extension loads
        # (its ``loaded`` flag gates every use), so no load-order dependency
        # is introduced and the rtls extension keeps working without it --
        # devices simply stay unassociated.
        try:
            self._mavlink_api = app.import_api("mavlink")
        except (KeyError, RuntimeError):
            self._mavlink_api = None

        heartbeat_interval = float(
            configuration.get("heartbeat_interval", DEFAULT_HEARTBEAT_INTERVAL)
        )
        device_timeout = float(
            configuration.get("device_timeout", DEFAULT_DEVICE_TIMEOUT)
        )
        self._refill_interval = max(heartbeat_interval, REFILL_MIN_INTERVAL)

        sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
        try:
            # no SO_REUSEADDR: a second process on the port must fail here
            # instead of silently taking a share of the boards' streams
            await sock.bind(("0.0.0.0", listen_port))
        except OSError as ex:
            sock.close()
            raise RuntimeError(
                f"rtls: cannot bind the management socket to UDP :{listen_port} "
                f"({ex}); the boards stream their telemetry to this port"
            ) from ex
        self._sock = sock

        dialect = load_dialect()
        self._protocol = protocol = RtlsProtocol(
            dialect,
            targets=targets,
            heartbeat_interval=heartbeat_interval,
            device_timeout=device_timeout,
        )

        logger.info(
            f"rtls: listening for board streams on UDP :{sock.getsockname()[1]}, "
            f"{len(targets)} listed devices heartbeated on UDP :{port}"
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
                        "X-RTLS-POS": self._handle_RTLS_POS,
                        "X-RTLS-VERIFY": self._handle_RTLS_VERIFY,
                        "X-RTLS-FORGET": self._handle_RTLS_FORGET,
                        "X-RTLS-LOCK": self._handle_RTLS_LOCK,
                        "X-RTLS-CELL": self._handle_RTLS_CELL,
                        "X-RTLS-SURVEY": self._handle_RTLS_SURVEY,
                    }
                ):
                    await self._run_protocol_loop(protocol, sock, logger)
        finally:
            sock.close()
            self._forget_run_state()

    def _forget_run_state(self) -> None:
        """What a stopped extension keeps of no device."""
        self._nursery = None
        self._sock = None
        self._survey = Survey(self, self._survey.revision + 1)
        self._uptime_ms.clear()
        self._lost.clear()
        self._retained_params.clear()
        self._sleeping.clear()
        self._sleep_pins.clear()
        self._anchor_tables.clear()
        self._location_asked.clear()
        self._mavlink_api = None
        self._fc_claims.clear()
        self._uav_map.clear()
        self._beacon_api = None
        self._clear_anchor_beacons()

    async def _run_protocol_loop(self, protocol, sock, logger) -> None:
        while True:
            now = time.monotonic()
            for payload, address in protocol.poll(now):
                await self._send(payload, address)

            await self._process_expiry(now, logger)

            # re-resolve each tag's flight-controller claim against the known
            # UAVs; a change pushes the (throttled) device list
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

    async def _process_datagram(
        self, data: bytes, address: tuple[str, int], now: float
    ) -> None:
        if self._protocol is None:
            return
        self._scan_pos_debug(data, now)
        events = self._protocol.feed(data, address, now)
        # A device's whole health report arrives as one datagram, one
        # NAMED_VALUE_FLOAT per stat, and every stats event carries the SDK's
        # accumulated snapshot: the device's last one is the full report, so
        # the earlier ones are neither handled nor dispatched.
        last_stats = {e.system_id: e for e in events if e.kind == "stats"}
        for event in events:
            if event.kind == "stats" and last_stats[event.system_id] is not event:
                continue
            if event.kind in _LIVE_ONLY_EVENTS and event.system_id in self._lost:
                # a lost device's late datagram: its retained snapshot stays
                # as it was until a heartbeat rediscovers it
                continue
            if event.kind == "discovered":
                # a lost device that returns is the same entry, live again,
                # named from what it reported before until its dump lands
                lost = self._lost.pop(event.system_id, None)
                if lost is not None:
                    self._retained_params[event.system_id] = _decoded_device_params(
                        lost
                    )
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
                self._sync_anchor_beacons()
                # push the new device list so clients that only listen (the
                # GUI queries X-RTLS-INF once) see the device appear
                await self._on_inf_change(now)
            elif event.kind == "heartbeat":
                # heartbeats re-latch the device's sleep state; a flip must reach
                # clients now, not on the 10 s slow INF refresh
                await self._track_sleeping(
                    event.system_id, event.data.get("sleeping"), now
                )
            elif event.kind == "uptime":
                if event.system_id in self._protocol.devices:
                    self._uptime_ms[event.system_id] = int(event.data["uptime_ms"])
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
                self._on_twr_summary(event.system_id, event.data, now)
            elif event.kind == "param_value":
                # a freshly learned parameter may be an anchor's position,
                # its MAC or role, or the origin a tag holds
                self._on_param_cached(event.data["name"])
                # a reply to a refill round may have just plugged the last
                # hole: finish promptly instead of on the next poll round
                self._check_refill_completion(event.system_id)
                self._release_retained_params(event.system_id)
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

    def _on_twr_summary(self, system_id: int, data: dict[str, Any], now: float) -> None:
        """Cache the spread (MAD) and sample count of one complete TWR
        summary generation; the generation replaces the previous one."""
        self._twr_summary[system_id] = {
            int(entry["peerMac"]): (float(entry["madM"]), int(entry["count"]), now)
            for entry in data.get("ranges", ())
        }

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
        exact: bool = False,
        resend_after: float | None = None,
    ) -> dict[str, Any]:
        """Writes a single parameter and waits for the device-side ack.
        With ``exact``, an accepted ack of another value is taken for a late
        ack of an earlier write and skipped. With ``resend_after``, the write
        is sent again every so many seconds while no ack comes, which only a
        write that may arrive twice allows.
        Returns ``{"value", "type", "result", "accepted"}`` where
        ``value`` is the value *acknowledged by the device* (decoded) and
        ``result`` is the raw PARAM_ACK code.

        The parameter type may be omitted if it is already known from an
        earlier parameter listing (devices are listed automatically on
        discovery). Raises KeyError for unknown devices, ValueError if
        the type cannot be determined and trio.TooSlowError on timeout,
        which also bounds the wait behind an earlier write of the same
        parameter: a write still queued when it expires is never sent."""
        deadline = trio.current_time() + timeout
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
                if (
                    exact
                    and event.data["result"] == PARAM_ACK_ACCEPTED
                    and decode_param_value(event.data["value"], param_type) != value
                ):
                    continue  # an earlier write's late ack
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
            with trio.fail_at(deadline):
                await lock.acquire()
            try:
                # re-resolve the device: while this write queued behind
                # the lock, the device may have been lost and
                # rediscovered under the same system id — mirroring the
                # ack into the pre-lock object would leave the LIVE
                # cache stale
                device = protocol.devices.get(system_id)
                if device is None:
                    raise KeyError(system_id)
                with self._subscribed_events() as events:
                    request = protocol.set_param(system_id, name, encoded, param_type)
                    if request is None:
                        raise KeyError(system_id)

                    async def send_until_acked():
                        while True:
                            await self._send(*request)
                            with trio.move_on_after(resend_after or math.inf):
                                return await wait_for_final_ack(events)

                    try:
                        with trio.fail_at(deadline):
                            event = await send_until_acked()
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
                            self._on_param_cached(name)
                    return {
                        "value": decode_param_value(event.data["value"], param_type),
                        "type": param_type_to_name(param_type),
                        "result": result,
                        "accepted": result == PARAM_ACK_ACCEPTED,
                    }
            finally:
                lock.release()
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
        timeout: float | None = None,
    ) -> dict[str, Any]:
        """Puts a device to sleep (``sleeping=True``) or wakes it through
        the ``SLEEP`` parameter: a tag as :meth:`_set_tag_sleep` describes,
        verified; an anchor as :func:`.anchor_sleep.set_anchor_sleep` does,
        answered by the write's acknowledgement. ``timeout`` defaults per
        role. A device whose role cannot be established is not
        commanded.

        Returns ``{"requested", "accepted", "sleeping", "detail"}``.
        Raises KeyError for unknown devices and trio.TooSlowError on
        timeout."""
        if self._survey.involves(system_id):
            return sleep_error(sleeping, "A survey is running")
        role = await self._command_role(system_id, timeout)
        if role is None:
            return sleep_error(
                sleeping,
                f"Device {system_id}: role unknown (UWB_ROLE not read); not commanded",
            )
        if role in ANCHOR_ROLES:
            result = await set_anchor_sleep(
                self,
                system_id,
                sleeping,
                timeout=ANCHOR_WRITE_TIMEOUT if timeout is None else timeout,
            )
            return {**result, **self._refusal_detail(system_id, result["accepted"])}
        return await self._set_tag_sleep(
            system_id,
            sleeping,
            settle=settle,
            timeout=DEFAULT_SLEEP_TIMEOUT if timeout is None else timeout,
        )

    async def _command_role(self, system_id: int, timeout: float | None) -> str | None:
        """A live device's role for a command: its cached ``UWB_ROLE``, a
        complete snapshot without one (a legacy tag), or a fresh read within
        the request's deadline; None when none of them gives it. Never the
        role retained from before a loss.

        Raises KeyError when the device is not live."""
        device = self._require_protocol().devices.get(system_id)
        if device is None:
            raise KeyError(system_id)
        role = role_from_params(_decoded_device_params(device, ("UWB_ROLE",)))
        if role is not None:
            return role
        if lacks_param(device, "UWB_ROLE"):
            return "tag"
        read_timeout = min(timeout or DEFAULT_PARAM_TIMEOUT, DEFAULT_PARAM_TIMEOUT)
        try:
            result = await self.get_param(system_id, "UWB_ROLE", timeout=read_timeout)
        except trio.TooSlowError:
            return None
        return role_from_params({"UWB_ROLE": result["value"]})

    def _fresh_stats(self, system_id: int) -> dict[str, Any] | None:
        """A device's stats snapshot unless it is stale (kept from before a
        loss, or from a stream that went silent)."""
        stats = self._stats.get(system_id)
        return None if stats is None or stats.get("stale") else stats

    def _power_state(self, system_id: int) -> str | None:
        """A device's current ``powerState`` from its fresh stats; None
        without one (firmware without ``pwrst``, or a stale snapshot)."""
        state = (self._fresh_stats(system_id) or {}).get("powerState")
        return None if state in (None, "unknown") else state

    async def _set_tag_sleep(
        self,
        system_id: int,
        sleeping: bool,
        *,
        settle: float,
        timeout: float,
    ) -> dict[str, Any]:
        """Puts a tag to sleep or wakes it. The firmware refuses a sleep
        request while its flight controller is armed (or not confirmed
        disarmed, in strict gate mode) by flipping the parameter back to 0
        shortly after the ack, so a sleep request is read back after
        ``settle`` seconds. On hardware a woken tag reboots shortly after
        the ack (to re-initialize the power-cycled UWB module) and drops
        off the network for a few seconds."""
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
                state = await self.get_param(system_id, SLEEP_PARAM, timeout=timeout)
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

    async def start_ota(
        self, system_id: int, image_path: str, *, role: str | None = None
    ) -> dict[str, Any]:
        """Starts an OTA upgrade for a device in the background; returns
        a snapshot of the newly created job record, which carries the
        image's ``species`` and where it was established
        (``speciesSource``).

        The uploaded image is confirmed (permanent), and a wrong-species
        image strips the device of its function, so the device's UWB_ROLE
        must match the image's species: from the image's build receipt, or,
        for an image without one, the ``role`` the caller declares. An image
        with neither is refused.

        Raises KeyError for unknown devices, ValueError for a refused image,
        trio.TooSlowError when the role read times out, and RuntimeError if
        an OTA job is already running for the device or the extension is
        not running."""
        if system_id not in self._require_protocol().devices:
            raise KeyError(system_id)
        if self._survey.involves(system_id):
            raise RuntimeError("A survey is running")
        species, source = _image_species(image_path, role)
        if species is None:
            raise ValueError(
                "the image has no build receipt and no declared role; build "
                "it with rtls-link-zephyr scripts/build.sh (refusing a "
                "permanent upload of an unchecked image)"
            )
        await self.verify_species(system_id, species)
        device, nursery = self._require_ota_slot(system_id)

        job = {
            "id": system_id,
            "image": image_path,
            "status": "running",
            "progress": 0.0,
            "version": None,
            "error": None,
            "species": species,
            "speciesSource": source,
        }
        self._ota_jobs[system_id] = job
        nursery.start_soon(self._run_ota, device.address[0], image_path, job)
        return dict(job)

    def _require_ota_slot(self, system_id: int) -> tuple[RtlsDevice, trio.Nursery]:
        """The live device and the nursery to run its job, when an OTA job
        may start for it now."""
        device = self._require_protocol().devices.get(system_id)
        if device is None:
            raise KeyError(system_id)
        job = self._ota_jobs.get(system_id)
        if job is not None and job["status"] == "running":
            raise RuntimeError(f"OTA already in progress for device {system_id}")
        if self._nursery is None:
            raise RuntimeError("extension is not running")
        return device, self._nursery

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
            locked = locked_detail(self, job["id"])
            job["status"] = "error"
            job["error"] = error if locked is None else f"{locked} ({error})"
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

    async def _on_stats(self, system_id: int, data: dict[str, Any], now: float) -> None:
        """Cache the latest health-telemetry snapshot for a device and
        broadcast it to clients, throttled to at most one per
        ``STATS_INTERVAL`` per device.

        The firmware emits one NAMED_VALUE_FLOAT per stat, all of a report in
        one datagram, and :meth:`_process_datagram` hands over only the last
        stats event of a datagram, so this runs once per report. The SDK's
        accumulated ``data`` is only complete once a full cycle has arrived.
        We cache every update (so the X-RTLS-STATS query always has the
        latest), but only broadcast once the full stat set is present --
        otherwise the first notification after discovery would carry a
        half-populated snapshot.

        On the leading edge of the throttle window we broadcast immediately;
        a complete update that lands inside the window is only cached here --
        the periodic flush in :meth:`_run_protocol_loop` pushes the latest
        snapshot once the window elapses, so newer values are never dropped."""
        role = self._device_role(system_id)
        body = _stats_json(system_id, data, role)
        temperatures = self._imu_temperatures(system_id)
        if temperatures:
            # The drone's own telemetry rides its tag's snapshot so that one
            # row shows the whole pre-show health of a drone
            body["imuTemperaturesC"] = temperatures
        table = self._anchor_table_json(system_id, data)
        if table is not None:
            body["anchorTable"] = table
        previous = self._stats.get(system_id) or {}
        self._survey.keep_session(system_id, body, previous)
        # anchorLock and cellSettings follow the lock, cellcfg and txgain stats;
        # an anchor's INF sleeping follows its powerState (standby <-> asleep
        # keeps the same heartbeat)
        if (
            any(
                body.get(key) != previous.get(key)
                for key in ("locked", "cellConfig", "txGain")
            )
            or (body.get("powerState") == "asleep")
            != (previous.get("powerState") == "asleep")
            or survey_inputs(body) != survey_inputs(previous)
        ):
            self._inf_pending = True
        self._stats[system_id] = body
        self._stats_at[system_id] = now
        self._survey.on_stats(system_id, body)
        self._cluster_clock.on_stats(system_id, data)
        if _is_anchor_stats(role):
            await self._follow_advertised_survey(system_id, data, now)
        # STATS_FIELDS is the SDK's required legacy set of a tag; newer stats
        # (``slp`` sleep state, ``vbat`` battery voltage, the cluster clock)
        # are optional and must not gate the broadcast, or firmware without
        # them would never get a health snapshot out. Anchors send none of
        # the set, so their reports are not gated at all.
        if not _is_anchor_stats(role) and not all(
            field in data for field in STATS_FIELDS
        ):
            return
        last = self._last_stats_broadcast.get(system_id)
        if last is not None and now - last < STATS_INTERVAL:
            return
        await self._broadcast_stats(system_id, now)

    def _anchor_table_json(
        self, system_id: int, data: dict[str, Any]
    ) -> list[dict[str, Any]] | None:
        """A tag's anchor table after the ``antbl`` of this report, each
        anchor with the system ID of the device that has its MAC (null while
        no known anchor has); None while the tag reports no table."""
        value = _finite_stat(data, TABLE_STAT)
        if value is None:
            self._anchor_tables.pop(system_id, None)
            return None
        table = self._anchor_tables.setdefault(system_id, AnchorTable())
        table.report(value, _finite_stat(data, "anexc"))
        system_ids = self._anchor_system_ids()
        return [
            {
                "row": entry.row,
                "mac": entry.mac,
                "state": entry.state,
                "systemId": system_ids.get(entry.mac),
            }
            for entry in table.entries
        ]

    async def _follow_advertised_survey(
        self, system_id: int, data: dict[str, Any], now: float
    ) -> None:
        """Reads an anchor's ``UWB_LOC`` again while the survey ID it
        advertises is not the cached one: its position was written past this
        server, or the read of it was lost. At once for an ID not asked for
        yet, then every ``LOCATION_READ_INTERVAL``."""
        advertised = _finite_stat(data, "sid")
        device = self._protocol.devices.get(system_id) if self._protocol else None
        if advertised is None or advertised < 0 or device is None:
            return
        stored = stored_location(device)
        cached = 0 if stored is None else stored.location.survey_id
        asked_for, asked_at = self._location_asked.get(system_id, (None, 0.0))
        if advertised == cached:
            self._location_asked.pop(system_id, None)
        elif asked_for != advertised or now - asked_at >= LOCATION_READ_INTERVAL:
            self._location_asked[system_id] = (advertised, now)
            request = self._require_protocol().request_param_read(
                system_id, anchor_location.PARAM
            )
            if request is not None:
                await self._send(*request)

    async def _flush_pending_stats(self, now: float) -> None:
        """Broadcast any cached stats snapshot that is newer than the last
        one sent and whose throttle window has elapsed (trailing edge). A
        snapshot not refreshed within STATS_STALE_AFTER is marked stale
        first, so the push tells clients the stream went silent."""
        for system_id, stats in list(self._stats.items()):
            harvested = self._stats_at.get(system_id)
            if (
                not stats.get("stale")
                and harvested is not None
                and now - harvested > STATS_STALE_AFTER
            ):
                stats = self._stats[system_id] = {**stats, "stale": True}
                self._anchor_tables.pop(system_id, None)
                # a silent tag no longer says which anchors it leaves out
                self._inf_pending = True
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
            if self._protocol is None or system_id not in self._protocol.devices:
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
                    if stamp > current or current - stamp > POS_REBOOT_BACKSTEP_MS:
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

    def _expire_devices(self, now: float) -> list[ProtocolEvent]:
        """Expire the devices that went silent past their timeout, keeping
        each one's last state (see :meth:`_handle_lost`); returns the
        ``lost`` events."""
        protocol = self._require_protocol()
        known = dict(protocol.devices)
        events = protocol.expire(now)
        for event in events:
            if event.kind == "lost":
                self._handle_lost(event, known.get(event.system_id))
        return events

    async def _process_expiry(self, now: float, logger) -> None:
        """The protocol loop's expiry pass: the lost devices are logged."""
        for event in self._expire_devices(now):
            if event.kind == "lost":
                logger.warning(f"rtls: device sysid {event.system_id} lost")

    def _handle_lost(
        self, event: ProtocolEvent, device: RtlsDevice | None = None
    ) -> None:
        """React to a device-``lost`` event: keep the device as last seen
        (listed offline in X-RTLS-INF) with its last stats marked stale, drop
        the live-only state (TWR telemetry, position stream, uptime, sleep
        latch, anchor table), then forward the event to subscribers. Also
        marks the device list dirty; the periodic flush in the protocol loop
        broadcasts the updated X-RTLS-INF and the stale stats (this handler
        is synchronous, so it cannot broadcast itself)."""
        if device is not None:
            self._lost[event.system_id] = device
        stats = self._stats.get(event.system_id)
        if stats is not None:
            self._stats[event.system_id] = {**stats, "stale": True}
        self._anchor_tables.pop(event.system_id, None)
        self._location_asked.pop(event.system_id, None)
        self._survey.on_lost(event.system_id)
        self._prune_pos(event.system_id)
        self._refill.pop(event.system_id, None)
        self._twr.pop(event.system_id, None)
        self._twr_summary.pop(event.system_id, None)
        self._uptime_ms.pop(event.system_id, None)
        self._sleeping.pop(event.system_id, None)
        # _sleep_pins deliberately survives loss: a woken device reboots off
        # the network (lost + rediscovered) inside its pin window
        self._cluster_clock.forget_device(event.system_id)
        self._sync_anchor_beacons()
        self._inf_pending = True
        self._dispatch_event(event)

    def _prune_stats(self, system_id: int) -> None:
        """Drop all cached stats state for a device (when it is forgotten)."""
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
            and (self._protocol.devices or self._lost)
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
        await hub.broadcast_message(hub.create_notification(self._inf_body(now)))

    # ---- tag<->drone association ----

    def _uav_ids_by_system_id(self) -> dict[int, str]:
        """The flockwave id of every UAV the mavlink extension has heard
        from, connected or not, keyed by MAVLink system id, from the mavlink
        extension's API (empty while the extension is not loaded). Tests
        override this seam."""
        api = self._mavlink_api
        if api is None or not getattr(api, "loaded", False):
            return {}
        try:
            return api.get_uav_ids_by_system_id()
        except Exception:
            return {}

    def _imu_temperatures(self, system_id: int) -> list[float | None]:
        """The IMU temperatures of the drone a device belongs to, from the
        mavlink extension's API (empty without a paired drone, a reading or
        the extension). Tests override this seam."""
        uav_id = self._uav_map.get(system_id)
        api = self._mavlink_api
        if uav_id is None or api is None or not getattr(api, "loaded", False):
            return []
        try:
            return api.get_imu_temperatures(uav_id)
        except Exception:
            return []

    def _associations(self) -> tuple[dict[int, FlightController], dict[int, str]]:
        """Each device's flight-controller claim (see :mod:`.fc_identity`)
        and the server UAV that claim names."""
        devices = self._protocol.devices if self._protocol else {}
        claims = flight_controller_claims(
            {
                device.system_id: (
                    _decoded_device_params(device, ("FC_SYS_ID", "FC_STATE")),
                    bool(getattr(device, "sleeping", False)),
                )
                for device in devices.values()
            }
        )
        return claims, uav_ids_for_claims(claims, self._uav_ids_by_system_id())

    def _refresh_uav_map(self) -> bool:
        """Recompute the claims and the device->UAV association and return
        whether either changed."""
        claims, mapping = self._associations()
        if claims == self._fc_claims and mapping == self._uav_map:
            return False
        self._fc_claims, self._uav_map = claims, mapping
        return True

    async def _poll_uav_map(self, now: float) -> None:
        """One pass of the association loop: recompute the claims and the
        mapping and, when either changed, push the (throttled) X-RTLS-INF
        device list so clients re-render the tag<->drone pairing without
        polling."""
        if self._refresh_uav_map():
            await self._on_inf_change(now)

    # ---- identity-param snapshot refill ----

    async def _poll_param_refill(self, now: float) -> None:
        """One pass of the identity-param refill: for every device whose
        refill round is due, re-request exactly the role-relevant identity
        params still missing from its snapshot (the startup all-boards
        dump loses datagrams; without this a device keeps a generic name
        and missing anchor beacons until it reboots).

        Rounds are paced by the refill interval and capped at
        ``REFILL_MAX_ATTEMPTS``; a device whose snapshot is complete —
        including the complete-but-unmatched anchor case — sheds its
        refill entry and is never polled again until rediscovery."""
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
                self._nursery.start_soon(self._send_refill_reads, system_id, missing)
            else:
                await self._send_refill_reads(system_id, missing)

    async def _send_refill_reads(self, system_id: int, names: list[str]) -> None:
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
        snapshot (see :func:`_missing_identity_param_names`)."""
        if device.param_count is not None and len(device.params) >= device.param_count:
            # the snapshot is count-complete, so the dump lost nothing:
            # an identity param absent from it is genuinely absent on the
            # device and must not be re-polled
            return []
        params = _decoded_device_params(device)
        return _missing_identity_param_names(params, role_from_params(params))

    # ---- client message handlers ----

    async def _handle_RTLS_INF(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        return hub.create_response_or_notification(
            body=self._inf_body(time.monotonic()), in_response_to=message
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
            return hub.reject(message, reason=self._no_device(system_id))
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
            return hub.reject(message, reason=self._no_device(system_id))
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
        if name == SLEEP_PARAM and (isinstance(value, bool) or value not in (0, 1)):
            # an anchor reads 2 in a standby it is not asleep in: a state,
            # not a remote command (wake and sleep are 0 and 1)
            return hub.reject(
                message,
                reason=f"{SLEEP_PARAM} accepts 0 (awake) or 1 (asleep) remotely",
            )

        try:
            result = await self.set_param(
                system_id, name, value, param_type, timeout=timeout
            )
        except KeyError:
            return hub.reject(message, reason=self._no_device(system_id))
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
                **self._refusal_detail(system_id, result["accepted"]),
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
            # without one, the deadline is the role's: an anchor's is the
            # write's ack, a tag's a parameter write and readback
            timeout = (
                _get_timeout(message, DEFAULT_SLEEP_TIMEOUT)
                if "timeout" in body
                else None
            )
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
                results[key] = sleep_error(sleeping, self._no_device(system_id))
            except ValueError as ex:
                results[key] = sleep_error(sleeping, str(ex))
            except trio.TooSlowError:
                results[key] = sleep_error(
                    sleeping,
                    f"Timeout while waiting for device {system_id}",
                )
            except Exception as ex:  # noqa: BLE001
                # never let one device's failure cancel the siblings
                results[key] = sleep_error(sleeping, str(ex))

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

        role = message.body.get("role")
        if role is not None and role not in ("tag", "anchor"):
            return hub.reject(
                message, reason=f"Invalid role: {role!r} (expected tag or anchor)"
            )
        try:
            job = await self.start_ota(system_id, image, role=role)
        except KeyError:
            return hub.reject(message, reason=self._no_device(system_id))
        except (ValueError, RuntimeError) as ex:
            return hub.reject(message, reason=str(ex))
        except trio.TooSlowError:
            return hub.reject(
                message,
                reason=f"Timeout while reading UWB_ROLE of device "
                f"{system_id} for the species check",
            )

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
            snapshot = {str(sysid): dict(stats) for sysid, stats in self._stats.items()}

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

    async def _handle_RTLS_FORGET(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        body = message.body
        try:
            # a single id follows the ids rule too (JSON true is not 1)
            ids = _get_optional_device_ids(
                body if "ids" in body else {"ids": [body.get("id")]}
            )
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        result = {
            str(system_id): self.forget_device(system_id)
            for system_id in dict.fromkeys(ids or ())
        }
        if any(entry["forgotten"] for entry in result.values()):
            await self._on_inf_change(time.monotonic())
        return hub.create_response_or_notification(
            body={"type": "X-RTLS-FORGET", "result": result},
            in_response_to=message,
        )

    def forget_device(self, system_id: int) -> dict[str, Any]:
        """Drop everything kept about an offline device: its X-RTLS-INF
        entry and its stale stats. A live device cannot be forgotten (it
        would be listed again on its next heartbeat)."""
        if self._protocol is not None and system_id in self._protocol.devices:
            return {"forgotten": False, "detail": f"Device {system_id} is online"}
        if system_id not in self._lost and system_id not in self._stats:
            return {"forgotten": False, "detail": f"No such device: {system_id}"}
        self._lost.pop(system_id, None)
        self._retained_params.pop(system_id, None)
        self._prune_stats(system_id)
        self._sleep_pins.pop(system_id, None)
        self._sync_anchor_beacons()
        return {"forgotten": True}

    async def _handle_RTLS_LOCK(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            body = await handle_lock_request(self, message.body)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        if body["results"]:
            await self._on_inf_change(time.monotonic())
        return hub.create_response_or_notification(body=body, in_response_to=message)

    def show_start_refusal(self) -> str | None:
        """Why a flight may not be authorized or scheduled now, or None (see
        :func:`.anchor_lock.show_gate`). Exported to the show extension."""
        return show_gate(self)["reason"]

    def _refusal_detail(self, system_id: int, accepted: bool) -> dict[str, str]:
        """``{"detail": "RTLS anchor A3 is locked"}`` for a write a locked
        anchor refused, else nothing."""
        locked = None if accepted else locked_detail(self, system_id)
        return {} if locked is None else {"detail": locked}

    async def _handle_RTLS_CELL(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            body = await handle_cell_request(self, message.body)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        if body["results"]:
            await self._on_inf_change(time.monotonic())
        return hub.create_response_or_notification(body=body, in_response_to=message)

    async def _handle_RTLS_SURVEY(
        self, message: "FlockwaveMessage", sender: "Client", hub: "MessageHub"
    ):
        try:
            body = await handle_survey_request(self, message.body)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        return hub.create_response_or_notification(body=body, in_response_to=message)

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
        return hub.create_response_or_notification(body=result, in_response_to=message)

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

    def _no_device(self, system_id: int) -> str:
        """Why a command cannot reach a device that is not live."""
        if system_id in self._lost:
            return f"Device {system_id} is offline"
        return f"No such device: {system_id}"

    def _require_protocol(self) -> RtlsProtocol:
        if self._protocol is None:
            raise RuntimeError("rtls protocol is not running")
        return self._protocol

    def _device_role(self, system_id: int) -> str | None:
        """A live device's role for display and the stats shape: its cached
        ``UWB_ROLE``, else the one it had before it was lost; None while the
        device is offline or its role is not yet known. Commands use
        :meth:`_command_role`."""
        device = self._protocol.devices.get(system_id) if self._protocol else None
        if device is None:
            return None
        params = _decoded_device_params(device, ("UWB_ROLE",))
        if "UWB_ROLE" not in params:
            params = {
                k: v
                for k, v in self._retained_params.get(system_id, {}).items()
                if k == "UWB_ROLE"
            }
        return role_from_params(params)

    def _identity_params(self, device) -> dict[str, Any]:
        """A device's decoded parameters, filled in from what it reported
        before it was lost while its new snapshot is incomplete. For naming
        and display only: live checks read the device's own cache."""
        retained = self._retained_params.get(device.system_id)
        params = _decoded_device_params(device)
        return params if retained is None else {**retained, **params}

    def _release_retained_params(self, system_id: int) -> None:
        """Drop a returned device's retained parameters once its new
        snapshot is count-complete."""
        if system_id not in self._retained_params or self._protocol is None:
            return
        device = self._protocol.devices.get(system_id)
        if (
            device is not None
            and device.param_count is not None
            and len(device.params) >= device.param_count
        ):
            del self._retained_params[system_id]

    def _inf_body(self, now: float) -> dict[str, Any]:
        """The X-RTLS-INF body of the query response and the broadcast."""
        cell, cell_states = cell_view(self)
        return {
            "type": "X-RTLS-INF",
            "status": self._inf_status(now, cell_states),
            "anchors": [entry for _, entry in self._site_anchors()],
            "survey": survey_json(self, anchor_views(self), now),
            "anchorLock": lock_json(self),
            "cellSettings": cell,
        }

    def _inf_status(
        self,
        now: float,
        cell_states: dict[int, tuple[str, str | None]] | None = None,
    ) -> dict[str, dict[str, Any]]:
        """The full X-RTLS-INF status map: one entry per live or lost
        device, keyed by system id (as string), with each live device's cell
        settings state (``cell_states``, from :func:`cell_view` unless
        given)."""
        devices = self._protocol.devices if self._protocol else {}
        claims, uav_ids = self._associations()
        status = {
            str(device.system_id): self._device_json(
                device, now, claims.get(device.system_id), uav_ids.get(device.system_id)
            )
            for device in devices.values()
        }
        for system_id, device in self._lost.items():
            if system_id not in devices:
                status[str(system_id)] = self._device_json(
                    device, now, None, None, online=False
                )
        if cell_states is None:
            cell_states = cell_view(self)[1]
        for system_id, (state, detail) in cell_states.items():
            status[str(system_id)]["cellSettings"] = state
            if detail is not None:
                status[str(system_id)]["cellSettingsDetail"] = detail
        return status

    def _device_json(
        self,
        device,
        now: float,
        flight_controller: FlightController | None,
        uav_id: str | None,
        *,
        online: bool = True,
    ) -> dict[str, Any]:
        job = self._ota_jobs.get(device.system_id)
        params = self._identity_params(device)
        role = role_from_params(params)

        body = {
            "id": device.system_id,
            "online": online,
            "address": list(device.address),
            "age": round(now - device.last_seen, 3),
            "firmwareVersion": firmware_version(device),
            "paramCount": device.param_count,
            "otaStatus": job["status"] if job is not None else None,
        }
        # an anchor in standby or off heartbeats STANDBY too: only its
        # power state says whether it is asleep
        if role in ANCHOR_ROLES:
            power_state = self._power_state(device.system_id) if online else None
            body["sleeping"] = power_state == "asleep"
        else:
            # getattr: tolerate an SDK that predates sleep mode
            body["sleeping"] = bool(getattr(device, "sleeping", False))
        uptime_ms = self._uptime_ms.get(device.system_id)
        if uptime_ms is not None:
            body["uptimeMs"] = uptime_ms
        if uav_id is not None:
            body["uav"] = uav_id
        if flight_controller is not None:
            body["flightController"] = flight_controller
        if role is not None:
            body["role"] = role
        if role == "tag":
            body["cell"] = _cell_id_from_params(params)
        name = _device_name(device.system_id, params, role)
        if name is not None:
            body["name"] = name
        twr = self._twr_rows(device.system_id, now)
        if twr:
            body["twr"] = twr
        return body

    def _twr_rows(self, system_id: int, now: float) -> list[dict[str, Any]]:
        """One X-RTLS-INF row per heard TWR peer, freshest first, with the
        age derived from the harvest stamp so the client can flag stale
        (peer-quiet) rows. A row carries the spread and sample count of the
        summary generation its distance came with: a summary harvested no
        earlier than the row (the firmware sends a generation in one
        datagram, which also carries the legacy distance frame)."""
        summary = self._twr_summary.get(system_id, {})
        rows = []
        for peer_mac, (distance_m, harvested) in sorted(
            self._twr.get(system_id, {}).items(),
            key=lambda kv: kv[1][1],
            reverse=True,
        ):
            row: dict[str, Any] = {
                "peerMac": peer_mac,
                "distanceM": round(distance_m, 3),
                "ageMs": round(max(0.0, now - harvested) * 1000.0),
            }
            spread = summary.get(peer_mac)
            if spread is not None and spread[2] >= harvested:
                row["madM"] = round(spread[0], 4)
                row["count"] = spread[1]
            rows.append(row)
        return rows

    # ---- site / cell / anchor beacons ----

    def _site_anchors(self) -> list[tuple[int, dict[str, Any]]]:
        """The X-RTLS-INF ``anchors`` list with each anchor's system ID:
        every known anchor, online or lost, by cell and by its row in its
        own ``UWB_AN*_MAC`` table (see :meth:`_site_anchor`)."""
        devices = self._protocol.devices if self._protocol else {}
        known = [(device, True) for device in devices.values()] + [
            (device, False)
            for system_id, device in self._lost.items()
            if system_id not in devices
        ]
        origins = self._cell_origins()
        anchors = []
        for device, online in known:
            params = self._identity_params(device)
            if role_from_params(params) in ANCHOR_ROLES:
                entry = _site_anchor(device.system_id, params, online, origins)
                anchors.append((device.system_id, entry))
        anchors.sort(
            key=lambda anchor: (
                anchor[1]["cell"],
                anchor[1]["index"] is None,
                anchor[1]["index"] or 0,
                anchor[0],
            )
        )
        return anchors

    def _cell_origins(self) -> dict[str, Origin]:
        """Each cell's origin as its lowest online tag holds it: the anchors
        store positions in the cell frame, and only a tag says where on the
        map that frame is."""
        devices = self._protocol.devices if self._protocol else {}
        names = (*ORIGIN_PARAMS, "UWB_ROLE", "CELL_ID", "RTLS_CELL_ID")
        origins: dict[str, Origin] = {}
        for _, device in sorted(devices.items()):
            params = _decoded_device_params(device, names)
            if role_from_params(params) == "tag" and all(
                name in params for name in ORIGIN_PARAMS
            ):
                lat_e7, lon_e7, alt_mm = (int(params[name]) for name in ORIGIN_PARAMS)
                origins.setdefault(
                    _cell_id_from_params(params), Origin(lat_e7, lon_e7, alt_mm)
                )
        return origins

    def _anchor_system_ids(self) -> dict[int, int]:
        """The system ID of the known anchor that has each ``UWB_MAC``; an
        online anchor before a lost one."""
        devices = self._protocol.devices if self._protocol else {}
        system_ids: dict[int, int] = {}
        for device in (*devices.values(), *self._lost.values()):
            params = _decoded_device_params(device, ("UWB_ROLE", "UWB_MAC"))
            if role_from_params(params) in ANCHOR_ROLES and "UWB_MAC" in params:
                system_ids.setdefault(int(params["UWB_MAC"]), device.system_id)
        return system_ids

    def _on_param_cached(self, name: str) -> None:
        """A parameter entered a device's cache: the anchors on the map
        follow, and a position that changed is pushed."""
        self._sync_anchor_beacons()
        if name == anchor_location.PARAM:
            self._inf_pending = True

    def _sync_anchor_beacons(self) -> None:
        """Renders every anchor that has a map position as a beacon and
        drops the beacon of every other."""
        if self._beacon_api is None:
            return
        placed = set()
        for system_id, anchor in self._site_anchors():
            position = anchor["position"]
            beacon = (
                self._get_or_create_anchor_beacon(anchor["id"]) if position else None
            )
            if beacon is None:
                continue
            placed.add(anchor["id"])
            index = anchor["index"]
            beacon.basic_properties.name = (
                f"RTLS anchor {system_id}" if index is None else f"RTLS A{index}"
            )
            beacon.update_status(
                position=GPSCoordinate(
                    lat=position["lat"], lon=position["lon"], amsl=position["amsl"]
                ),
                active=anchor["active"],
            )
        for beacon_id in list(self._anchor_beacons):
            if beacon_id not in placed:
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

    def exports(self):
        return {
            "cluster_clock": self._cluster_clock,
            "devices": self._get_devices,
            "protocol": lambda: self._protocol,
            "get_param": self.get_param,
            "get_param_list": self.get_param_list,
            "set_param": self.set_param,
            "set_sleep": self.set_sleep,
            "forget_device": self.forget_device,
            "show_start_refusal": self.show_start_refusal,
            "start_ota": self.start_ota,
            "verify_species": self.verify_species,
        }

    def _get_devices(self):
        if self._protocol is None:
            return {}
        return dict(self._protocol.devices)


def _image_species(image: str, declared: str | None) -> tuple[str | None, str | None]:
    """The species (``"tag"`` / ``"anchor"``) of an OTA image and its
    source: the image's build receipt (rtls-link-zephyr ``scripts/build.sh``),
    else the role the client declared, else unknown. Raises ValueError when
    the receipt cannot vouch for the image (stale, unreadable, an unknown
    app) or contradicts the declared role."""
    try:
        receipt = read_receipt(image)
    except (json.JSONDecodeError, KeyError, TypeError, OSError) as ex:
        raise ValueError(f"unreadable image receipt beside {image}: {ex}") from ex
    if receipt is None:
        return declared, ("declared" if declared else None)
    if declared is not None and declared != receipt.app:
        raise ValueError(
            f"the image is a {receipt.app} image (its build receipt), not a "
            f"{declared} image; refusing the upload"
        )
    return receipt.app, "receipt"


#: events that only a live device's data may come from
_LIVE_ONLY_EVENTS = frozenset(("stats", "twr", "twr_summary"))


def _configured_devices(
    values, default_port: int
) -> tuple[list[tuple[str, int]], dict[tuple[str, int], int]]:
    """Parse the ``devices`` config into management targets plus the SMP
    port each declares, when it declares one.

    Two entry forms coexist so hardware configs stay untouched: the plain
    ``host`` / ``host:port`` string (SMP at the fixed :1337), and
    ``{"address": ..., "smp_port": ...}`` for simulated devices, whose SMP
    endpoints share one loopback host and therefore live on per-device
    ports (published in the sim manifest)."""
    targets: list[tuple[str, int]] = []
    smp_ports: dict[tuple[str, int], int] = {}
    for value in values:
        if isinstance(value, dict):
            address = parse_address(value["address"], default_port)
            smp_port = value.get("smp_port")
            if smp_port is not None:
                smp_ports[address] = int(smp_port)
        else:
            address = parse_address(value, default_port)
        targets.append(address)
    return targets, smp_ports


def _decoded_device_params(
    device, names: tuple[str, ...] | None = None
) -> dict[str, Any]:
    """Decode a device's cached raw params (all of them, or only ``names``)
    into plain Python values, keyed by name. Parameters whose type is not
    yet known are skipped."""
    params: dict[str, Any] = {}
    for name in device.params if names is None else names:
        param_type = device.param_types.get(name)
        if param_type is None or name not in device.params:
            continue
        params[name] = decode_param_value(device.params[name], param_type)
    return params


def _cell_id_from_params(params: dict[str, Any]) -> str:
    # CELL_ID is the firmware's operator-facing cell label; RTLS_CELL_ID is
    # the older speculative name kept as a fallback.
    value = params.get("CELL_ID") or params.get("RTLS_CELL_ID")
    return str(value) if value not in (None, "") else "default"


def _anchor_beacon_id(cell_id: str, index: int | None, system_id: int) -> str:
    """An anchor's X-RTLS-INF id: by its row in its own table, or by its
    system ID while its ``UWB_MAC`` is in no row of it."""
    if index is None:
        return f"rtls::{cell_id}::device_{system_id}"
    return f"rtls::{cell_id}::anchor_{index}"


def _site_anchor(
    system_id: int, params: dict[str, Any], online: bool, origins: dict[str, Origin]
) -> dict[str, Any]:
    """One entry of the X-RTLS-INF ``anchors`` list: the anchor's row in its
    own table, the position and survey ID it stores itself (``UWB_LOC``;
    ``ned`` is null while it is not surveyed) and, through the origin of its
    cell, where that is on the map (``position``, null without either)."""
    cell_id = _cell_id_from_params(params)
    index = _anchor_index_for_device(params)
    stored = location_from_params(params)
    location = None if stored is None else stored.location
    ned = position = None
    if location is not None:
        north, east, down = location.ned_m
        ned = {
            "north": round(north, 3),
            "east": round(east, 3),
            "down": round(down, 3),
        }
        origin = origins.get(cell_id)
        with suppress(ValueError):
            if origin is not None:
                lat_e7, lon_e7, alt_mm = ned_to_global_e7(origin, north, east, down)
                position = {
                    "lat": round(lat_e7 * 1e-7, 7),
                    "lon": round(lon_e7 * 1e-7, 7),
                    "amsl": round(alt_mm * 1e-3, 3),
                }
    mac = params.get("UWB_MAC")
    return {
        "id": _anchor_beacon_id(cell_id, index, system_id),
        "cell": cell_id,
        "index": index,
        "mac": None if mac is None else int(mac),
        "position": position,
        "ned": ned,
        "surveyId": None if location is None else location.survey_id,
        "planar": location is not None and location.planar,
        "active": online,
        "systemId": system_id if online else None,
    }


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
    # A0's block, slots and slot length make the cell's rate (cell_settings.py)
    rate = A0_RATE_PARAMS if role == "anchor-initiator" else ()
    missing = [name for name in rate if name not in params]
    return missing + _missing_naming_param_names(params, role)


def _missing_naming_param_names(params: dict[str, Any], role: str | None) -> list[str]:
    """The identity params a device's snapshot must gain before its name
    (anchor A-index) or its frame (tag) can resolve — an empty list when
    nothing needs re-requesting.

    Missing means absent from the snapshot, never present-but-mismatched:
    an anchor with a complete MAC table that simply does not contain its
    own UWB_MAC yields an empty list (a real configuration state that must
    not be re-polled forever)."""
    if role in ("anchor-initiator", "anchor-responder"):
        if _anchor_index_for_device(params) is not None:
            return []  # A-index already resolves
        missing = [name for name in ("UWB_MAC", "UWB_AN_COUNT") if name not in params]
        missing.extend(_missing_anchor_table_names(params))
        return missing
    if role == "tag":
        return [name for name in TAG_IDENTITY_PARAMS if name not in params]
    if role is None and "UWB_ROLE" not in params:
        # the device cannot be classified (UWB_ROLE lost with the dump, or
        # a legacy tag that genuinely lacks it). The marker makes the poll
        # re-request the whole dump: holes whose names were never listed
        # cannot be enumerated by targeted reads.
        return ["UWB_ROLE"]
    return []


def _missing_anchor_table_names(params: dict[str, Any]) -> list[str]:
    """The absent ``UWB_AN{i}_MAC`` names for every row an anchor's
    ``UWB_AN_COUNT`` declares — empty when the count itself is absent (it is
    requested first) or undecodable."""
    if "UWB_AN_COUNT" not in params:
        return []
    try:
        count = int(params["UWB_AN_COUNT"])
    except (TypeError, ValueError):
        return []
    count = max(0, min(count, REFILL_MAX_ANCHOR_TABLE))
    return [
        name
        for name in (f"UWB_AN{index}_MAC" for index in range(count))
        if name not in params
    ]


def _rounded(digits: int) -> Callable[[float], float]:
    return lambda value: round(value, digits)


def _code_name(names: dict[int, str]) -> Callable[[float], str]:
    """Converter of a firmware state code (an exact small integer carried as
    a float) to its name in one of the SDK's code tables; any other value is
    ``"unknown"``."""

    def convert(value: float) -> str:
        code = int(value)
        return names.get(code, "unknown") if code == value else "unknown"

    return convert


A0_RATE_PARAMS = ("UWB_BLOCK_MS", "UWB_SLOTS_RR", "UWB_SLOT_RSTU")


def _running_or_none(value: float) -> int | None:
    """``cellcfg`` and ``txgain``: -1 while the device runs no session."""
    return None if value < 0 else int(value)


def _survey_id_or_none(value: float) -> int | None:
    """``sid``: 0 while the session advertises no location, -1 while no
    session runs."""
    return None if value <= 0 else int(value)


#: stats (wire name) carried 1:1 into the X-RTLS-STATS body (key, converter),
#: each present only while its stat is. Non-finite values are dropped.
STATS_BODY_FIELDS: tuple[tuple[str, str, Callable[[float], Any]], ...] = (
    # sleep state; omitted (not False) for firmware without sleep mode
    ("slp", "sleeping", bool),
    ("vbat", "batteryVoltage", _rounded(3)),
    ("soc", "batteryPercent", _rounded(1)),
    # positive while discharging
    ("ibat", "batteryCurrentA", _rounded(3)),
    ("btemp", "batteryTemperatureC", _rounded(1)),
    ("pwrst", "powerState", _code_name(ANCHOR_POWER_STATES)),
    ("pwrin", "powerInput", _code_name(ANCHOR_POWER_INPUTS)),
    ("chg", "charging", _code_name(ANCHOR_CHARGE_PHASES)),
    ("chgblk", "chargeBlock", _code_name(ANCHOR_CHARGE_BLOCKS)),
    ("uwbst", "uwbStatus", int),
    ("rnghz", "rangingRateHz", _rounded(2)),
    ("urst", "radioRestarts", int),
    ("moved", "moved", bool),
    # the anchor's LOCK: it refuses every remote change while set
    ("lock", "locked", bool),
    ("tilt", "tiltDeg", _rounded(1)),
    ("knocks", "knocks", int),
    ("elink", "ethernetLink", bool),
    ("edrop", "ethernetDrops", int),
    # rssi only while associated; wdisc/wtxerr count since boot
    ("rssi", "wifiRssiDbm", int),
    ("wdisc", "wifiDisconnects", int),
    ("wtxerr", "wifiTxErrors", int),
    ("boots", "bootCount", int),
    # the code and TX level of the session the device runs (cell_settings.py)
    ("cellcfg", "cellConfig", _running_or_none),
    ("txgain", "txGain", _running_or_none),
    # the survey ID of the position an anchor's session advertises, and the
    # one a tag's table follows
    ("sid", "surveyId", _survey_id_or_none),
    # an anchor's live range to its stored initiator minus the one its
    # survey measured; absent while it has no reading. As reported: the
    # warning compares it with a bound, and a rounded 0.08004 would not warn
    # where the anchor's own ring does.
    ("rdev", "rangeDeviationM", float),
    *(
        (name, key, int if digits is None else _rounded(digits))
        for name, key, digits in TAG_SURVEY_STATS
    ),
    ("rstc", "resetCause", int),
    # cluster-clock sync freshness (fleet-verify clock rule and the UI)
    ("clkok", "clockSyncOk", bool),
    # the anchors the tag's chip flagged NLOS on most rows and the anchors
    # its solver is outvoting: bit i is row i of the tag's anchorTable
    ("nlosmask", "nlosMask", int),
    ("outmask", "outvotedMask", int),
)

#: the X-RTLS-STATS keys of the tag's solver stats, sent as 0 while the stat
#: is absent; anchors do not solve and omit them
TAG_SOLVER_FIELDS = (
    ("rate", "solveRateHz", float),
    ("solvepct", "solvePct", float),
    ("anc", "anchorsSeen", int),
    ("agems", "fixAgeMs", int),
    ("ppm", "clockPpm", float),
    ("ancmask", "anchorMask", int),
)

ANCHOR_ROLES = ("anchor-initiator", "anchor-responder")


def _finite_stat(data: dict[str, Any], name: str) -> float | None:
    """A stat's value, or None while it is absent or not finite (which can
    be neither JSON nor graded)."""
    if name not in data:
        return None
    value = float(data[name])
    return value if math.isfinite(value) else None


def _is_anchor_stats(role: str | None) -> bool:
    """Whether a stats snapshot takes the anchor shape: only once UWB_ROLE
    says anchor. A tag's 10 s status report carries no solver stats either,
    so an unknown role stays tag-shaped."""
    return role in ANCHOR_ROLES


def _stats_json(
    system_id: int, data: dict[str, Any], role: str | None = None
) -> dict[str, Any]:
    """Map the SDK's NAMED_VALUE_FLOAT stat names to the UI body shape.

    Integer-semantic floats (counts, milliseconds, bitmasks) are cast to int
    and state codes to their names."""
    body: dict[str, Any] = {"id": int(system_id)}
    if not _is_anchor_stats(role):
        for name, key, convert in TAG_SOLVER_FIELDS:
            body[key] = convert(data.get(name, 0.0))
    for name, key, convert in STATS_BODY_FIELDS:
        value = _finite_stat(data, name)
        if value is not None:
            body[key] = convert(value)
    # UWB link quality over the tag's last report interval: the weakest
    # anchor's mean RSSI (0: no row carried a reading) and the solver's mean
    # residual RMS (-1: no solve). A sentinel becomes null.
    rssi = _finite_stat(data, "urssi")
    if rssi is not None:
        body["uwbRssiDbm"] = round(rssi, 1) if rssi < 0.0 else None
    # the pack's level: ok, warning (below its warning level while running
    # from it) or low (latched)
    lowbat = _finite_stat(data, "lowbat")
    if lowbat is not None:
        level = _code_name(ANCHOR_LOW_BATTERY)(lowbat)
        body["lowBattery"] = level == "low"
        body["batteryWarning"] = level == "warning"
    rmse = _finite_stat(data, "rmse")
    if rmse is not None:
        body["residualRmsM"] = round(rmse, 4) if rmse >= 0.0 else None
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
            "title": "Device management addresses to heartbeat from startup; "
            "boards streaming to listen_port are found without them",
            "items": {"type": "string"},
            "default": [],
        },
        "listen_port": {
            "type": "integer",
            "title": "Local UDP port the devices stream their telemetry to "
            "(their MGMT_PORT)",
            "default": DEFAULT_LISTEN_PORT,
        },
        "register_beacons": {
            "type": "boolean",
            "title": "Register configured RTLS anchors as map beacons",
            "default": True,
        },
        "lock_required_for_show": {
            "type": "boolean",
            "title": "Authorize or schedule a flight only while every online "
            "anchor reports its lock",
            "default": True,
        },
    }
}
