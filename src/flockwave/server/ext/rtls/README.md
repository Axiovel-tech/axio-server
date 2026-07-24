# rtls — Axiovel rtls-link device management

Manages rtls-link UWB positioning devices over their management UDP
channel (MAVLink 2, default port 3333) and exposes them to Skybrush
clients through the server's message hub:

- **Discovery / presence**: the extension announces itself with GCS
  heartbeats (unicast to configured `devices` plus `broadcast`
  addresses); each device's management link learns the server from
  inbound datagrams and heartbeats back (component id 197). Devices are
  tracked with a liveness timeout, and their full parameter list is
  fetched automatically on discovery. In addition, the extension
  listens for the firmware's autonomous **state advertisements**
  (UDP :3343, see below) so devices are found and kept fresh without
  being probed; `passive: true` makes advertisements the primary
  presence source.
- **Configuration**: PARAM_EXT list/read/set against the device's
  parameter registry (an accepted set persists on the device).
- **Telemetry**: the firmware's unsolicited health stats are cached and
  re-broadcast as `X-RTLS-STATS`, the tag's opt-in position-estimate
  debug stream as `X-RTLS-POS`, and inter-anchor TWR ranges surface in
  `X-RTLS-INF`. The same stats feed drives the cluster->GPS show-clock
  pin distribution (see below), and the responders' rolling TWR summaries
  feed the anchor-geometry fit (see `X-RTLS-GEO fit`).
- **OTA**: MCUmgr/SMP upload → mark pending → reset, via
  `rtlslink.ota` / `smpclient` (asyncio; run in a worker thread from
  Trio). On the ESP32-S3 MCUboot is overwrite-only — no bootloader
  revert — so the recovery path is health-check + re-upload of the
  previous artifact. `smpclient` is an optional dependency of the SDK and
  the server installs the SDK's `ota` extra because OTA and geometry-sync
  reboot are first-class RTLS operations.

## Firmware requirements

This extension speaks only the **zephyr-generation** rtls-link protocol
(`rtls-link-zephyr`): management heartbeats on component id 197, stats
as `NAMED_VALUE_FLOAT`, PARAM_EXT configuration and MCUmgr/SMP OTA.
Legacy `rtls-link` (ESP32/Arduino) devices use a different dialect
(component id 191, `RTLS_DEVICE_STATUS`/`RTLS_COMMAND`, HTTP OTA) and
are **invisible to this extension by design** — manage them with the
standalone `rtls-link-manager` application instead.

## The rtls-link SDK

The protocol and OTA code lives in the standalone **`rtls-link`**
Python package (module `rtlslink`), maintained in the firmware repo
([Axiovel-tech/rtls-link-zephyr](https://github.com/Axiovel-tech/rtls-link-zephyr),
directory `py/`) and pulled in as a git dependency in `pyproject.toml`.
This extension is only the message-hub glue over the SDK:

- `rtlslink.protocol.RtlsProtocol` — the sans-IO protocol core (also
  hosts the PARAM_EXT value codec and the raw-payload extraction that
  works around pymavlink's lossy `char[]` decoding). The extension
  drives it from its own Trio UDP socket.
- `rtlslink.ota.upgrade` — the blocking SMP upgrade helper, run in a
  worker thread.

The SDK also ships the `rtls-link` CLI for debugging devices directly,
without a running server: `discover`, `param list/get/set`, `monitor`,
`ota`, and `selfcheck` (end-to-end protocol check against a live
firmware; `selfcheck.py` in this directory is just a deprecated shim
for it). Pass `--debug` for raw frame tracing; inside the server the
same traces are available by raising the `rtlslink` logger to DEBUG.

## Layout

- `extension.py` — the Trio/flockwave glue over the SDK: discovery
  loop, the awaitable parameter transactions, OTA jobs and the client
  message handlers documented below.
- `show_clock.py` — the cluster->GPS show-clock pin manager (see
  below).
- `geometry.py` — cell-geometry consistency across the tag fleet: the
  `X-RTLS-GEO` check/sync operations (see below).
- `verify.py` — the `X-RTLS-VERIFY` fleet pre-flight rule set (see
  below).
- `fit.py` — the per-responder rolling-TWR cache, distributed calibration
  capture and `X-RTLS-GEO` fit op (see below).
- `anchor_geometry.py` — the pure strict/refined four-tripod geometry
  models behind the fit (no MAVLink, devices or server state).
- `cell_compat.py` — fallback cell-model helpers (role, origin + anchor
  NED table, NED->global) for SDK pins that predate them; the
  `rtlslink` implementations are used when present.
- `selfcheck.py` — deprecated shim forwarding to
  `rtls-link selfcheck`.

## Configuration

```jsonc
"EXTENSIONS": {
  "rtls": {
    "port": 3333,
    "devices": ["192.168.4.1"],        // static addresses (optional)
    "broadcast": ["255.255.255.255"],  // discovery broadcast
    "advertisement_port": 3343,        // state-advertisement listener (0 disables)
    "passive": false,                  // advertisement-driven presence (see below)
    "hello_interval": 60,              // active probe cadence in passive mode, s
    "register_beacons": true,          // anchors as map beacons (see X-RTLS-INF)
    "show_clock_pin": true             // cluster->GPS show-clock pin (see below)
  }
}
```

`heartbeat_interval` and `device_timeout` (seconds) may also be set to
override the presence cadence; the defaults are 2 s / 6 s in active mode
and `hello_interval` / 30 s in passive mode.

## Passive presence (state advertisements)

Firmware with the state-advertisement feature (fw PR
`feature/state-advertisement`) announces itself autonomously: one UDP
datagram to `:3343` after every DHCP bind and then every `ADV_PERIOD_S`
(device parameter, default 10 s), carrying HEARTBEAT (identity + sleep
state), SYSTEM_TIME (uptime) and PARAM_EXT_VALUE frames for
`FW_VERSION` and `UWB_ROLE`. The extension always listens on
`advertisement_port` (default 3343; `0`/`null` disables) and treats
every advertisement as a device heartbeat: the device is discovered /
kept alive at *(source IP, management `port`)* — the datagram itself
leaves a throwaway socket the firmware never reads from — its
version/role/sleep/uptime are refreshed, and the same
gained/lost `X-RTLS-INF` notifications fire as for active discovery.

With `passive: false` (default) nothing else changes: the extension
still probes with GCS heartbeats every `heartbeat_interval` (2 s) and
expires devices after `device_timeout` (6 s); advertisements just
discover boards earlier and enrich `X-RTLS-INF`.

With `passive: true` the mgmt channel stops being hammered — during
shows the 2.4 GHz AP is contended, and management flapping is
operator-visible noise. Active probing slows to one "hello" every
`hello_interval` (default 60 s; still needed so boards learn the server
address for their unicast telemetry, and so legacy boards get probed at
all), and `device_timeout` defaults to 30 s (3× the 10 s advertisement
period) unless configured explicitly.

**Fleet requirements and legacy boards.** `passive: true` assumes the
fleet runs advertisement-capable firmware. Legacy boards (no
advertisement image) are still tracked, with caveats:

- Once a legacy board hears one hello, its management link locks onto
  the server and its firmware streams **1 Hz heartbeats** (plus ~2 Hz
  stats) to it unconditionally — the board keeps *itself* alive; the
  30 s timeout does not flap it in steady state. (Contrary to the
  intuition that a board "answering each 60 s hello" would expire on a
  30 s timeout: boards are not reply-only, they free-run once peered.)
- As extra insurance, in passive mode the extension refreshes liveness
  from **any** inbound datagram on either socket, content-independent:
  real firmware datagrams exist that the SDK protocol core emits no
  event for at all (the `pn`/`pe`/`pd` position stats, SYSTEM_TIME; the
  core itself only counts heartbeats). A board whose heartbeats are
  lost to a contended AP stays alive as long as anything from it gets
  through. Attribution is by the MAVLink header **system id** carried
  in the datagram's frames, guarded by the source IP matching the
  device's known address — never by source address alone: DHCP reuses
  IPs across power cycles, and an address-keyed refresh would keep a
  ghost device alive forever on its successor's traffic. On an IP
  mismatch (the device moved) nothing is refreshed; the normal
  heartbeat path migrates the recorded address. A device must still be
  *discovered* by a heartbeat or advertisement first.
- Residual gaps: after a legacy board reboots (or the server restarts
  on a new source port), the board sends nothing until the next hello,
  so its (re)discovery can take up to `hello_interval` — advertising
  boards re-announce within `ADV_PERIOD_S` instead. A legacy board that
  goes fully silent is only re-probed on the hello cadence, so a true
  outage is detected within `device_timeout` but recovery detection can
  lag by up to `hello_interval`.

If the pinned `rtls-link` SDK predates the advertisement parser
(`rtlslink.advertisement`), the extension logs one warning, disables
the listener and otherwise works exactly as before — `passive: true`
then degrades to just the slow hello + 30 s timeout, which is only
safe with self-heartbeating boards (see above).

## Show-clock GPS pin (UWB-timebase show start)

Tags recover a shared absolute cluster time from the UWB anchor cluster
and report it in their health stats (`clkh`/`clks` cluster seconds, with
a `clkok` freshness flag). On the first fresh sample the extension mints
a *pin* — "at cluster tick C0 the GPS time is (week, tow)" — by pairing
the reported cluster time with the server's own wall clock, and
distributes the **identical** pin to every tag as the `GPS_PIN_WEEK` /
`GPS_PIN_TOW_MS` / `GPS_PIN_C0_HI` / `GPS_PIN_C0_LO` parameters. Each
tag then synthesizes the same GPS time from its own cluster clock and
feeds it to its autopilot as GPS_INPUT, so ArduPilot's native
GPS-synchronized show scheduler self-triggers in lockstep across the
fleet with no go-time packet. Inter-drone agreement rides entirely on
the shared cluster clock; the pin's absolute accuracy (server clock +
stats latency, tens of ms) only offsets the show against wall clock,
equally for every drone.

- Writes follow the firmware's ordering contract: `GPS_PIN_WEEK` is
  zeroed first and written with the real week **last**, so a
  half-applied pin reads as disabled on the tag instead of mixing old
  and new C0 halves.
- Pushes ride the stats feed, so they retry until every write is
  acknowledged; a device that is lost and rediscovered is re-pinned (it
  may have rebooted with default parameters).
- A cluster restart (a tag's reported cluster time deviating from the
  pin's prediction by more than 5 s — e.g. the time-reference anchor
  power-cycled) invalidates every distributed pin: the extension logs a
  warning, mints a fresh pin and redistributes it to the whole fleet.

Set `show_clock_pin: false` in the configuration to disable pin
management entirely.

## Client message API

The extension registers experimental (`X-` prefixed, schema-exempt)
message types on the server's message hub. All messages follow the
usual Flockwave envelope (`{"$fw.version": ..., "id": ..., "body":
{...}}`); only bodies are shown below. Requests that fail validation,
address an unknown device, or time out are answered with a standard
`ACK-NAK` body carrying a human-readable `reason`.

Devices are addressed by their MAVLink **system id** (`id`, an
integer; numeric strings are accepted). Parameter-transaction requests
accept an optional `timeout` (seconds, capped at 60) overriding the
defaults (5 s for single-parameter operations, 10 s for full lists).

Parameter types are the MAV_PARAM_EXT_TYPE names, lower-case:
`uint8`, `int8`, `uint16`, `int16`, `uint32`, `int32`, `uint64`,
`int64`, `real32`, `real64`, `custom`. Numeric values travel as JSON
numbers; `custom` values as UTF-8 strings.

### X-RTLS-INF — list discovered devices

Request:

```json
{"type": "X-RTLS-INF"}
```

Response — one entry per live device, keyed by system id (as string),
plus a site-level `anchors` list:

```json
{
  "type": "X-RTLS-INF",
  "status": {
    "42": {
      "id": 42,
      "address": ["192.168.4.42", 3333],
      "age": 0.52,
      "firmwareVersion": "1.2.3",
      "uptimeMs": 123456,
      "paramCount": 23,
      "otaStatus": null,
      "sleeping": false,
      "uav": "05",
      "role": "tag",
      "name": "RTLS tag 42",
      "twr": [{"peerMac": 1, "distanceM": 14.1, "ageMs": 120}]
    }
  },
  "anchors": [
    {
      "id": "rtls::default::anchor_0",
      "cell": "default",
      "index": 0,
      "mac": 1,
      "position": {"lat": 41.39, "lon": 2.15, "amsl": 10.0},
      "ned": {"north": -10.0, "east": -10.0, "down": 0.0},
      "active": true
    }
  ]
}
```

- `age` — seconds since the last heartbeat from the device (in passive
  mode: since any datagram from it).
- `firmwareVersion` — from the device's latest state advertisement when
  one has been received, else the `FW_VERSION` parameter if the device
  exposes one, otherwise `null`.
- `uptimeMs` — device uptime in milliseconds, as reported by its latest
  state advertisement; absent for devices that never advertised (the
  underlying counter wraps after ~49.7 days, like the firmware's).
- `paramCount` — size of the device's parameter registry once known
  (the list is auto-fetched on discovery), otherwise `null`.
- `otaStatus` — status of the device's last OTA job
  (`"running"` / `"success"` / `"error"`) or `null` if there was none.
- `sleeping` — `true` while the drone is in sleep mode (power rails to
  the motors/flight controller, ELRS receiver and UWB module cut; WiFi
  and this management link still up). Derived from the device's
  heartbeat (`MAV_STATE_STANDBY`), so it is live even though sleep is
  commanded through the `SLEEP` parameter. **Omitted** when the device
  has stayed alive past the device timeout without a heartbeat
  re-latching the state (possible in passive mode, where any traffic
  refreshes liveness) — the latched value is then a guess, and a UI
  should render the absence as "unknown". An `X-RTLS-INF` notification
  is pushed (throttled) whenever a device's `sleeping` flips, and an
  accepted sleep/wake transaction updates the flag optimistically —
  a woken device reads `sleeping: false` immediately, even though it
  reboots off the network for a few seconds before its first `ACTIVE`
  heartbeat. The transaction's outcome is pinned for up to 30 s:
  contradicting in-flight heartbeats (the firmware acks the `SLEEP`
  write before its power task cuts over) are overridden until a
  heartbeat confirms the new state or the pin expires.
- `uav` — the flockwave id of the drone this device is associated with,
  or absent when there is none (anchors, spare tags, tags without a
  WiFi-UART bridge). The association is **derived, never configured**: a
  drone's flight-controller MAVLink reaches the server through its tag's
  WiFi-UART bridge, so a connected UAV whose UDP source IP equals a
  device's management IP is that device's drone. It is recomputed
  continuously (DHCP renewals move it, a disappearing UAV or device
  clears it) and an IP claimed by more than one UAV maps to none —
  better unmapped than mis-attributed. An `X-RTLS-INF` notification is
  pushed (throttled) whenever a device's mapping changes.
- `role` — `"tag"`, `"anchor-initiator"`, `"anchor-responder"` or
  `"disabled"`, from the latest state advertisement or the device's
  `UWB_ROLE` parameter; absent when the device exposes neither.
- `name` — a human-readable role-aware label (e.g. `"RTLS anchor A0"`);
  absent for devices with no recognised role.
- `twr` — inter-anchor TWR telemetry: a list (freshest first) of
  `{peerMac, distanceM, ageMs}` rows, one per peer anchor the device
  currently hears on the UWB ether. The peer MAC is decoded from the
  firmware's `twr<peer-mac-hex>` NAMED_VALUE_FLOAT and `ageMs` is the time
  since that range was last harvested. Present only on anchors that report
  ranges.

The site-level `anchors` list mirrors the configured cell geometry: each
anchor carries a stable id `rtls::<cell>::anchor_<i>`, its GPS position
(cell origin + NED), its native cell-frame NED coordinates in meters
(`ned` — the frame the X-RTLS-POS estimates are expressed in, so a
debug view can plot both without a lossy GPS round-trip), and `active`
— true only when a live anchor device with the matching `UWB_MAC` is
online. These anchors are also published
to clients through the existing Skybrush **beacon** layer (same stable
ids), so the map renders them without a bespoke anchor layer. Set
`register_beacons: false` to disable the beacon registration.

Devices that miss their liveness timeout disappear from the map.

The server also **broadcasts** `X-RTLS-INF` notifications with the same
body shape (no `refs` member, since they are not responses) whenever a
device is discovered or lost or its `sleeping` state changes (from a
heartbeat flip or an accepted sleep/wake transaction) — throttled to at
most one per second, with transitions inside the window coalesced into
one trailing-edge notification — and every 10 s without transitions so
`age` values keep refreshing. Each notification carries the **full** current status map;
clients should replace their device list wholesale rather than merge.

### X-RTLS-PARAM-LIST — full parameter list

Drives a full PARAM_EXT list transaction against the device and
responds once **all** parameters have been received (or NAKs on
timeout).

Request:

```json
{"type": "X-RTLS-PARAM-LIST", "id": 42, "timeout": 10}
```

Response:

```json
{
  "type": "X-RTLS-PARAM-LIST",
  "id": 42,
  "count": 4,
  "params": {
    "MAV_SYS_ID": {"value": 42, "type": "uint8", "index": 0},
    "UWB_CHANNEL": {"value": 5, "type": "int32", "index": 1},
    "POS_X": {"value": 1.5, "type": "real32", "index": 2},
    "FW_VERSION": {"value": "1.2.3", "type": "custom", "index": 3}
  }
}
```

### X-RTLS-PARAM-GET — read one parameter

Issues a PARAM_EXT read and responds with the value reported by the
device (NAK on timeout).

Request:

```json
{"type": "X-RTLS-PARAM-GET", "id": 42, "name": "UWB_CHANNEL"}
```

Response:

```json
{
  "type": "X-RTLS-PARAM-GET",
  "id": 42,
  "name": "UWB_CHANNEL",
  "value": 5,
  "paramType": "int32"
}
```

### X-RTLS-PARAM-SET — write one parameter

Issues a PARAM_EXT set and waits for the device-side acknowledgement;
the response carries the value **as acknowledged by the device** and
the raw PARAM_ACK result code (`0` accepted, `1` value unsupported,
`2` failed; in-progress acks are awaited transparently). A device-side
rejection is still a normal response (`accepted: false`); only
timeouts and malformed requests are NAKed.

Since the firmware's tag/anchor application split, an out-of-bounds
numeric set is acked `1` (value unsupported) with the **clamped value
the device actually applied** in the response — not a silent success.
The motivating case is `UWB_ROLE`, whose bounds are image-pinned
(tag `1..1`, anchor `2..3`): a device can no longer change species by
parameter; flash/OTA the role-matched image instead. UIs should show
`accepted: false` plus the returned value as "device rewrote this".

Request — `paramType` may be omitted when the type is already known
from an earlier listing (always the case after discovery):

```json
{
  "type": "X-RTLS-PARAM-SET",
  "id": 42,
  "name": "UWB_CHANNEL",
  "value": 9,
  "paramType": "int32"
}
```

Response:

```json
{
  "type": "X-RTLS-PARAM-SET",
  "id": 42,
  "name": "UWB_CHANNEL",
  "value": 9,
  "paramType": "int32",
  "result": 0,
  "accepted": true
}
```

### X-RTLS-SLEEP — sleep / wake drones

Puts one or more devices into sleep mode, or wakes them. Sleep is
commanded through the firmware's `SLEEP` parameter; the firmware
**refuses** to sleep while its flight controller is armed (or, in the
default strict gate mode, while a live disarmed heartbeat cannot be
confirmed) by flipping the parameter back to `0` — the server detects
that by reading the parameter back after a settle delay and reports
the refusal per device. A sleeping drone keeps WiFi and the
management channel up, stays discoverable (`sleeping: true` in
`X-RTLS-INF`) and can be woken with the same message.

On hardware, a woken device reboots a moment after acknowledging the
wake (to re-initialize the power-cycled UWB module) and drops off the
network for a few seconds before re-appearing.

Request — `ids` lists the target devices (a single `id` is also
accepted); `sleeping` selects the direction:

```json
{
  "type": "X-RTLS-SLEEP",
  "ids": [42, 43],
  "sleeping": true
}
```

Response — one entry per device; `accepted: false` with a `detail`
of the refusal is a normal response (only malformed requests NAK):

```json
{
  "type": "X-RTLS-SLEEP",
  "sleeping": true,
  "result": {
    "42": {"requested": true, "accepted": true, "sleeping": true, "detail": "asleep"},
    "43": {"requested": true, "accepted": false, "sleeping": false, "detail": "refused by device (arming gate: vehicle armed or flight controller not confirmed disarmed)"}
  }
}
```

### X-RTLS-OTA — firmware update

With an `image` field the message **starts** an OTA job for the device
(NAK if the device is unknown, the file does not exist, or a job is
already running for that device); without it, it **queries** the
status of the device's last job. One job per device at a time; the
image path is a server-side filesystem path.

The firmware ships as **two role-matched artifacts** since the
tag/anchor application split (`build/<board>/tag/...` and
`build/<board>/anchor/...`); a device's role is image-pinned and OTA
must upload the matching one. The optional `role` field declares which
species the artifact was built for — the server then reads the
device's `UWB_ROLE` and NAKs the upload on a mismatch instead of
letting a wrong-species image strip the device of its function:

Start:

```json
{
  "type": "X-RTLS-OTA",
  "id": 42,
  "image": "/srv/firmware/rtls-link-anchor-1.4.0.bin",
  "role": "anchor"
}
```

Without `role` the upload is unguarded (pre-split behavior).

Response (job snapshot; also the shape of the status query response,
whose `job` is `null` when the device never had a job):

```json
{
  "type": "X-RTLS-OTA",
  "id": 42,
  "job": {
    "id": 42,
    "image": "/srv/firmware/rtls-link-1.3.0.bin",
    "status": "running",
    "progress": 0.0,
    "version": null,
    "error": null
  }
}
```

- `status` — `"running"`, then `"success"` or `"error"`.
- `progress` — upload progress, `0.0 … 1.0`.
- `version` — version string of the uploaded image on success.
- `error` — failure description when `status` is `"error"`.

While a job runs, the server **broadcasts** `X-RTLS-OTA` notifications
with the same body shape (no `refs` member, since they are not
responses) on progress changes (at most every 0.5 s) and once on
completion. UIs may rely on the notifications or poll with the status
query.

### X-RTLS-STATS — health telemetry

The firmware streams health stats unsolicited (one `NAMED_VALUE_FLOAT`
per stat, ~2 Hz); the server caches the latest snapshot per device and
**broadcasts** `X-RTLS-STATS` notifications, throttled to at most one
per device per second (with a trailing-edge flush, so the newest
snapshot inside a throttle window is never lost). A notification carries
the entry of the device that updated; the query returns every known
device, or one with the optional `id` (an unknown id yields an empty
snapshot, not an error). Snapshots of devices that drop off the network
are pruned with them.

Request:

```json
{"type": "X-RTLS-STATS"}
```

Response / notification body — entries keyed by system id (as string):

```json
{
  "type": "X-RTLS-STATS",
  "stats": {
    "42": {
      "id": 42,
      "solveRateHz": 12.5,
      "solvePct": 98.0,
      "anchorsSeen": 4,
      "fixAgeMs": 80,
      "clockPpm": 1.2,
      "anchorMask": 15,
      "sleeping": false,
      "batteryVoltage": 7.812
    }
  }
}
```

- `sleeping` — mirrors the `slp` stat; **omitted** (not `false`) for
  firmware that predates sleep mode, so a UI can tell "unknown".
- `batteryVoltage` — the optional `vbat` stat, in volts; omitted on
  boards that cannot measure it.
- The first broadcast for a device waits until the full legacy stat set
  (`solveRateHz` … `anchorMask`) has arrived once, so it never carries a
  half-populated snapshot; optional stats never gate the broadcast.

### X-RTLS-POS — live position-estimate debug stream

Surfaces the tag firmware's position-estimate debug emit
([rtls-link-zephyr#14](https://github.com/Axiovel-tech/rtls-link-zephyr/issues/14)):
when a tag's `POS_DBG_HZ` parameter is nonzero (off by default; set it
with `X-RTLS-PARAM-SET`), every solved NED position streams to the
server as `NAMED_VALUE_FLOAT pn`/`pe`/`pd`/`psig`, and the server
**broadcasts** `X-RTLS-POS` notifications with the latest estimate per
device, throttled to at most one per device per 100 ms. Intended for
the "Debug Pos Estimates" pre-flight view in control; anchors for the
same plot come from the `ned` member of the `X-RTLS-INF` anchor list.

The same body shape is available as a **query** (optional `id` narrows
it to one device; unknown ids yield an empty snapshot, not an error):

```json
{"type": "X-RTLS-POS"}
```

Response / notification body — one entry per device that has reported
a complete estimate, keyed by system id (as string):

```json
{
  "type": "X-RTLS-POS",
  "positions": {
    "42": {
      "id": 42,
      "north": 1.204,
      "east": -0.351,
      "down": -0.82,
      "sigma": 0.12,
      "timeBootMs": 123456,
      "ageMs": 3
    }
  }
}
```

- `north` / `east` / `down` — the solved position in the cell's NED
  frame, meters.
- `sigma` — the firmware's reported NE sigma (meters); absent when the
  solver had no covariance for this estimate.
- `timeBootMs` — the device-side timestamp of the estimate (the stamp
  that groups one emit cycle on the wire).
- `ageMs` — server-side age of the estimate at send time; with the
  stream on, notifications carry ~0, while a query answered from cache
  can be older. Clients should fade/flag entries whose updates stop.

Estimates of a device that drops off the network are pruned with it.

### X-RTLS-GEO — canonical cell geometry: adopt / check / sync

Every drone's tag carries its own copy of the cell geometry
(`ORIGIN_LAT_E7/LON_E7/ALT_MM`, `POS_YAW_DEG`, `CELL_ID`,
`UWB_AN_COUNT` and the `UWB_AN{i}_X/Y/Z/MAC/BIAS_M` anchor table); tags
that disagree position their drones in different frames. This message
answers the daily pre-flight question "do my drones agree?" and repairs
the ones that do not.

`op` selects the operation:

- **`check`** diffs the geometry of every live tag (or the tags in
  `ids`) against a *reference* tag and reports per-device verdicts;
- **`sync`** writes the reference geometry to the target tags (verified
  per-parameter acks; one device's failure never affects another), then
  **reboots** each fully rewritten tag over MCUmgr/SMP — the same
  management surface OTA uses — so the new geometry takes effect (the
  firmware reads the anchor table at startup). Pass `"reboot": false`
  to skip the reset. A device whose writes partially failed is reported
  `partial` and deliberately NOT rebooted (that would activate a mixed
  geometry — re-run the sync). Writes order the anchor table first and
  `UWB_AN_COUNT` last, so a half-synced registry never declares a
  window onto a half-written table.

THE SERVER OWNS THE TRUTH: each cell's canonical geometry is a
persisted document (`geometry.json` in the extension's data dir).
Bootstrap it once with **`op: "adopt"`** — with a `reference` system id
the named tag's geometry is taken verbatim; without one the fleet must
be unanimous, so a drifted tag can never be adopted by accident. From
then on `check` diffs EVERY live tag against the canonical geometry and
`sync` distributes it; a calibration fit updates it through the sync
op's explicit `geometry` payload. Pass `cell` to pick among multiple
stored cells. Optional
members: `ids` (target system ids; default = every other live tag),
`tolerance` (float comparison tolerance in the parameter's own unit,
default `1e-4`), `timeout` (per parameter transaction, as usual). Both
operations compare against the server's parameter cache (kept fresh by
discovery, the refill poller and the server's own writes); `sync`'s
device-side acks re-verify reality where it matters.

Request:

```json
{"type": "X-RTLS-GEO", "op": "check"}
```

Response — one entry per target (the reference is never a target),
keyed by system id; `status` is `consistent`, `mismatch` (with
`deltas`), `incomplete` (with `missing`) or `error` (with `detail`):

```json
{
  "type": "X-RTLS-GEO",
  "op": "check",
  "reference": 42,
  "cell": "default",
  "consistent": false,
  "devices": {
    "43": {
      "status": "mismatch",
      "deltas": {"UWB_AN1_X": {"expected": 10.0, "actual": 10.5}}
    },
    "44": {"status": "consistent"}
  }
}
```

Sync request / response:

```json
{"type": "X-RTLS-GEO", "op": "sync", "reference": 42, "reboot": true}
```

```json
{
  "type": "X-RTLS-GEO",
  "op": "sync",
  "reference": 42,
  "cell": "default",
  "devices": {
    "43": {
      "status": "synced",
      "written": ["UWB_AN1_X"],
      "skipped": ["ORIGIN_LAT_E7", "..."],
      "failures": {},
      "rebooted": true
    }
  }
}
```

- `status` — `synced` (all needed writes accepted), `partial` (some
  writes failed; see `failures`) or `error` (device stopped answering;
  see `detail`).
- `written` / `skipped` — parameters written vs. already consistent.
- `failures` — parameter name → human-readable failure (device-side
  rejections carry the ack code and the value the device applied).
- `rebooted` — present only when `reboot` was requested; `false` comes
  with a `rebootDetail` explaining why (writes failed, nothing to
  write, smpclient missing, or the reset itself failed).

A sync that changed anything also pushes an `X-RTLS-INF` notification,
so clients re-render the site anchors immediately. Requests that
resolve no complete reference (no live tag with a full cell, unknown
`reference`/`cell`) are NAKed.

Concurrency: only one sync runs at a time (a second request is NAKed),
parameter writes are serialized per (device, parameter) with a
late-ack drain, and a post-write verification re-diff gates the
reboot. Two windows remain that the PARAM_EXT wire protocol cannot
close (acks carry no transaction id, and verify-then-reboot cannot be
atomic): an ack straggling in more than `timeout` + 1 s late may be
attributed to a subsequent write of the same parameter, and a
parameter written by a third party in the instant between verification
and reset is only caught by the next `check`. A device reported
`partial`/`error` may hold mixed persistent geometry until a re-run
converges it (its cells are re-homed away from it right after the
sync, but it stays eligible again later). None of these windows is
silent: the next `check` reports the fleet inconsistent. UIs should
therefore re-run `check` after every sync, and re-run `sync` until
every device reports `synced`.

### X-RTLS-GEO fit — measure the anchors' true geometry

Tripods go up in roughly the surveyed spots; "roughly" is centimeters
of error the UWB solver bakes into every position. The standing
geometry is measured from the ranging the responders do anyway — the
server only selects a fresh, bounded-skew set of rolling windows.

**Responder rolling summaries (rtls-link summary protocol v1).** On the
SR250, each DL-TDoA responder measures its range to the A0 initiator;
A0 receives no corresponding per-responder measurements. Each A1–A7
responder therefore keeps its own 2 s rolling window and publishes a
robust aggregate at 1 Hz on the management channel, as **one bundled
datagram** of NAMED_VALUE_FLOAT frames that all carry that device
generation's `time_boot_ms`. A0 and sources that cannot summarize
publish nothing.

| field | meaning |
| --- | --- |
| `trcap` | summary protocol version (currently 1) |
| `trseq` | generation sequence, 24-bit wrap-around |
| `trmask` | one bit per valid peer slot (a responder normally reports only bit 0 for A0) |
| `twrXXXX` | filtered range to peer MAC `XXXX`, meters — median of the window's inliers (median ± 3×MAD gate) |
| `twmXXXX` | the window's MAD, meters |
| `twnXXXX` | inlier sample count (a peer needs ≥ 20 to be published at all) |

`XXXX` is the peer MAC as exactly four lowercase hex digits. The
`rtlslink` SDK reassembles coherent generations — the three header
fields plus a complete range/MAD/count triple for every masked slot,
all stamped with the same `time_boot_ms` — and emits one `twr_summary`
event per device generation; frames from different generations are never
combined. The extension caches the newest coherent summary per system
id.

**Fitting.** `{"op": "fit", "mode": "strict"}` fits the newest
measurement:

- resolves the canonical cell geometry (the single stored cell; the
  four-tripod fit requires exactly 8 configured anchors) and maps every
  configured MAC to one unique online device with the expected role;
- waits up to `timeout` seconds (default and cap 4) for one responder
  summary per A1–A7 that is **fresher than the request** and whose
  server receipt timestamps are within 1.5 s;
- validates every source independently: protocol version 1, exactly one
  A0 peer range (`trmask=0x01`), and at least 20 samples — violations
  NAK naming the offending responder;
- combines the seven spokes into a server-owned calibration capture,
  runs the strict model, and **pins** that exact capture + result.

`{"op": "fit", "mode": "refined", "captureId": 4711}` re-fits exactly
the pinned capture (`captureId` must match; anything else, or refining
before any strict fit, NAKs). The refined pass never consumes new
telemetry, so the strict and refined verdicts always describe the same
seven device generations. Firmware `trseq` and `time_boot_ms` remain
per-device provenance and are never compared across device clocks.

Both models place the anchors in canonical A0-origin NED coordinates:
A0 at the origin, A0→A1 along +X, `POS_YAW_DEG` 0; slots A0–A3 are the
lower plane, A4–A7 the upper one (upper anchors get negative `zM`).

- `strict` — two congruent, perpendicular rectangles; parameters
  `lengthM`, `widthM`, `heightM`.
- `refined` — aligned upper/lower parallelograms sharing one corner
  angle; parameters `bottomLengthM`, `bottomWidthM`, `topLengthM`,
  `topWidthM`, `heightM`, `angleDeg`. Hard safety bounds: the angle
  within ±5° of 90°, upper−lower length/width differences within
  max(0.25 m, 2 % of the lower dimension).

A model whose worst spoke residual exceeds 0.15 m is rejected
(`accepted: false`, human-readable `reasons`). `refined` is
additionally accepted only when its RMS improvement over `strict`
exceeds the measurement noise floor (median of the spokes' MAD, at
least 1 cm) AND no parameter reached a safety bound — a bound-riding
fit means the installation deviates more than the refined model
permits, and is rejected instead of hidden.

Explicit NON-GOALS: per-anchor move suggestions, free per-anchor XYZ
fitting, full pairwise-mesh capture and independent per-plane skew.
Seven A0-star radii cannot identify any of them.

Response — `refined` is `null` on a strict-only run; `comparison`
(refined runs only) restates the acceptance arithmetic
(`rmsImprovementM`, `noiseFloorM`, `meaningfulImprovement`);
`selectedModel` names the requested model when it was accepted, else
`null`; `applyGeometry` is a ready-to-sync geometry payload (origin,
`CELL_ID`, MACs and biases copied from the canonical geometry,
`POS_YAW_DEG` forced to 0, fitted `xM`/`yM`/`zM` as the anchor table)
or `null` when the selected model was rejected:

```json
{
  "type": "X-RTLS-GEO",
  "op": "fit",
  "mode": "strict",
  "cell": "default",
  "summary": {
    "systemId": 70,
    "version": 1,
    "sequence": 4711,
    "timeBootMs": 123456,
    "validMask": 254,
    "ageMs": 180,
    "ranges": [
      {"anchorIndex": 1, "peerMac": 2, "distanceM": 20.001, "madM": 0.012, "count": 57},
      "..."
    ]
  },
  "strict": {
    "model": "strict",
    "accepted": true,
    "parameters": {"lengthM": 20.003, "widthM": 15.001, "heightM": 2.499},
    "anchors": [{"index": 0, "xM": 0.0, "yM": 0.0, "zM": 0.0}, "..."],
    "rmsM": 0.011,
    "weightedObjective": 0.000123,
    "residuals": [
      {"anchorIndex": 1, "peerMac": 2, "measuredM": 20.001, "predictedM": 20.003,
       "residualM": 0.002, "madM": 0.012, "count": 57, "weight": 1.0},
      "..."
    ],
    "reasons": [],
    "warnings": []
  },
  "refined": null,
  "selectedModel": "strict",
  "applyGeometry": {"ORIGIN_LAT_E7": 413900000, "POS_YAW_DEG": 0.0, "...": "..."}
}
```

To APPLY a fit result, pass its `applyGeometry` to the sync op as an
explicit payload — it is validated and written to EVERY tag (the
former reference included), with the same verified-write/reboot
semantics:

```json
{"type": "X-RTLS-GEO", "op": "sync", "geometry": {"ORIGIN_LAT_E7": 413900000, "...": "..."}, "reboot": true}
```

Firmware/SDK dependency: the fit needs A0 firmware that speaks the
rolling-summary protocol (v1) and an `rtls-link` SDK new enough to
emit the `twr_summary` event; without them a strict fit NAKs after the
wait with "no rolling TWR summary arrived from A0".

### X-RTLS-VERIFY — fleet pre-flight verification

Runs the whole "are my drones consistent?" rule set in one message:
**geometry** (the X-RTLS-GEO check: origin, `POS_YAW_DEG`, `CELL_ID`,
anchor table, majority reference), **firmware** uniformity per role,
tag↔drone **pairing** coverage, the ArduPilot **yaw-source** rule
(`EK3_SRC1_YAW == 9`, the axio fork's virtual compass, and a
fleet-consistent `EK3_SRC_VC_YAW`, read live over MAVLink with
per-drone error isolation; the tags' `POS_YAW_DEG` is reported
alongside), and the **uwb** solving state (rate, fix age, solve
quality, cluster-clock sync from the live stats). With
`"inDepth": true` a sixth rule reads the ArduPilot navigation/tuning
set (EKF sources, VISO, WPNAV, LOIT, position/attitude controllers,
IMU filters) from every paired drone and reports cross-drone
differences — always as warnings: deliberate per-drone tuning exists.

```json
{"type": "X-RTLS-VERIFY", "inDepth": false}
```

Response: `rules` (each with `id`, `label`, `severity`
(`error`/`warning`), `status` (`pass`/`fail`/`skipped`), a
human-readable `detail` and rule-specific extras), `passed` (no
error-severity rule failed) and the embedded `geometry` check body for
UI reuse. Concurrent runs are NAKed; expect the in-depth pass to take
a few seconds per fleet (live MAVLink parameter reads).

### Notes for control-UI developers

- The server treats `X-`-prefixed messages as experimental: they skip
  JSON schema validation, so unknown body members are ignored rather
  than rejected — keep client payloads tidy.
- `X-RTLS-PARAM-LIST` / `-GET` / `-SET` block server-side until the
  device transaction completes; expect responses to arrive up to
  `timeout` seconds after the request. Responses reference the request
  message id in the `refs` member of their envelope, as usual.
- Parameter writes that the device accepts are persisted on the device
  by the firmware's parameter registry.

## Extension exports

Other server extensions can use the same machinery via the exports of
this extension: `devices()`, `protocol()`, and the awaitable
`get_param(system_id, name)`, `get_param_list(system_id)`,
`set_param(system_id, name, value, param_type=None)`,
`set_sleep(system_id, sleeping)`,
`verify_species(system_id, role)` and
`start_ota(system_id, image_path)`.

## Testing

`test/test_ext_rtls.py` exercises the message handlers against the
SDK's sans-IO protocol core with a fake transport and a scripted fake
device (no sockets):

```sh
uv run pytest test/test_ext_rtls.py
```

Protocol-level tests (codec roundtrips, heartbeat/param state machine,
client, CLI) live in the SDK's own test suite in the firmware repo
(`py/tests/`), not here.
