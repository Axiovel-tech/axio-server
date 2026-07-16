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
- **OTA**: MCUmgr/SMP upload → mark pending → reset, via
  `rtlslink.ota` / `smpclient` (asyncio; run in a worker thread from
  Trio). On the ESP32-S3 MCUboot is overwrite-only — no bootloader
  revert — so the recovery path is health-check + re-upload of the
  previous artifact. `smpclient` is an optional dependency of the SDK
  (`rtls-link[ota]`); without it, starting an OTA job fails at runtime.

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
    "hello_interval": 60               // active probe cadence in passive mode, s
  }
}
```

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
  from **any** inbound datagram whose source address matches a known
  device, on either socket — attribution is by address, not by decoded
  content, because real firmware datagrams exist that the SDK protocol
  core emits no event for at all (the `pn`/`pe`/`pd` position stats,
  SYSTEM_TIME; the core itself only counts heartbeats). A board whose
  heartbeats are lost to a contended AP stays alive as long as anything
  from it gets through. A device must still be *discovered* by a
  heartbeat or advertisement first — unattributable datagrams from
  unknown addresses refresh nothing.
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
  commanded through the `SLEEP` parameter.
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
device is discovered or lost — throttled to at most one per second,
with transitions inside the window coalesced into one trailing-edge
notification — and every 10 s without transitions so `age` values keep
refreshing. Each notification carries the **full** current status map;
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
`set_sleep(system_id, sleeping)` and
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
