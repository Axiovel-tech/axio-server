# rtls — rtls-link device management

Manages rtls-link UWB anchors and tags over their MAVLink management link
and exposes them to Control as the `X-RTLS-*` messages: discovery and
presence, parameters, sleep and wake, OTA, health and position telemetry,
anchors as map beacons, the fleet geometry check and the pre-flight
verification. It also owns the server's estimate of the UWB cluster clock,
which the `mavlink` extension needs to send a show start in cluster time.

What other repos rely on lives in [AxioStack `contracts/`](https://github.com/Axiovel-tech/AxioStack/tree/main/contracts):

| Topic | Contract |
|---|---|
| the `X-RTLS-*` messages Control and axio-e2e use | [`rtls-client-api.md`](https://github.com/Axiovel-tech/AxioStack/blob/main/contracts/rtls-client-api.md) |
| the tag's geometry stats and what each failure code means | [`rtls-geometry-stats.md`](https://github.com/Axiovel-tech/AxioStack/blob/main/contracts/rtls-geometry-stats.md) |
| the pre-show health metrics and their zones | [`rtls-fleet-health.md`](https://github.com/Axiovel-tech/AxioStack/blob/main/contracts/rtls-fleet-health.md) |
| the cluster clock and the show start | [`show-timing.md`](https://github.com/Axiovel-tech/AxioStack/blob/main/contracts/show-timing.md) |

## Firmware it speaks to

Only rtls-link-zephyr boards: management heartbeats on component id 197,
stats as `NAMED_VALUE_FLOAT`, `PARAM_EXT` parameters and MCUmgr/SMP OTA.
Boards with the legacy ESP32/Arduino rtls-link firmware use another dialect
and are invisible here by design.

The protocol and OTA code is the `rtls-link` Python SDK (module `rtlslink`)
from rtls-link-zephyr `py/`, pinned by commit in `pyproject.toml`. This
extension is the Trio and message-hub glue over it. A firmware change that
adds stats reaches the server only after that pin moves.

The SDK's `rtls-link` CLI talks to boards without a server (`discover`,
`param`, `monitor`, `ota`, `selfcheck`); `--debug` traces raw frames.
Inside the server, raise the `rtlslink` logger to DEBUG for the same traces.

## How boards are found

Every board streams its management telemetry to `MGMT_HOST:MGMT_PORT` from
boot. The extension listens on `listen_port` (the firmware's `MGMT_PORT`
default, 3343) and finds a board from its first heartbeat, at whatever
address it has; it then sends it GCS heartbeats and fetches its parameter
list. The extension does not start when `listen_port` is taken.

The optional `devices` list, heartbeated on `port` from startup, is only for
boards that do not stream here: a wrong `MGMT_HOST`, or a stream that another
client (the CLI, a second laptop) took while the server was silent; a board
keeps streaming to that client until someone else speaks to it.

## Configuration

```jsonc
"rtls": {
  "enabled": true,
  "listen_port": 3343,        // the boards' MGMT_PORT
  "devices": [],              // optional, see above
  "port": 3333,               // management port of the boards in "devices"
  "register_beacons": true,   // anchors as map beacons
  "heartbeat_interval": 2,    // s
  "device_timeout": 6,        // s without a heartbeat before a board is lost
  "lock_required_for_show": true  // SHOW-SETCFG authorizes a flight only
                                  // with anchors online, all locked
}
```

The extension is not loaded by default; `etc/conf/axio-rtls-indoor.jsonc`
enables it.

## Things the code does not make obvious

- **The server never writes anchor tables.** Every tag fits the cell itself
  at boot. `X-RTLS-GEOM` only compares the fits across the fleet, and a tag
  refits on `UWB_GEOM_RECAL=1`.
- **OTA has no bootloader revert.** MCUboot on the ESP32-S3 is
  overwrite-only, and the SDK confirms every uploaded image, so a bad image
  is recovered by uploading the previous one. Tag and anchor images are
  different artifacts: the server takes the image's species from the
  `.receipt.json` that rtls-link-zephyr `scripts/build.sh` writes beside it
  (the declared `role` only without one) and checks it against `UWB_ROLE`;
  `UWB_ROLE` itself cannot be changed by parameter.
- **Sleep is a parameter write the firmware may undo.** On a tag, `SLEEP` is
  written back to `0` while the drone's autopilot is armed or not confirmed
  disarmed; the server reads it back after a settle delay to report the
  refusal. An anchor reboots when it enters or leaves operating (8-15 s off
  the network) and falls asleep in place from standby; `anchor_sleep.py`
  verifies the outcome from the `powerState` it reports after the write and
  does not command an anchor that reports none. A woken board reboots.
- **The anchor lock lives in the firmware.** A locked anchor (`LOCK=1`,
  reported as its `lock` stat) refuses every remote change itself. The
  server only writes `LOCK`, derives the cell's lock from the stats, and
  answers the show extension (`show_start_refusal()`) before it authorizes a
  flight.
- **Cell settings are derived, never stored.** Each board reports the code
  of the session it booted with (`cellcfg`, anchors also `txgain`); the
  server compares it with the code of its cached parameters and with what A0
  runs, and `X-RTLS-CELL` writes the ranging rate and the anchors' TX level,
  then reboots the running boards the state after the writes calls for (A0
  last). One apply at a time; one server is one cell.
- **Lost boards stay listed.** A board that misses `device_timeout` stays in
  `X-RTLS-INF` offline, with its last stats marked stale, until an operator
  forgets it or the server restarts; nothing treats it as live.
- **Drone pairing comes from the tag.** A tag reports its autopilot's system
  id (`FC_SYS_ID` / `FC_STATE`) and keeps reporting it while asleep; the
  server pairs the tag with the drone of that system id.
- **The IMU temperatures in `X-RTLS-STATS` are not tag stats.** They are the
  paired drone's own `SCALED_IMU*` messages, joined in by the server.

## Code

- `extension.py`: discovery, parameter transactions, sleep, OTA, stats and
  the message handlers. Its exports (`devices()`, `get_param()`,
  `set_param()`, `set_sleep()`, `start_ota()`, `cluster_clock`, …) are the
  entry points for other extensions.
- `anchor_sleep.py`: an anchor's sleep and wake and their verification.
- `anchor_lock.py`: `X-RTLS-LOCK`, the lock state derived from the stats
  and the show gate.
- `cell_settings.py`: `X-RTLS-CELL`, the per-device cell-settings state, its
  lock warning and Verify rule.
- `cluster_clock.py`: the wall-clock estimate of the cluster clock.
- `geometry_agreement.py`: `X-RTLS-GEOM`, a pure function over the cached
  stats.
- `verify.py`: `X-RTLS-VERIFY`.
- `fc_identity.py`: the tag-to-drone pairing.
- `fitted_cell.py`, `cell_compat.py`: the anchor cell built from a tag's
  fit, and fallbacks for SDK pins that predate the cell helpers.
- `selfcheck.py`: deprecated shim for `rtls-link selfcheck`.

## Tests

```sh
uv run pytest test/test_ext_rtls.py test/test_rtls_*.py
```

The tests drive the handlers against the SDK's sans-IO protocol core with a
scripted fake device, without sockets. Protocol tests belong to the SDK, in
rtls-link-zephyr `py/tests/`. `test/rtls_support.py` and
`test/anchor_sleep_support.py` and `test/anchor_lock_support.py` hold the
fakes; `test/test_ext_show_anchor_gate.py` covers the show gate.
