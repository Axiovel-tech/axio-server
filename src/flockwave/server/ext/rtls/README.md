# rtls — rtls-link device management

Manages rtls-link UWB anchors and tags over their MAVLink management link
and exposes them to Control as the `X-RTLS-*` messages: discovery and
presence, parameters, sleep and wake, OTA, health and position telemetry,
anchors as map beacons, the anchor survey and the pre-flight
verification. It also owns the server's estimate of the UWB cluster clock,
which the `mavlink` extension needs to send a show start in cluster time.

What other repos rely on lives in [AxioStack `contracts/`](https://github.com/Axiovel-tech/AxioStack/tree/main/contracts):

| Topic | Contract |
|---|---|
| the `X-RTLS-*` messages Control and axio-e2e use | [`rtls-client-api.md`](https://github.com/Axiovel-tech/AxioStack/blob/main/contracts/rtls-client-api.md) |
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

- **The anchors own the geometry.** Each anchor stores its position and a
  survey ID (`UWB_LOC`) and advertises them; a tag holds no anchor list and
  positions on the anchors of one survey ID that it hears. So the server
  writes no table and no bias to a tag and reads none from it. An anchor's
  position in `X-RTLS-INF` is the one that anchor stores (its `UWB_LOC` is
  read again when the survey ID it advertises is not the cached one), and
  only its place on the map needs a tag: the origin of the lowest online
  tag of its cell.
- **The survey the cell is on is derived, never stored.** An anchor is on
  the survey ID its session advertises (`sid`), or on the one it stores
  while it runs no session; the cell is on the ID most online anchors are
  on, the newer of two held equally. One server is one cell here too. A tag
  is positioned when it follows that ID and uses four anchors, or three when
  every anchor that advertises it is `planar` (`survey_view.positioned`),
  and it is expected to use the anchors that advertise the ID now: what a
  sleeping anchor stores is that anchor's own finding.
- **A tag's anchor table arrives one anchor per report.** `antbl` rotates
  over the anchors the tag tracks; the server assembles `anchorTable` and
  drops an anchor not named for two rotations. A rotation is counted in
  stats events of the tag, of which its 10 s status report is one more.
  Bit *i* of a tag's anchor masks is row *i* of that table.
- **OTA has no bootloader revert.** MCUboot on the ESP32-S3 is
  overwrite-only, and the SDK confirms every uploaded image, so a bad image
  is recovered by uploading the previous one. Tag and anchor images are
  different artifacts: the server takes the image's species from the
  `.receipt.json` that rtls-link-zephyr `scripts/build.sh` writes beside it
  (the declared `role` only without one) and checks it against `UWB_ROLE`;
  `UWB_ROLE` itself cannot be changed by parameter.
- **Sleep is a parameter write.** On a tag the firmware may undo it: `SLEEP`
  is written back to `0` while the drone's autopilot is armed or not
  confirmed disarmed, and the server reads it back after a settle delay to
  report the refusal. An anchor answers in the write's acknowledgement
  (`PARAM_ACK_FAILED` for a request it would not act on now); the server
  returns that answer without waiting, and the anchor's `powerState` shows
  the transition and whether it is asleep. A woken board reboots.
- **The anchor lock lives in the firmware.** A locked anchor (`LOCK=1`,
  reported as its `lock` stat) refuses every remote change itself. The
  server only writes `LOCK`, derives the cell's lock from the stats, and
  answers the show extension (`show_start_refusal()`) before it authorizes a
  flight.
- **Cell settings are derived, never stored.** Each board reports the code
  of the session it runs (`cellcfg`, anchors also `txgain`) and the server
  compares it with what A0 runs. A write applies on the board at once, so
  `X-RTLS-CELL` only writes: the tags first, the anchors only when every tag
  took it. One apply at a time; one server is one cell.
- **A survey gives every anchor its own position.** `X-RTLS-SURVEY` makes
  each anchor of a group the ranging initiator in turn (`UWB_SURVEY`, held
  in the anchors' RAM: an anchor returns to its stored session by itself
  when the server goes away), fits the positions to the ranges, and on the
  operator's commit writes each anchor its `UWB_LOC`: position, survey ID
  and its own measured range to the stored initiator. The anchors are the
  only store: the survey ID continues from what they hold (a commit is
  refused while an online anchor's stored value cannot be read), a missed
  write is not rolled back, and the commit does not lock (Control locks
  after it). The same commit again brings in an anchor that missed it,
  unless an anchor of the group was lost or restarted since the measurement, or
  something else was written to the group. A start and a commit are refused
  while a drone reports itself armed; an unknown arming state does not
  refuse, and a drone armed during a run does not stop it. While a survey
  runs, the cell
  settings, the lock, and sleep and OTA of its anchors are refused, and its
  anchors keep the session they last reported through the restarts of their
  ranging. An online anchor of the group's cell must be in the group unless
  it reports another power state than operating: left out while it ranges,
  it would lose its initiator at every step and keep the old survey ID. A
  start that leaves one out is refused, and a start that names no group
  takes exactly those anchors.
- **A commit names the survey it commits.** The survey state carries a
  `revision`, in every reply and notification, that the server raises on
  every change of what it holds (a phase, a measurement, a fit, a commit's
  result, a cancel) and never lowers: a new instance of the extension starts
  at the wall clock in milliseconds. `commit` must carry the
  current one and `fit` may; a request with another is refused with the
  reason `stale` and writes nothing, so a client cannot commit a fit another
  client replaced while it was looking at the old one.
- **A height group is "about the same height".** With one height group in
  the frame its anchors share one height exactly: ranges within one plane
  cannot tell heights apart, and the survey is `planar`. With two or more,
  every anchor gets its own height, held within 0.05 m (one sigma,
  `HEIGHT_TOLERANCE_M`) of its group's common height, and
  `fit.anchors[id].heightOffsetM` says how far it stands above it (null for
  an anchor in no group, 0 in the exact case). A group of one is no group:
  the server drops it when it reads the frame, that anchor's height is free,
  and the frame echoed in the state is the frame as used. z = 0 is the
  common height of the origin's group. Ranges do not know which way is up
  either: "level" is the tilt of the cell that leaves the least offsets, so
  one anchor 4 cm off a four-anchor tier reads about +2 cm and its
  neighbours the rest. The pulls are not part of `rmsM` or of any pair's
  residual.
- **Firmware lane stalls are in the server log only.** A board sends a stall
  of 250 ms or more once, after it ended; the `logging` extension's
  `skybrushd.log` keeps the line with its time. The cause is in the board's
  `lanes` shell command until it reboots.
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
- `anchor_sleep.py`: an anchor's sleep and wake, answered by the ack.
- `anchor_lock.py`: `X-RTLS-LOCK`, the lock state derived from the stats
  and the show gate.
- `cell_settings.py`: `X-RTLS-CELL`, the per-device cell-settings state,
  the `cellSettings` summary, its lock warning and Verify rule.
- `cluster_clock.py`: the wall-clock estimate of the cluster clock.
- `survey_view.py`: the survey the cell is on: the `survey` member of
  `X-RTLS-INF`, the lock warnings `not-surveyed`, `range-deviation` and
  `range-deviation-danger`, the
  Verify rules `survey` and `anchors`.
- `tag_table.py`: a tag's anchor table assembled from `antbl`.
- `survey.py`: `X-RTLS-SURVEY`, the survey run as a state machine and its
  commit; `survey_group.py` who takes part and in which order,
  `survey_request.py` the request's frame and placement,
  `survey_notify.py` the pushes of its state,
  `survey_commit.py` the survey ID and the `UWB_LOC` values and write.
- `survey_step.py`: one survey step on the anchors, over the extension's
  own socket: the command and the report are the SDK's (`rtlslink.survey`),
  the write-and-wait is done here because the SDK's needs its own client.
- `survey_fit.py`: anchor positions fitted to the ranges of an anchor
  survey, and the gate on committing them. Pure, numpy only.
- `verify.py`: `X-RTLS-VERIFY`.
- `fc_identity.py`: the tag-to-drone pairing.
- `selfcheck.py`: deprecated shim for `rtls-link selfcheck`.

## Tests

```sh
uv run pytest test/test_ext_rtls.py test/test_rtls_*.py
```

The tests drive the handlers against the SDK's sans-IO protocol core with a
scripted fake device, without sockets. Protocol tests belong to the SDK, in
rtls-link-zephyr `py/tests/`. `test/rtls_support.py`,
`test/anchor_lock_support.py`, `test/survey_support.py` (anchors that
take survey steps and store positions) and `test/survey_view_support.py`
(a surveyed cell and a tag that reports its table) hold the fakes; `test/test_ext_show_anchor_gate.py` covers the show gate.
