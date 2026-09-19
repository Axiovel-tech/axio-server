# Field operations through Axio Server

`axio-operator` uses the running server's TCP interface, normally localhost
port 5001. It reuses the server's MAVLink connections for flight-controller
parameters and telemetry. Tag power and fitted geometry go through the RTLS
SDK after checking the server's onboard association against the tag itself.
It does not open a second flight-controller connection or change stream rates.

Start with `uv run axio-operator devices`. Copy the server UAV IDs from the
result; a server ID need not be the numeric MAVLink system ID. Do not infer
an association from a sticker, an IP suffix, or a numeric server ID.

```sh
uv run axio-operator devices
uv run axio-operator power --fc-id 11 --wake --wait-for flight-controller
uv run axio-operator params read --uav 11 --profile rc-switches
uv run axio-operator params compare --uav 6 --reference 11 --profile rc-switches
uv run axio-operator params apply --uav 6 --reference 11 --profile rc-switches
uv run axio-operator telemetry --uav 6 --uav 11
uv run axio-operator geometry --uav 6 --uav 11 --tolerance 0.05
uv run axio-operator power --uav 6 --sleep
```

The IDs above are examples. For a remote server, put `--host HOST --port PORT`
before the command. The existing TCP interface must be enabled and reachable.
The operator client does not implement server authentication; use a trusted
local connection or tunnel with the server's existing deployment protections.

## Completion and failures

Commands print JSON. Exit zero means the requested check or operation completed;
comparison differences and partial failures return nonzero. Missing data is
reported explicitly. An unavailable optional temperature is listed separately
and does not invalidate a received heartbeat.

Power reports acceptance, an observed reboot when required, tag reconnection,
and optionally a live onboard flight-controller heartbeat. The latter is not
flight readiness. Sleep waits for actual sleep state and reports an arming
refusal. These operations finish on observed state; their timeout is a limit,
not an unconditional delay. Firmware with requested-heartbeat and correlated
uptime support is required for verified power. Older firmware can still be
inspected, but an unsupported verification step fails explicitly.

`--timeout` defaults to 30 seconds. Flight-controller operations include time
queued behind another operation. The server stops early enough to return a
partial result before either the client or its command manager expires.
`values`, `errors`, and completed `changes` remain available on that path.
A write marked `unverified` has an unknown outcome and must be read back before
retrying. Disconnection of the CLI does not undo an already accepted command.

## Parameters and ownership

Use `params read --name NAME` for selected fresh reads. Repeat `--name` and
`--uav` to select more. `--profile rc-switches` selects flight mode assignments,
RC channel options, and stick channel mapping. It supplies names, not assumed
correct values; explicitly choose a tested reference drone or a reviewed JSON
file with `--values FILE`.

Compare and apply fetch current values and types. Apply validates every desired
value for that drone before its first write, skips unchanged values, writes in
selection/file order, and verifies each changed value with a fresh read. Integer
values that cannot survive the MAVLink float representation exactly are rejected.
A fresh armed heartbeat blocks each write. A failure stops subsequent writes on
that drone and reports earlier changes; there is no automatic rollback or fleet
transaction. Another client can still change parameters or arm a vehicle, so
keep manual control changes paused during configuration.

The server handles at most two operator UAVs at once and four selected parameter
reads per UAV. Commands for the same UAV are serialized. It retries missing
MAVLink replies through the existing driver. RTLS parameters belong to
`rtls-link fleet read/compare/apply`; use those commands for tag configuration.
ArduPilot remains authoritative for flight-controller settings.

## Identity while sleeping

Updated tag firmware learns the local autopilot heartbeat and publishes read-only
`FC_SYS_ID` and `FC_STATE` metadata. The server exposes this as `flightController`
in `X-RTLS-INF` and the `devices` output, with state and observation age:

- `unknown`: no confirmed onboard identity.
- `live`: recently confirmed while awake.
- `remembered`: previously confirmed, including a sleeping or disconnected FC.
- `ambiguous`: conflicting identities or duplicate tag/FC IDs.

The server maps the association to an existing UAV ID only when it is unique.
It does not invent a server ID. At a cold start, `power --fc-id` can select a
sleeping tag by its remembered numeric FC identity before a server UAV exists.
Remembered metadata can be stale after replacing hardware; it is not a current
flight-controller heartbeat. Ambiguous identity blocks targeted operations.
Old firmware keeps the existing live source-IP mapping and cannot provide the
same sleeping-identity guarantee.

## Geometry and temperatures

Geometry reads the actual fitted anchor table through the RTLS SDK's SMP shell
support. It checks calibration generation, complete tables, frame parameters,
reboot continuity, freshness, and every pair of selected drones. It compares raw
NED coordinates and reports the worst difference. It does not align tables in a
way that could hide different flight coordinate frames. Uncalibrated, stale,
missing, mismatched, or unsupported data cannot pass.

Telemetry requests fresh MCU, IMU, and barometer samples by default. Use repeated
`--sensor` options for `MCU`, `IMU`, `IMU2`, `IMU3`, `barometer`, or `barometer2`.
Every available value includes degrees Celsius, the MAVLink message source,
request duration, and sample age at completion. Missing sensors stay unavailable;
no value is inferred from the enclosure, ambient temperature, or another drone.
The heartbeat reports armed state, system status, and custom mode. It does not
summarize all pre-arm checks or declare the drone ready to fly.
