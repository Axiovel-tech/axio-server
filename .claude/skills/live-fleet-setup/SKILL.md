---
name: live-fleet-setup
description: How the live indoor fleet reaches this server - one shared MAVLink listener on UDP 14550, the RTLS management stream, the tag bridge parameters, broadcast, sysids, launch and checks. Read before touching mavlink connection config, assigning ports, or configuring rtls-link boards for a bench or show session. Per-instance port layouts belong to the simulator only.
---

# Live fleet setup

For the real fleet: drones whose rtls-link tags join the private board
network. The simulator shifts every port per instance on purpose; never copy
its layout into a live config (see the end).

Board-side rules (static addressing, the access point, the management
stream, the firewall, identity) are the firmware's: read
rtls-link-zephyr `docs/guidance/bench.md` ("Network") and
`docs/reference/networking.md`. This skill covers the server side.

## Addresses: discover them, never hard-code them

- A board's address is `NET_PREFIX` /24 plus its `MAV_SYS_ID` (no DHCP).
- The server host is the boards' `MGMT_HOST`: its address on the board
  network. Read `MGMT_HOST` and `NET_PREFIX` from any board
  (`rtls-link param get sysid:<N> MGMT_HOST`, or Control's RTLS parameter
  dialog), then find the host interface in that /24 with `ip -4 addr`.
- Which sysids are in use: the fleet records (`repos/ops/axio-fleet` in
  AxioStack) and the boards the server has found (`X-RTLS-INF`, Control's
  RTLS panels).

## MAVLink: one listener on UDP 14550

Every tag bridges its autopilot's MAVLink to `WIFI_UART_HOST:WIFI_UART_PORT`,
and every tag sends to the same place: the server host, port 14550. Drones
are told apart by MAVLink system id, not by port. The server needs exactly
one connection, as in `etc/conf/axio-rtls-indoor.jsonc`:
`udp-listen://:14550?broadcast_port=…`.

- Listen on all interfaces. A `127.0.0.1` listener receives nothing from
  the board network; four per-drone localhost listeners once showed zero
  drones for that reason.
- Never add per-drone connections or ports. A new drone appears with no
  server change once its tag bridges to 14550.

Tag parameters for a drone to appear:

| Parameter | Value |
|---|---|
| `WIFI_UART_EN` | 1 |
| `WIFI_UART_PORT` | 14550 |
| `WIFI_UART_HOST` | the server host's board-network address (= `MGMT_HOST`) |

They apply at the tag's next boot: set, read back, reboot the tag (the
autopilot keeps power). An empty or non-IPv4 `WIFI_UART_HOST` makes the tag
wait for the ground to speak first, so a drone missing while the others
stream usually has a wrong `WIFI_UART_HOST`.

## Broadcast: decided, not finished

Decision on axio-server#60 (read it with `gh pr view 60 -R Axiovel-tech/axio-server --comments`):
fleet-wide messages (broadcast-mode commands, lights, start and authorization
config, the show schedule) go out once, as a broadcast to port **14550**,
with no per-drone copies; show authorization goes by broadcast only, so a
drone that misses broadcasts visibly stays unauthorized; the access point
must pass wired-to-wireless broadcast.

Until that change lands:

- The configs still say `broadcast_port=14555`, a Skybrush convention. The
  tags listen only on `WIFI_UART_PORT` (14550), and the bench access point
  filters wired-to-wireless broadcast, so **no broadcast reaches a drone**.
  That includes Control's broadcast mode, which it switches to when a show
  start is authorized: the show page's stop buttons do not reach the drones.
- Do not "fix" it with per-drone copies or new ports, and do not treat 14555
  as correct. The switch to 14550 waits on the bench test #60 describes.

## RTLS management

Boards stream their management telemetry to `MGMT_HOST:MGMT_PORT` (3343 by
default) from boot; the `rtls` extension listens there and finds them
without a device list. See `src/flockwave/server/ext/rtls/README.md`.

## Sysids

- `199` is the factory default (`CONFIG_RTLS_LINK_MAVLINK_SYSTEM_ID`). A
  board still on it gets no address and its network stays down; it is set up
  over its serial or USB shell. 199 is never a fleet sysid.
- Every board needs its own sysid. A board given a fleet board's sysid takes
  that board's address, and the server keys devices by sysid, so commands for
  the real drone silently go to the spare (seen live: "drone stuck" was a
  bench rig answering under the drone's sysid with no autopilot). Find the
  culprit by MAC.

## Launch and checks

```sh
# in the axio-server checkout
uv run skybrushd -c etc/conf/axio-rtls-indoor.jsonc
```

- Before starting, check nothing stale holds the HTTP ports
  (`ss -tlnp | grep -E ':500[01]'`); a stale `skybrushd` makes the new one
  fail its bind.
- Ingest: `ss -ulnp | grep 14550` must show `0.0.0.0:14550`. If
  `sudo tcpdump -ni <board interface> udp port 14550` sees packets but no
  drone appears, the listener is wrong, not the drones.
- A board-initiated flow that seems dead: check the host firewall first
  (firmware `docs/guidance/bench.md`). The server needs UDP from the board
  subnet allowed to `MGMT_PORT` and 14550.

## Simulation is different on purpose

`rtls-link sim up` runs instance `n` on loopback: boards on `127.77.*`, the
RTLS management listener on `3343 + 100·n`, the MAVLink listener on
`14550 + 100·n` (rtls-link-zephyr `py/rtlslink/sim/plan.py`). Take the
server config from `rtls-link sim gcs-config`, never by hand (AxioStack
`contracts/production-simulation.md`, `contracts/rtls-sim-run-directory.md`).
Broadcast does not reach the sim tags either; #60 covers that too. Never
copy these ports into a live config, and never fix a live problem with fresh
ports: align the tags to 14550.
