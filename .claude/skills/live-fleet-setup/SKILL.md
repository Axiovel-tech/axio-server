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
`udp-listen://:14550?subnet=<board /24>&broadcast_port=14550`.

- Bind the board subnet: `subnet=` is the /24 of the tags' `NET_PREFIX`,
  and the listener binds the host's address in it. Bound to the subnet,
  every broadcast goes to the subnet's broadcast address from the start;
  bound to all interfaces it would go to `255.255.255.255` out of the
  default route (an office Wi-Fi, say) until the first drone is heard. A
  `127.0.0.1` listener receives nothing from the board network; four
  per-drone localhost listeners once showed zero drones for that reason.
- Keep `broadcast_port=14550`, the tags' bridge port (next section); a bare
  `udp-listen` crashes the mavlink extension on broadcast-address updates.
  The outdoor configs keep 14555.
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

## Broadcast: the access point and one server per subnet

Fleet-wide messages go out once, as a broadcast to the board subnet on
14550: commands in broadcast mode (Control switches to it when a show start
is authorized, so the show page's stop buttons use it), lights, the start
configuration and authorization, the time axis and the show schedule. Each
tag's bridge hands them to ArduPilot. Each drone acknowledges a broadcast
command; the server waits for the acknowledgments and sends the command
again to each drone that stayed silent, on its own address, as in selection
mode. The start configuration and schedule go only by broadcast: a drone
that misses them shows as not ready, and the start is cancelled at T-15 s,
before the drones commit to it, unless every drone of the show is ready.

So the network has to carry broadcast, at the bench and at every venue:

- The access point passes broadcast from the wired side to its wireless
  stations. Grandstream: SSID → Advanced → multicast/broadcast suppression
  **disabled**. The TP-Link TL-WA1201 passes it with no setting.
- DTIM 1, and no 802.11b rates on 2.4 GHz (Grandstream: radio → "Allow
  Legacy Devices (802.11b)" off; TP-Link: mode 802.11g/n mixed), so
  broadcasts go out at 6 Mbps or more instead of 1 Mbps.
- **One ground server per board subnet.** Every drone obeys every
  broadcast to its subnet: a second server on the same subnet commands your
  drones, and each server's start configuration (sent every second)
  overrides the other's. Teams sharing the access point and the wired
  network each take their own /24: the tags' `NET_PREFIX`, `WIFI_UART_HOST`
  and `MGMT_HOST` set to the team's server address in it, that address
  added to the server host's board interface, and `subnet=` set to it. A tag
  ignores another subnet's broadcast (measured 2026-11-23); only
  `255.255.255.255` reaches every subnet, and the `subnet=` binding never
  sends it. The subnets still share the radio, which costs nothing at these
  rates.
- Keep the server host quiet on its board interface (no mDNS or other
  chatter): every broadcast on the air costs every tag CPU time.

Measured on the bench (rtls-link-zephyr
`docs/records/2026-11-20-bench-broadcast.md`): broadcasts through the
TP-Link reached and were acknowledged by the addressed drone every time,
round trip 9–42 ms.

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
- Ingest: `ss -ulnp | grep 14550` must show the host's board address on
  14550. If
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
The simulator's broker stands in for the broadcast domain: it relays the
server's loopback broadcasts to every sim tag (rtls-link-zephyr
`docs/reference/simulation.md`). Never
copy these ports into a live config, and never fix a live problem with fresh
ports: align the tags to 14550.
