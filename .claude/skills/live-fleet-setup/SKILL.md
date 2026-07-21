---
name: live-fleet-setup
description: How the LIVE indoor drone fleet reaches this server (network topology, the single UDP 14550 MAVLink ingest convention, device-side bridge params, ports, launch recipe). Read BEFORE touching mavlink connection config, assigning ports, or configuring RTLS-Link devices for a live/bench session — the multi-port patterns you may find elsewhere belong to SIMULATION isolation only.
---

# Live fleet setup (singu-server bench / indoor shows)

The rules in this file are for the **real fleet** (physical drones with
RTLS-Link tags on the board network). Simulations have different,
deliberately isolated conventions — see "Sim vs live" below. The most
common agent mistake this skill exists to prevent: **inventing new ports
or per-drone listeners for live MAVLink. Don't. Everything arrives on
UDP 14550.**

## Topology

The host (`singu-server`) is multi-homed:

| Interface | Address | Role |
|---|---|---|
| `enp2s0` | `192.168.0.100/24` | Board network (private AP). All RTLS-Link devices live here. |
| `wlp1s0` | `10.10.10.x/24` | Internet uplink, default route. |

`ufw` is default-deny incoming: any **board-initiated** flow needs an
explicit allow on `enp2s0`. Existing allows: 3333/udp (mgmt), 3334/udp
(logs), 3335/udp (anchor telemetry), 3343/udp (phone-home), 14550/udp
(mavlink). Conntrack masks the deny for host-initiated flows, which
makes firewall problems look intermittent — check `sudo ufw status`
first when a board-initiated feature seems dead.

## MAVLink ingest: ONE listener, UDP 14550

Every drone's flight-controller MAVLink reaches this server through its
RTLS-Link tag's WiFi-UART bridge, and **every tag sends to the same
place: `192.168.0.100:14550`**. Drones are distinguished by MAVLink
sysid, not by port. The server therefore needs exactly one connection in
the `mav` network:

```jsonc
"mavlink": {
  "enabled": true,
  "networks": {
    "mav": {
      "connections": [
        "udp-listen://:14550?broadcast_port=14555"
      ]
    }
  }
}
```

- Bind all interfaces (or `192.168.0.100`) — **never `127.0.0.1`**: the
  packets arrive on `enp2s0`, and a localhost bind silently receives
  nothing (this exact bug cost a live troubleshooting round on
  2026-07-21: four `127.0.0.1:1460x` listeners, zero drones visible).
- Always keep the `?broadcast_port=` suffix — a bare `udp-listen`
  crashes the mavlink ext on broadcast-address updates.
- Do NOT add per-drone connections/ports for live drones. New drones
  appear with zero server config changes as long as their tag bridges
  to 14550.

## Device-side bridge (RTLS-Link tag params)

For a drone's ArduPilot to appear, its tag must have:

```
WIFI_UART_EN   = 1
WIFI_UART_PORT = 14550
WIFI_UART_HOST = 192.168.0.100
```

`WIFI_UART_HOST` is the usual culprit when one drone is missing while
others stream: an empty host means the bridge never transmits. These
params are applied **at tag boot** — after changing them, reboot the tag
(`kernel reboot cold` over SMP; the FC keeps power through a tag
reboot). Set + verify readback + reboot, in that order.

Mind mgmt-link contention: the running server polls every device on
its single learned UDP peer, so CLI `param set/get` against a live
server flakes intermittently — retry a couple of times before
concluding anything.

## RTLS device management

The rtls ext (see `etc/conf/axio-rtls-indoor.jsonc`) lists device mgmt
addresses (`192.168.0.101`–`.112`, port 3333) with `"passive": true`.
DHCP reshuffles IPs between sessions — identify devices by MAC/sysid,
never by remembered IP. Device roster (stable): anchors sysid 101–108,
tags/drones 19x–200.

## Launch recipe

```sh
cd ~/dev/fw/axiovel/axio-server
.venv/bin/skybrushd -c etc/conf/axio-rtls-indoor.jsonc
# GUI (dev server): cd ~/dev/fw/axiovel/control && npm start -- --host 0.0.0.0
```

Before starting, check nothing stale holds 5000/5001 (`ss -tlnp`); a
stale skybrushd makes the new one fail its HTTP bind. Verify ingest
with `ss -ulnp | grep 14550` (must show `0.0.0.0:14550`) and, when in
doubt, `sudo tcpdump -ni enp2s0 udp port 14550` — if packets arrive but
no UAVs appear, the listener is wrong, not the drones.

## Sim vs live (why you may have seen other port schemes)

The simulation stack (`rtls-link sim up`, `SIM_INSTANCE`) deliberately
shifts every well-known port per instance (e.g. mgmt 3333+100·n,
per-vehicle gcs_out on 14600+10·n) so concurrent sims don't collide.
Those schemes are correct **in sim scenarios only**. Never copy
per-instance or per-vehicle port layouts into the live config, and
never "fix" a live problem by assigning fresh ports — align the device
side to 14550 instead.

## Spare boards default to sysid 200

A factory-fresh / spare AxioLight tag boots with `MAV_SYS_ID = 200` — the
same sysid as a fleet tag. The server keys devices by sysid, so powering a
spare next to a live fleet hijacks that identity: commands for the real
drone silently go to the spare (observed live 2026-07-21: "drone 11 is
stuck" was a bench test rig answering as 200 with no FC attached, whose
arming gate refused every sleep). Before powering any spare on the board
network, renumber it (`rtls-link param set <ip> MAV_SYS_ID 250` + reboot)
or keep it off the fleet AP. Identify the culprit by MAC, not IP.
