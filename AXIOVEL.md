# Axiovel's fork: what it adds and how to work on it

This repository is Axiovel's fork of Skybrush Server, the production ground
server of the indoor show stack. `AGENTS.md` is upstream's file (commands and
code style); this file is what the fork adds. Cross-repo facts live in
[AxioStack `contracts/`](https://github.com/Axiovel-tech/AxioStack/tree/main/contracts);
links below go there.

## Branches

`AV-dev-1.0` is Axiovel's branch; every Axiovel change, docs included, targets
it. `main` tracks upstream and takes no Axiovel commit.

## What the fork adds

| Area | Code | Read |
|---|---|---|
| rtls-link anchors and tags: discovery, parameters, sleep, OTA, health, anchor survey, pre-flight verify, cluster clock | `ext/rtls/` | `ext/rtls/README.md`; contracts `rtls-client-api.md`, `rtls-anchor-geometry.md`, `rtls-fleet-health.md` |
| `armed` in `UAV-INF`: the autopilot's arming state from its heartbeat, absent when stale or unknown; Control's RTLS guards read it, not `DISARMED` | `model/uav.py`, `ext/mavlink/driver.py` | `rtls-client-api.md` ("The drones' arming state") |
| Pre-show fleet health: the drones' IMU temperatures joined to their tags' stats | `ext/rtls/extension.py`, `ext/mavlink/network.py` | `rtls-fleet-health.md` |
| Show upload: `__show_upload` compiles a `.skyc` to `.skyb`, `__gdm_upload` validates an engine `.gdmb`; both write the drone's show slot `/axio/show.bin` over MAVFTP | `ext/mavlink/show_slot.py`, `show/gdm_upload.py`, `show/gdmb.py` | `show-artifacts.md`, `gdm-format.md` |
| Show start in UWB cluster time (`AXIO_SHOW_SCHEDULE`), point of no return | `ext/mavlink/schedule.py`, `ext/mavlink/takeoff.py`, `ext/show/` | `show-timing.md` |
| Show music, from the `.skyc` or `.gdm` Control loaded, played on the show clock | `ext/show_audio/` | its README; `show-timing.md` |
| SMPTE timecode clock `mtc` (LTC audio or UDP) | `ext/timecode/` | `show-timing.md` |
| Flight-controller log collection, `axio-fc-logs` | `ext/mavlink/log_collection/` | its README |
| ArduPilot firmware update staged on the SD card over MAVFTP (`X-AP-OTA`) | `ext/mavlink/firmware/` | the code |
| Live fleet networking | `etc/conf/axio-rtls-indoor.jsonc` | `.claude/skills/live-fleet-setup/` |

Paths are under `src/flockwave/server/`.

## Invariants

- **Stay mergeable with upstream.** Put Axiovel code in new modules where
  possible; do not rename, move or restyle upstream files. The rebrand kept on
  purpose: the `skybrushd` entry point, the `flockwave-server` package name,
  upstream's `etc/conf/skybrush-*.jsonc`, and the wire values
  Control and AVCopter depend on (`SYS-VER`, SSDP URNs, the `skyb` magic, the
  legacy `/collmot/show.skyb` path).
- **An `X-RTLS-*`, show-upload or timing change is a contract change.** Update
  the AxioStack contract and its other side with it.
- **The `rtls-link` SDK is pinned by commit** in `pyproject.toml`. Firmware
  stats or protocol changes reach the server only when that pin moves.
- **Live MAVLink is one listener on UDP 14550**, drones told apart by system
  id. Never add per-drone ports outside the simulator.

## Testing

- Unit tests and linters: the commands in `AGENTS.md`; CI runs
  `.github/workflows/tests.yml` and `linters.yml`.
- Against firmware and SITL: the simulated production stack (AxioStack
  `contracts/production-simulation.md`); `rtls-link sim gcs-config` writes
  this server's config for it.
- Hardware-only behaviour (Wi-Fi broadcast, timing on the real access point)
  needs a bench run; say so in the PR when it is still pending.
