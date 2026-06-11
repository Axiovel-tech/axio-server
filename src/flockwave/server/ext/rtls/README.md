# rtls — Axiovel rtls-link device management (v0)

Manages rtls-link UWB positioning devices over their management UDP
channel (MAVLink 2, default port 3333):

- **Discovery**: the extension announces itself with GCS heartbeats
  (unicast to configured `devices` plus `broadcast` addresses); each
  device's management link learns the server from inbound datagrams and
  heartbeats back (component id 197). Devices are tracked with liveness
  timeout.
- **Configuration**: PARAM_EXT list/read/set against the device's
  parameter registry (an accepted set persists on the device).
- **OTA** (`ota.py`): MCUmgr/SMP upload → mark pending → reset, via
  `smpclient` (asyncio; run in a worker thread from Trio). On the
  ESP32-S3 MCUboot is overwrite-only — no bootloader revert — so the
  recovery path is health-check + re-upload of the previous artifact.

## Layout

- `protocol.py` — sans-IO protocol core: no sockets, no clock, no
  framework. Verified against live firmware by `selfcheck.py`
  (plain-asyncio/pymavlink, runs without a flockwave install):

  ```sh
  python3 selfcheck.py --host <device-ip> --port 3333
  ```

- `extension.py` — the Trio/flockwave wrapper (config schema below).
- `ota.py` — SMP upgrade helper.

## Configuration

```jsonc
"EXTENSIONS": {
  "rtls": {
    "port": 3333,
    "devices": ["192.168.4.1"],        // static addresses (optional)
    "broadcast": ["255.255.255.255"]   // discovery broadcast
  }
}
```

## v0 scope and what comes next

v0 tracks devices, fetches their parameter lists on discovery, and
exposes `devices` / `protocol` through the extension exports. Planned
next: a request/response parameter API for the control UI, anchors
registered as Beacon objects once positions exist (post-survey), and
OTA wired into a server command with the health-check/rollback loop.
