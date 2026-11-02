# Flight-controller log collection (`axio-fc-logs`)

Downloads ArduPilot dataflash logs over MAVFTP and keeps only files whose
bytes are proven complete.

```bash
# Through the running server: it keeps its MAVLink link and writes on its host
axio-fc-logs collect --uav 03 --uav 04 --dest /abs/path/logs --latest 3

# A flight controller on USB (or any pymavlink address), several in parallel
axio-fc-logs collect --direct 3=/dev/serial/by-id/usb-ArduPilot_... \
    --direct 4=/dev/ttyACM1 --dest logs --file 00000042.BIN
```

`--port` is the server's TCP JSON port (base port + 1, 5001 by default).
SITL keeps its logs in `logs`, not `/APM/LOGS`: pass `--log-dir logs`.

## What a run does

1. Refuses unless the autopilot of the expected system ID answers, is
   disarmed, and has no authorized (`SHOW_START_AUTH`) or scheduled
   (`SHOW_START_TIME`) show start. With `--min-battery-voltage`, it also
   refuses on a low or unreported battery.
2. Sets `LOG_DISARMED=0` so the open log stops growing, after saving the
   original value to `collect-state.json`. A run killed before restoring
   leaves that file, and the next run restores the saved original rather than
   the current 0.
3. Picks the newest `--latest` logs (by `LASTLOG.TXT`, since log numbers wrap),
   or the named `--file`s.
4. Downloads with MAVFTP burst reads into `<name>.part`, records received
   ranges in `<name>.part.json` after syncing the data, and refetches gaps.
   A rerun resumes from the record.
5. Publishes `<name>` only when the flight controller's whole-file CRC32
   equals the local one; the receipt `<name>.json` adds the SHA-256. A
   mismatch discards the partial copy. An existing verified copy is reused
   after checking its SHA-256; one that no longer matches its receipt, or
   whose remote file has since grown, is reported and left untouched.
6. Stops if the vehicle arms or the battery drops below the limit.
7. Restores `LOG_DISARMED` and verifies the readback, also after a failure or
   cancellation (Ctrl-C cancels the server-side run too).

## Output

Per vehicle (`<dest>/<uav id>/` or `<dest>/sysid-<n>/` for direct links):
the verified logs, their receipts, `manifest.json` (all verified files),
`status.json` (latest status), and one `run-<time>.json` run receipt per
run, including the transport and the provenance of the code that ran.

Standard output is JSON lines: a fixed-field `status` event per vehicle per
`--status-interval` (`phase`: guarding, closing, listing, transferring,
verifying, restoring, done or failed; bytes, rate, ETA, battery), then one
`summary`. The exit status is 0 only when every selected file is verified and
`LOG_DISARMED` was restored.
