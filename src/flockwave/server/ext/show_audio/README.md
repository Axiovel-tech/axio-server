# show_audio — show music in sync with the show clock

Plays the show's music on an audio output of the server host. Audio t = 0
is show t = 0: sample 0 reaches the DAC at show T0 + `offsetMs`.

## Requirements

Decoding uses `soundfile`, whose Linux wheel bundles libsndfile (WAV, FLAC,
Ogg, MP3). Playback uses `sounddevice`, which needs the system PortAudio
library:

```sh
sudo apt install libportaudio2
```

Without it the extension still loads and accepts tracks; arming reports
state `error` ("PortAudio library not found").

## Configuration

| Key          | Default | Meaning                                                    |
|--------------|---------|------------------------------------------------------------|
| `device`     | `null`  | ALSA output device name (or part of it) or PortAudio index; `null` = system default output |
| `offset_ms`  | `0`     | Initial playback offset; positive = music later            |
| `arm_lead_s` | `5`     | How long before sample 0 the output stream is opened       |

Use PortAudio's ALSA host API. On a PipeWire host the default device goes
through PipeWire's ALSA plugin, whose DAC timestamps put the music within
±0.2 ms of the schedule on a null sink; PortAudio's PulseAudio host API
devices put it ~50 ms early and are refused. To play on another PipeWire
sink, keep `device` at `null` and start the server with
`PIPEWIRE_NODE=<sink node name>` (if that sink is missing, PipeWire falls
back to the default one).

## When it plays

- The player follows the `show` clock only when the server itself starts
  the show: start method AUTO, authorized (any scope, including rehearsal
  and lights-only), start time set. With the RC start method it logs that
  the music needs the AUTO start and stays idle.
- Before the show extension's point of no return (`point_of_no_return_seconds`,
  T−10 s by default) it follows reschedules and cancellations. After it the
  playback is committed: only `X-SHOW-AUDIO-STOP` or a server shutdown stops
  it, and track uploads, `X-SHOW-AUDIO-CLEAR` and `X-SHOW-AUDIO-SETCFG` are
  rejected.
- A start time already in the past joins mid-track at the matching
  position, unless the operator stopped the music for that start.
- Sample 0 is placed using PortAudio's `outputBufferDacTime`, mapped to
  `time.time()` in the audio callback; the track then plays without
  re-seeking. The log reports the measured start error and the
  end-of-track drift of the output device against the wall clock.

## Messages

- `X-SHOW-AUDIO-UPLOAD` `{sha256, name, mediaType, size, offset, data}` —
  `data` is a base64 chunk of at most 256 KiB decoded, sent in order from
  `offset` 0. Responds `{result: "partial" | "loaded"}`; the track replaces
  the current one once complete, its SHA-256 matches and it decodes.
- `X-SHOW-AUDIO-CLEAR` `{}` — unloads the track.
- `X-SHOW-AUDIO-INF` `{}` — `{track: {sha256, name, mediaType, duration} |
  null, state: "idle" | "armed" | "playing" | "finished" | "error",
  startTime, offsetMs, device, error}`. Also broadcast as a notification
  whenever it changes.
- `X-SHOW-AUDIO-STOP` `{}` — stops playback now (operator abort).
- `X-SHOW-AUDIO-SETCFG` `{offsetMs}` — sets the playback offset
  (|offsetMs| ≤ 10000).
