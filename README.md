![Tests](https://github.com/Axiovel-tech/axio-server/actions/workflows/tests.yml/badge.svg)
![Linters](https://github.com/Axiovel-tech/axio-server/actions/workflows/linters.yml/badge.svg)

# Skybrush Server

Skybrush Server is the server component behind the Skybrush ecosystem; it handles
communication channels to drones and provides an abstraction layer on top of them
so frontend apps (like Skybrush Live) do not need to know what type of drones
they are communicating with.

The server also provides additional facilities like clocks, RTK correction
sources, weather providers and so on. It is extensible via extension modules
that can be loaded automatically at startup or dynamically while the server is
running. In fact, most of the functionality in the server is implemented in the
form of extensions; see the `flockwave.server.ext` module in the source code
for the list of built-in extensions. You may also develop your own extensions to
extend the functionality of the server.

## About this fork (axio-server)

This repository is Axiovel's fork of Skybrush Server with built-in support
for the rtls-link UWB indoor positioning system:

- **`rtls` extension** — manages rtls-link devices over their MAVLink
  management channel (discovery/presence, parameters, sleep/wake, OTA,
  health and position telemetry, anchor beacons) and distributes the
  UWB-timebase show-clock GPS pin. See
  [`src/flockwave/server/ext/rtls/README.md`](src/flockwave/server/ext/rtls/README.md)
  for the full client message API (`X-RTLS-*`) and configuration. The
  extension is *not* in the default extension list; enable it in your
  configuration file.
- **`ltc_timecode` extension** — an SMPTE timecode clock registered under
  the id `mtc`, fed by a network LTC stream (one `HH:MM:SS:FF` timecode
  per UDP datagram, port 5005 by default), so a show can be scheduled on
  and follow the master timecode of a larger production.
- **RC start reflection** in the `mavlink` extension — on a fleet with a
  shared show clock (GPS, or the UWB-timebase synthetic GPS pin), the
  first drone that reports an RC-initiated start time is treated as the
  observer of the RC flip, and that one absolute start time is scheduled
  for the whole fleet (the start method switches to AUTO so it is visible
  and revocable in the GCS). Enabled by default; disable with
  `rc_start_reflection: false` in the `mavlink` extension configuration.

[`etc/conf/axio-rtls-indoor.jsonc`](etc/conf/axio-rtls-indoor.jsonc) is the
reference configuration for indoor shows. The MAVLink ingest convention for
real fleets is a **single shared UDP 14550 listener**: every rtls-link tag
bridges its flight controller's MAVLink stream to the server host on port
14550 (the `mavlink` extension's `default` connection), so no per-drone
ports or endpoints are configured.

## Installation

1. Install `uv`. `uv` will manage a virtual environment for this project to keep
   things nicely separated. You won't pollute the system Python with the
   dependencies of the Skybrush server and everyone will be happier.
   See <https://docs.astral.sh/uv/> for installation instructions.

2. Check out the source code of the server.

3. Run `uv sync` to install all the dependencies and the server itself in a
   separate virtualenv. The virtualenv will be created in a folder named
   `.venv` in the project folder.

4. Run `uv run skybrushd` to start the server.

## Documentation

- [User guide](https://doc.collmot.com/public/skybrush-live-doc/latest/)

## Development

This project contains both public and private dependencies in `pyproject.toml`.
Public dependencies are either on PyPI or in our public PyPI index at
[Gemfury](https://gemfury.com). Private dependencies hosted in our private
PyPI index are _not_ required to build the community version of Skybrush Server.

However, if you are working with the project on your own and make any changes to
`pyproject.toml` that would necessitate the regeneration of the lockfile of
`uv` (i.e. `uv.lock`), `uv` itself may attempt to connect to our private package
index as it needs information about _all_ dependencies to generate a consistent
lockfile. In this case, you should remove all the dependencies from
`pyproject.toml` that are pinned to the `collmot` package index -- these are
the ones in the `pro` or `collmot` extras.

## License

Skybrush Server is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Skybrush Server is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
