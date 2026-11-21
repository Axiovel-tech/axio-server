![Tests](https://github.com/Axiovel-tech/axio-server/actions/workflows/tests.yml/badge.svg)
![Linters](https://github.com/Axiovel-tech/axio-server/actions/workflows/linters.yml/badge.svg)

# Axio Server

> **Axiovel status:** this fork is Axiovel's production ground server. Control
> sends it each drone's show as a `.skyc` (`__show_upload`, compiled to
> `.skyb`) or a `.gdm` (`__gdm_upload`, the engine's `.gdmb` validated), and
> the server uploads it to AVCopter's show slot over MAVFTP.

Axio Server is the server component behind the Axio ecosystem; it handles
communication channels to drones and provides an abstraction layer on top of them
so frontend apps (like Axio Control) do not need to know what type of drones
they are communicating with.

The server also provides additional facilities like clocks, RTK correction
sources, weather providers and so on. It is extensible via extension modules
that can be loaded automatically at startup or dynamically while the server is
running. In fact, most of the functionality in the server is implemented in the
form of extensions; see the `flockwave.server.ext` module in the source code
for the list of built-in extensions. You may also develop your own extensions to
extend the functionality of the server.

## About this fork (axio-server)

Axiovel's fork adds rtls-link UWB device management, show upload in the `.gdm`
format, the show start in UWB cluster time, show music and timecode, fleet
health and flight-controller log collection. [`AXIOVEL.md`](AXIOVEL.md) maps
each to its code and to the cross-repo contracts.

[`etc/conf/axio-rtls-indoor.jsonc`](etc/conf/axio-rtls-indoor.jsonc) is the
reference configuration for indoor shows.

### Timecode input

The `timecode` extension's `ltc_audio` source needs the PortAudio library
(`sudo apt install libportaudio2`) and precise ADC timestamps, which
PortAudio's ALSA devices provide. On a PipeWire desktop use the `default` (or
`pipewire`) device with the LTC interface as the system's default input and
`sample_rate` set to the graph rate (48000 by default); PortAudio's PulseAudio
devices time blocks only to tens of milliseconds.

## Installation

1. Install `uv`. `uv` will manage a virtual environment for this project to keep
   things nicely separated. You won't pollute the system Python with the
   dependencies of the Axio Server and everyone will be happier.
   See <https://docs.astral.sh/uv/> for installation instructions.

2. Check out the source code of the server.

3. Run `uv sync` to install all the dependencies and the server itself in a
   separate virtualenv. The virtualenv will be created in a folder named
   `.venv` in the project folder.

4. Run `uv run skybrushd` to start the server.

## Documentation

- API documentation: run `uv run python doc/make.py` to build the
  `doc/axio-server-docs-<date>.zip` archive from the source code (pdoc).

## Development

Dependencies come from PyPI, Skybrush's public Gemfury index (`[tool.uv.index]`
in `pyproject.toml`) and, for the `rtls-link` SDK, Axiovel's rtls-link-zephyr
repository on GitHub, pinned by commit.

## License

Axio Server is a fork of Skybrush Server by CollMot Robotics Ltd.

Axio Server is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Axio Server is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
