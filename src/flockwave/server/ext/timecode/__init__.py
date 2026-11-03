"""Extension that provides the SMPTE timecode clock ``mtc``, fed by a
configurable timecode source, so a drone show can be scheduled on and
follow the timecode of a larger production."""

from .extension import dependencies, description, run, schema

__all__ = ("dependencies", "description", "run", "schema")
