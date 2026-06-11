"""Extension that manages Axiovel rtls-link UWB positioning devices:
discovery over the management UDP channel, PARAM_EXT configuration and
MCUmgr/SMP firmware updates."""

from .extension import construct, description, schema

__all__ = ("construct", "description", "schema")
