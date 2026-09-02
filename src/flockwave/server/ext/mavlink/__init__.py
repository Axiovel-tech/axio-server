"""Axio Server extension that adds support for drone flocks that use
the MAVLink protocol.
"""

from .extension import MAVLinkDronesExtension
from .schema import schema

__all__ = ("construct", "dependencies", "description", "enhancers", "schema")

construct = MAVLinkDronesExtension
dependencies = ("rc", "show", "signals")
description = "Support for drones that use the MAVLink protocol"
# The legacy generic firmware-update API writes ArduPilot ABIN files directly
# to their bootloader-visible name. Keep it unavailable so X-AP-OTA remains
# the only MAVLink application-firmware entry point and owns all safety gates.
enhancers = {}
