"""Logger object for the Axio Server."""

from flockwave.logger import log as base_log

__all__ = ("log",)

log = base_log.getChild("server")
