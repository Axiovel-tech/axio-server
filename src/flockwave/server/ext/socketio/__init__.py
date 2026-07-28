"""Extension that extends the Axio Server with support for incoming
messages on a Socket.IO connection.
"""

from .extension import dependencies, description, run, schema

__all__ = ("dependencies", "description", "run", "schema")
