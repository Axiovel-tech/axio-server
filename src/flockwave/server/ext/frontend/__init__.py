"""Extension that adds a simple frontend index page to the Axio Server,
served over HTTP.
"""

from .extension import dependencies, description, exports, load, schema

__all__ = ("dependencies", "description", "exports", "load", "schema")
