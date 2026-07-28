"""Main package for the Axio proxy that connects to a remote IP address and
port, and listens for incoming HTTP requests from there. Incoming requests are
then parsed and forwarded to a _local_ Axio Server; responses are relayed
back.

This can be used in field tests where the Axio Server on the field does not
have a public IP address that remote services may use. The setup in this case
can be as follows:

- A `socat` instance is set up on the remote server to listen on _two_
  TCP ports: port 5000 and 5001.

- An `nginx` proxy is set up on the remote server that performs SSL offloading
  and forwards all incoming HTTP requests to port 5001.

- The Axio Server and the Axio proxy is started up on the field computer.
  The proxy is instructed to connect to the remote server on port 5000 and to
  the local Axio Server, also on port 5000.
"""

from .version import __version__, __version_info__

__all__ = ("__version__", "__version_info__")
