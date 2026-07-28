"""Default configuration for the Axio proxy server.

This script will be evaluated first when the proxy attempts to load its
configuration. Configuration files may import variables from this module
with `from flockwave.proxy.config import SOMETHING`, and may also modify
them if the variables are mutable.
"""

# Location of the local Axio Server that the proxy will connect to.
LOCAL_SERVER = "tcp://localhost:5000"

# Location of the remote socket that the proxy will connect to.
REMOTE_SERVER = "tcp://proxy.skybrush.collmot.com:5555"
