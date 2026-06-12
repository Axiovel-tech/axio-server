"""MCUmgr/SMP OTA orchestration for rtls-link devices.

Implements the rollback story for the ESP32-S3 (overwrite-only MCUboot,
no bootloader-level revert): upload, mark pending, reset, then health
check — and if the device does not come back, the caller re-uploads the
previous artifact while it is still reachable.

smpclient is asyncio-based; under the Trio-based server, run
:func:`upgrade` in a worker thread (``trio.to_thread.run_sync`` with
``asyncio.run``).
"""

from __future__ import annotations

import asyncio
from typing import Callable, Optional

__all__ = ("upgrade",)

#: progress callback: called with (bytes uploaded, total bytes)
ProgressCallback = Callable[[int, int], None]


async def _upgrade_async(
    address: str,
    image_path: str,
    timeout: float,
    on_progress: Optional[ProgressCallback] = None,
) -> str:
    from smpclient import SMPClient
    from smpclient.requests.image_management import (
        ImageStatesRead,
        ImageStatesWrite,
        ImageUploadWrite,  # noqa: F401
    )
    from smpclient.requests.os_management import ResetWrite
    from smpclient.transport.udp import SMPUDPTransport

    with open(image_path, "rb") as f:
        image = f.read()

    client = SMPClient(SMPUDPTransport(), address, timeout_s=timeout)
    await client.connect()

    async for offset in client.upload_file(image, "image"):
        if on_progress is not None:
            on_progress(offset, len(image))

    states = await client.request(ImageStatesRead())
    slot1 = next(s for s in states.images if s.slot == 1)
    await client.request(ImageStatesWrite(hash=slot1.hash, confirm=False))
    await client.request(ResetWrite())
    return slot1.version


def upgrade(
    address: str,
    image_path: str,
    *,
    timeout: float = 10.0,
    on_progress: Optional[ProgressCallback] = None,
) -> str:
    """Blocking upgrade helper (run inside a worker thread from Trio).
    Returns the version string of the uploaded image.

    ``on_progress``, if given, is invoked from the worker's event loop
    with ``(uploaded_bytes, total_bytes)`` as the upload proceeds; it
    must be quick and thread-safe."""
    return asyncio.run(_upgrade_async(address, image_path, timeout, on_progress))
