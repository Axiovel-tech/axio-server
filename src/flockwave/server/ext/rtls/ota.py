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

__all__ = ("upgrade",)


async def _upgrade_async(address: str, image_path: str, timeout: float) -> str:
    from smpclient import SMPClient
    from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
    from smpclient.requests.image_management import ImageUploadWrite  # noqa: F401
    from smpclient.requests.os_management import ResetWrite
    from smpclient.transport.udp import SMPUDPTransport

    with open(image_path, "rb") as f:
        image = f.read()

    client = SMPClient(SMPUDPTransport(), address, timeout_s=timeout)
    await client.connect()

    async for _offset in client.upload_file(image, "image"):
        pass

    states = await client.request(ImageStatesRead())
    slot1 = next(s for s in states.images if s.slot == 1)
    await client.request(ImageStatesWrite(hash=slot1.hash, confirm=False))
    await client.request(ResetWrite())
    return slot1.version


def upgrade(address: str, image_path: str, *, timeout: float = 10.0) -> str:
    """Blocking upgrade helper (run inside a worker thread from Trio).
    Returns the version string of the uploaded image."""
    return asyncio.run(_upgrade_async(address, image_path, timeout))
