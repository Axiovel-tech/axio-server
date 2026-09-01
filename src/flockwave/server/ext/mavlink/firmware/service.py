"""Flockwave message handler for Axio's ArduPilot firmware updater."""

from __future__ import annotations

import base64
import binascii
from typing import TYPE_CHECKING, Callable

from .apj import MAX_APJ_SIZE
from .backend import (
    ArduPilotUpdateBackend,
    FirmwareUpdateConfiguration,
    _reason_detail,
)
from .model import OTAJob
from .transaction import (
    CancellationRejectedError,
    FirmwareUpdateCoordinator,
    UpdateBusyError,
)

if TYPE_CHECKING:
    import trio

    from flockwave.server.app import SkybrushServer
    from flockwave.server.message_hub import MessageHub
    from flockwave.server.model import Client, FlockwaveMessage

    from ..driver import MAVLinkUAV

MAX_ENCODED_APJ_SIZE = ((MAX_APJ_SIZE + 2) // 3) * 4


class ArduPilotOTAService:
    """Exposes one-at-a-time SD-card firmware updates as ``X-AP-OTA``."""

    def __init__(
        self,
        app: SkybrushServer,
        nursery: trio.Nursery,
        uavs: Callable[[], list[MAVLinkUAV]],
        configuration: FirmwareUpdateConfiguration | None = None,
    ):
        configuration = configuration or FirmwareUpdateConfiguration()
        self._app = app
        self._uavs = uavs
        self._configuration = configuration
        self._coordinator = FirmwareUpdateCoordinator(
            nursery,
            self._backend_for_id,
            self._broadcast_job,
            allowed_board_ids=configuration.allowed_board_ids,
        )

    async def handle_message(
        self, message: FlockwaveMessage, _sender: Client, hub: MessageHub
    ):
        op = message.body.get("op")
        if op == "targets":
            return self._targets(message, hub)
        if op == "cancel":
            return self._cancel(message, hub)
        try:
            uav_id = _require_uav_id(message)
        except ValueError as ex:
            return hub.reject(message, reason=str(ex))
        if op == "status":
            return self._job_response(message, hub, uav_id)
        if op == "start":
            return self._start(message, hub, uav_id)
        return hub.reject(message, reason="Invalid X-AP-OTA operation")

    def _targets(self, message: FlockwaveMessage, hub: MessageHub):
        targets = []
        for uav in self._uavs():
            state = ArduPilotUpdateBackend(uav, self._configuration).target_state()
            targets.append(_target_json(state))
        return hub.create_response_or_notification(
            {"type": "X-AP-OTA", "op": "targets", "targets": targets},
            in_response_to=message,
        )

    def _job_response(self, message: FlockwaveMessage, hub: MessageHub, uav_id: str):
        job = self._coordinator.get(uav_id)
        operation_id = message.body.get("operationId")
        if operation_id is not None and (
            not isinstance(operation_id, str)
            or job is None
            or job.operation_id != operation_id
        ):
            return hub.reject(message, reason="No matching firmware update operation")
        return hub.create_response_or_notification(
            _job_body(uav_id, job, op="status"), in_response_to=message
        )

    def _cancel(self, message: FlockwaveMessage, hub: MessageHub):
        operation_id = message.body.get("operationId")
        if not isinstance(operation_id, str) or not operation_id:
            return hub.reject(message, reason="Missing operationId")
        try:
            job = self._coordinator.cancel(operation_id)
        except KeyError:
            return hub.reject(message, reason="No matching firmware update operation")
        except CancellationRejectedError as ex:
            return hub.reject(message, reason=str(ex))
        return hub.create_response_or_notification(
            _job_body(job.uav_id, job, op="cancel"), in_response_to=message
        )

    def _start(self, message: FlockwaveMessage, hub: MessageHub, uav_id: str):
        name = message.body.get("name")
        sha256 = message.body.get("sha256")
        image = message.body.get("image")
        if not isinstance(name, str) or not isinstance(sha256, str):
            return hub.reject(message, reason="Missing firmware name or SHA-256")
        try:
            payload = _decode_payload(image)
            self._backend_for_id(uav_id)
            job = self._coordinator.start(
                uav_id=uav_id,
                name=name,
                payload=payload,
                sha256=sha256,
            )
        except (ValueError, KeyError) as ex:
            return hub.reject(message, reason=str(ex))
        except UpdateBusyError as ex:
            return hub.reject(message, reason=str(ex))
        return hub.create_response_or_notification(
            _job_body(uav_id, job, op="start"), in_response_to=message
        )

    def _backend_for_id(self, uav_id: str) -> ArduPilotUpdateBackend:
        for uav in self._uavs():
            if uav.id == uav_id:
                return ArduPilotUpdateBackend(uav, self._configuration)
        raise KeyError(f"No such MAVLink UAV: {uav_id}")

    async def _broadcast_job(self, job: OTAJob) -> None:
        hub = self._app.message_hub
        await hub.broadcast_message(
            hub.create_notification(_job_body(job.uav_id, job, op="status"))
        )


def _decode_payload(image: object) -> bytes:
    if not isinstance(image, str) or not image:
        raise ValueError("Firmware image must be a non-empty base64 string")
    if len(image) > MAX_ENCODED_APJ_SIZE:
        raise ValueError("Encoded APJ exceeds the server size limit")
    try:
        return base64.b64decode(image, validate=True)
    except (ValueError, binascii.Error) as ex:
        raise ValueError("Firmware image is not valid base64") from ex


def _require_uav_id(message: FlockwaveMessage) -> str:
    value = message.body.get("id")
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError("Missing or invalid UAV id")
    return value


def _job_body(uav_id: str, job: OTAJob | None, *, op: str) -> dict:
    return {
        "type": "X-AP-OTA",
        "op": op,
        "id": uav_id,
        "job": job.json() if job else None,
    }


def _target_json(state) -> dict:
    return {
        "id": state.id,
        "compatible": state.compatible,
        "currentHash": state.current_hash,
        "currentVersion": state.current_version,
        "error": (
            {"code": state.reason_code, "detail": _reason_detail(state)}
            if state.reason_code
            else None
        ),
        "label": state.id,
        "safety": {
            "connected": state.connected,
            "disarmed": state.disarmed,
            "onGround": state.on_ground,
            "powerSufficient": state.power_sufficient,
        },
    }
