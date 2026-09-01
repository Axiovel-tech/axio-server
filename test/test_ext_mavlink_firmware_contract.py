"""Exact wire-contract tests for the Control-facing X-AP-OTA messages."""

import base64
from types import SimpleNamespace
from typing import Any, cast

import pytest
import trio

from flockwave.server.app import SkybrushServer
from flockwave.server.ext.mavlink.firmware.backend import (
    FirmwareUpdateConfiguration,
    TargetState,
)
from flockwave.server.ext.mavlink.firmware.model import OTAJob
from flockwave.server.ext.mavlink.firmware.service import (
    MAX_ENCODED_APJ_SIZE,
    ArduPilotOTAService,
    _decode_payload,
    _job_body,
    _require_uav_id,
    _target_json,
)
from flockwave.server.ext.mavlink.firmware.transaction import (
    CancellationRejectedError,
)
from flockwave.server.message_hub import MessageHub
from flockwave.server.model import Client
from flockwave.server.model.builders import FlockwaveMessageBuilder


class CoordinatorStub:
    def __init__(self, job: OTAJob | None = None):
        self.job = job
        self.start_args: dict[str, Any] | None = None
        self.cancel_arg: str | None = None

    def get(self, uav_id: str) -> OTAJob | None:
        assert uav_id == "7"
        return self.job

    def start(self, **kwargs) -> OTAJob:
        self.start_args = kwargs
        assert self.job is not None
        return self.job

    def cancel(self, operation_id: str) -> OTAJob:
        self.cancel_arg = operation_id
        if self.job is None:
            raise KeyError(operation_id)
        return self.job


def make_message(**body):
    return FlockwaveMessageBuilder().create_message({"type": "X-AP-OTA", **body})


PROVISIONED = FirmwareUpdateConfiguration(provisioned_uav_ids=frozenset({"7"}))


async def make_service(*, uavs=None, app=None, configuration=PROVISIONED):
    hub = MessageHub()
    app = app or cast(SkybrushServer, SimpleNamespace(message_hub=hub))
    nursery_manager = trio.open_nursery()
    nursery = await nursery_manager.__aenter__()

    def provider():
        return uavs or []

    service = ArduPilotOTAService(app, nursery, provider, configuration)
    return service, hub, nursery_manager, nursery, provider, app


def test_target_wire_shape_matches_control_contract() -> None:
    target = TargetState(
        id="7",
        compatible=True,
        connected=True,
        disarmed=True,
        on_ground=True,
        power_sufficient=True,
        board_id=1177,
        current_hash="0123abcd",
        current_version="4.6.1",
        reason_code=None,
    )

    assert _target_json(target) == {
        "id": "7",
        "compatible": True,
        "currentHash": "0123abcd",
        "currentVersion": "4.6.1",
        "error": None,
        "label": "7",
        "safety": {
            "connected": True,
            "disarmed": True,
            "onGround": True,
            "powerSufficient": True,
        },
    }


def test_target_error_is_structured() -> None:
    target = TargetState(
        id="7",
        compatible=True,
        connected=True,
        disarmed=False,
        on_ground=True,
        power_sufficient=True,
        board_id=1177,
        current_hash="0123abcd",
        current_version="4.6.1",
        reason_code="armed",
    )
    assert _target_json(target)["error"] == {
        "code": "armed",
        "detail": "UAV is armed",
    }


def test_job_wire_shape_is_flat_and_response_op_is_preserved() -> None:
    job = OTAJob(operation_id="op-1", uav_id="7", name="arducopter.apj")
    job.transferred_bytes = 12
    job.total_bytes = 34
    job.expected.update(gitHash="0123abcd", version=None)
    job.observed.update(gitHash="89abcdef", version="4.6.1")

    assert _job_body("7", job, op="start") == {
        "type": "X-AP-OTA",
        "op": "start",
        "id": "7",
        "job": {
            "id": "7",
            "operationId": "op-1",
            "status": "running",
            "phase": "validating",
            "bytesTransferred": 12,
            "bytesTotal": 34,
            "committed": False,
            "cancellable": True,
            "expectedHash": "0123abcd",
            "expectedVersion": None,
            "observedHash": "89abcdef",
            "observedVersion": "4.6.1",
            "error": None,
        },
    }


async def test_cancel_request_needs_only_operation_id() -> None:
    hub = MessageHub()
    app = cast(SkybrushServer, SimpleNamespace(message_hub=hub))
    message = FlockwaveMessageBuilder().create_message(
        {"type": "X-AP-OTA", "op": "cancel", "operationId": "stale"}
    )
    async with trio.open_nursery() as nursery:
        service = ArduPilotOTAService(app, nursery, lambda: [])
        response = await service.handle_message(message, cast(Client, None), hub)
        nursery.cancel_scope.cancel()

    assert response.body["type"] == "ACK-NAK"
    assert response.body["reason"] == "No matching firmware update operation"


async def test_service_constructor_wires_configuration_and_dependencies() -> None:
    service, _, manager, nursery, provider, app = await make_service()
    try:
        assert service._app is app
        assert service._uavs is provider
        assert service._configuration.allowed_board_ids == frozenset({1177})
        assert service._configuration.provisioned_uav_ids == frozenset({"7"})
        assert service._coordinator._nursery is nursery
        assert callable(service._coordinator._backend_factory)
        assert callable(service._coordinator._notifier)
        assert service._coordinator._allowed_board_ids == frozenset({1177})
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)


async def test_service_passes_explicit_board_allowlist_to_coordinator() -> None:
    service, _, manager, nursery, _, _ = await make_service()
    try:
        configuration = FirmwareUpdateConfiguration(allowed_board_ids=frozenset())
        replacement = ArduPilotOTAService(
            service._app, nursery, service._uavs, configuration
        )
        assert replacement._coordinator._allowed_board_ids == frozenset()
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)


async def test_targets_response_has_exact_control_shape() -> None:
    uav = SimpleNamespace(id="7")
    service, hub, manager, nursery, _, _ = await make_service(uavs=[uav])
    target = TargetState(
        id="7",
        compatible=True,
        connected=True,
        disarmed=True,
        on_ground=True,
        power_sufficient=True,
        board_id=1177,
        current_hash="0123abcd",
        current_version="4.6.1",
        reason_code=None,
    )
    try:
        service._configuration = None
        service._uavs = lambda: []
        service._targets = lambda message, hub: hub.create_response_or_notification(
            {
                "type": "X-AP-OTA",
                "op": "targets",
                "targets": [_target_json(target)],
            },
            in_response_to=message,
        )
        request = make_message(op="targets")
        response = await service.handle_message(request, cast(Client, None), hub)
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)

    assert response.body == {
        "type": "X-AP-OTA",
        "op": "targets",
        "targets": [_target_json(target)],
    }
    assert response.refs == request.id


async def test_start_decodes_payload_and_preserves_request_operation() -> None:
    job = OTAJob(operation_id="op-1", uav_id="7", name="firmware.apj")
    coordinator = CoordinatorStub(job)
    uav = SimpleNamespace(id="7")
    service, hub, manager, nursery, _, _ = await make_service(uavs=[uav])
    service._coordinator = coordinator
    try:
        request = make_message(
            op="start",
            id="7",
            name="firmware.apj",
            image=base64.b64encode(b"apj").decode(),
            sha256="0" * 64,
        )
        response = await service.handle_message(
            request,
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)

    assert coordinator.start_args == {
        "uav_id": "7",
        "name": "firmware.apj",
        "payload": b"apj",
        "sha256": "0" * 64,
    }
    assert response.body["op"] == "start"
    assert response.body["id"] == "7"
    assert response.body["job"]["operationId"] == "op-1"
    assert response.refs == request.id


async def test_start_rejects_uav_without_provisioned_bootloader() -> None:
    job = OTAJob(operation_id="op-1", uav_id="7", name="firmware.apj")
    coordinator = CoordinatorStub(job)
    service, hub, manager, nursery, _, _ = await make_service(
        uavs=[SimpleNamespace(id="7")],
        configuration=FirmwareUpdateConfiguration(),
    )
    service._coordinator = coordinator
    try:
        response = await service.handle_message(
            make_message(
                op="start",
                id="7",
                name="firmware.apj",
                image="YXBq",
                sha256="0" * 64,
            ),
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)

    assert response.body["reason"] == (
        "UAV is not provisioned with the OTA bootloader"
    )
    assert coordinator.start_args is None


async def test_status_rejects_stale_operation_and_accepts_current_one() -> None:
    job = OTAJob(operation_id="op-1", uav_id="7", name="firmware.apj")
    service, hub, manager, nursery, _, _ = await make_service()
    service._coordinator = CoordinatorStub(job)
    try:
        stale = await service.handle_message(
            make_message(op="status", id="7", operationId="stale"),
            cast(Client, None),
            hub,
        )
        request = make_message(op="status", id="7", operationId="op-1")
        current = await service.handle_message(
            request,
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)

    assert stale.body["reason"] == "No matching firmware update operation"
    assert current.body["op"] == "status"
    assert current.body["id"] == "7"
    assert current.body["job"]["operationId"] == "op-1"
    assert current.refs == request.id


async def test_status_rejects_non_string_operation_even_if_job_is_malformed() -> None:
    job = OTAJob(operation_id=cast(str, 7), uav_id="7", name="firmware.apj")
    service, hub, manager, nursery, _, _ = await make_service()
    service._coordinator = CoordinatorStub(job)
    try:
        response = await service.handle_message(
            make_message(op="status", id="7", operationId=7),
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)
    assert response.body["reason"] == "No matching firmware update operation"


async def test_cancel_routes_only_operation_id_and_preserves_op() -> None:
    job = OTAJob(operation_id="op-1", uav_id="7", name="firmware.apj")
    coordinator = CoordinatorStub(job)
    service, hub, manager, nursery, _, _ = await make_service()
    service._coordinator = coordinator
    try:
        request = make_message(op="cancel", operationId="op-1")
        response = await service.handle_message(
            request,
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)

    assert coordinator.cancel_arg == "op-1"
    assert response.body["op"] == "cancel"
    assert response.body["id"] == "7"
    assert response.body["job"]["operationId"] == "op-1"
    assert response.refs == request.id


@pytest.mark.parametrize("operation_id", [None, "", 7])
async def test_cancel_requires_string_operation_id(operation_id) -> None:
    service, hub, manager, nursery, _, _ = await make_service()
    try:
        response = await service.handle_message(
            make_message(op="cancel", operationId=operation_id),
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)
    assert response.body["reason"] == "Missing operationId"


async def test_cancel_rejection_is_returned_cleanly() -> None:
    class RejectingCoordinator(CoordinatorStub):
        def cancel(self, operation_id: str) -> OTAJob:
            raise CancellationRejectedError("past commit")

    service, hub, manager, nursery, _, _ = await make_service()
    service._coordinator = RejectingCoordinator()
    try:
        response = await service.handle_message(
            make_message(op="cancel", operationId="op-1"),
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)
    assert response.body["reason"] == "past commit"


@pytest.mark.parametrize(
    ("image", "reason"),
    [
        (None, "Firmware image must be a non-empty base64 string"),
        ("", "Firmware image must be a non-empty base64 string"),
        ("%%%", "Firmware image is not valid base64"),
        ("A" * (MAX_ENCODED_APJ_SIZE + 1), "Encoded APJ exceeds the server size limit"),
    ],
)
def test_payload_decoder_rejects_invalid_or_oversized_images(
    image, reason: str
) -> None:
    with pytest.raises(ValueError) as raised:
        _decode_payload(image)
    assert str(raised.value) == reason


def test_payload_decoder_accepts_bound_and_validates_padding() -> None:
    assert _decode_payload(base64.b64encode(b"apj").decode()) == b"apj"
    encoded = "A" * MAX_ENCODED_APJ_SIZE
    assert len(_decode_payload(encoded)) == 3 * 1024 * 1024


@pytest.mark.parametrize(("name", "sha256"), [(None, "0" * 64), ("firmware.apj", None)])
async def test_start_requires_both_name_and_hash(name, sha256) -> None:
    service, hub, manager, nursery, _, _ = await make_service()
    try:
        response = await service.handle_message(
            make_message(op="start", id="7", name=name, sha256=sha256, image="YQ=="),
            cast(Client, None),
            hub,
        )
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)
    assert response.body["reason"] == "Missing firmware name or SHA-256"


@pytest.mark.parametrize("uav_id", [None, "", 7, "x" * 129])
def test_uav_id_validation(uav_id) -> None:
    message = make_message(id=uav_id)
    with pytest.raises(ValueError) as raised:
        _require_uav_id(message)
    assert str(raised.value) == "Missing or invalid UAV id"
    assert _require_uav_id(make_message(id="x" * 128)) == "x" * 128


async def test_broadcast_is_always_status_and_contains_operation_id() -> None:
    sent = []

    class Hub:
        def create_notification(self, body):
            return body

        async def broadcast_message(self, message) -> None:
            sent.append(message)

    app = cast(SkybrushServer, SimpleNamespace(message_hub=Hub()))
    service, _, manager, nursery, _, _ = await make_service(app=app)
    job = OTAJob(operation_id="op-1", uav_id="7", name="firmware.apj")
    try:
        await service._broadcast_job(job)
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)
    assert sent == [_job_body("7", job, op="status")]


async def test_backend_lookup_preserves_uav_and_configuration() -> None:
    uav = SimpleNamespace(id="7")
    service, _, manager, nursery, _, _ = await make_service(uavs=[uav])
    try:
        backend = service._backend_for_id("7")
        assert backend._uav is uav
        assert backend._configuration is service._configuration
        with pytest.raises(KeyError) as raised:
            service._backend_for_id("missing")
        assert raised.value.args == ("No such MAVLink UAV: missing",)
    finally:
        nursery.cancel_scope.cancel()
        await manager.__aexit__(None, None, None)
