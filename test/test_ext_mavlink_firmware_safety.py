"""Safety-state consistency tests for ArduPilot firmware updates."""

import pytest
from test_ext_mavlink_firmware_backend import FakeUAV, make_backend

from flockwave.server.ext.mavlink.firmware.backend import UpdateOperationError
from flockwave.server.ext.show.config import AuthorizationScope


@pytest.mark.parametrize(
    ("age", "takeoff_time", "authorization", "code", "detail"),
    [
        (
            float("inf"),
            None,
            AuthorizationScope.NONE,
            "showStateUnknown",
            "UAV drone-show state is unavailable or stale",
        ),
        (
            0,
            1,
            AuthorizationScope.NONE,
            "showScheduled",
            "UAV has a scheduled takeoff",
        ),
        (
            0,
            None,
            AuthorizationScope.LIVE,
            "showAuthorized",
            "UAV is authorized for a show",
        ),
    ],
)
def test_discovery_and_start_share_show_readiness_reason(
    age: float,
    takeoff_time: int | None,
    authorization: AuthorizationScope,
    code: str,
    detail: str,
) -> None:
    uav = FakeUAV(1177, show_status_age=age)
    uav.scheduled_takeoff_time = takeoff_time
    uav.scheduled_takeoff_authorization_scope = authorization
    backend = make_backend(uav)

    assert backend.target_state().reason_code == code
    with pytest.raises(UpdateOperationError) as raised:
        backend.check_safety(1177)

    assert raised.value.code == code
    assert str(raised.value) == detail
