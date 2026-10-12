from types import SimpleNamespace

from flockwave.server.ext.mavlink.takeoff import MAVLinkScheduledTakeoffManager
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.takeoff import TakeoffConfiguration


def test_uav_reporting_the_whole_second_start_time_needs_no_update():
    config = TakeoffConfiguration(
        authorization_scope=AuthorizationScope.LIVE, takeoff_time=1_760_000_012.75
    )
    in_sync = SimpleNamespace(
        scheduled_takeoff_authorization_scope=AuthorizationScope.LIVE,
        scheduled_takeoff_time=1_760_000_012,
    )
    stale = SimpleNamespace(
        scheduled_takeoff_authorization_scope=AuthorizationScope.LIVE,
        scheduled_takeoff_time=1_760_000_011,
    )

    needs_update = MAVLinkScheduledTakeoffManager.uav_needs_update
    assert not needs_update(None, in_sync, config)  # type: ignore[arg-type]
    assert needs_update(None, stale, config)  # type: ignore[arg-type]
