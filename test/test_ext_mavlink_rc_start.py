"""Tests for the RC show-start reflector (fleet-safe RC trigger)."""

from types import SimpleNamespace

from flockwave.server.ext.mavlink.rc_start import RCStartReflector, find_rc_start_time
from flockwave.server.ext.show.config import (
    AuthorizationScope,
    DroneShowConfiguration,
    StartMethod,
)

NOW = 1_752_000_000.0


def uav(t):
    return SimpleNamespace(scheduled_takeoff_time=t)


def make_config(
    method=StartMethod.RC, authorized=True, start_time=None
) -> DroneShowConfiguration:
    config = DroneShowConfiguration()
    config.start_method = method
    config.authorized_to_start = authorized
    config.authorization_scope = (
        AuthorizationScope.LIVE if authorized else AuthorizationScope.NONE
    )
    config.start_time_on_clock = start_time
    return config


def test_find_rc_start_time_picks_earliest_plausible():
    uavs = [
        uav(None),
        uav(int(NOW + 10)),  # the RC observer's schedule
        uav(int(NOW + 11)),  # a second drone's own (jittered) schedule
        uav(int(NOW - 100)),  # stale leftover from a past run
        uav(int(NOW + 4000)),  # implausibly far future
    ]
    assert find_rc_start_time(uavs, now=NOW) == int(NOW + 10)


def test_find_rc_start_time_none_without_reports():
    assert find_rc_start_time([uav(None), uav(None)], now=NOW) is None


class _Recorder:
    def __init__(self, config):
        self.config = config
        self.scheduled = []

    def get_configuration(self):
        return self.config

    def schedule_start(self, t):
        self.scheduled.append(t)


def test_reflector_reflects_when_armed():
    rec = _Recorder(make_config())
    reflector = RCStartReflector(rec.get_configuration, rec.schedule_start)
    assert reflector.check([uav(int(NOW + 10)), uav(int(NOW + 11))], now=NOW)
    assert rec.scheduled == [float(int(NOW + 10))]


def test_reflector_requires_rc_method():
    rec = _Recorder(make_config(method=StartMethod.AUTO))
    reflector = RCStartReflector(rec.get_configuration, rec.schedule_start)
    assert not reflector.check([uav(int(NOW + 10))], now=NOW)
    assert rec.scheduled == []


def test_reflector_requires_authorization():
    rec = _Recorder(make_config(authorized=False))
    reflector = RCStartReflector(rec.get_configuration, rec.schedule_start)
    assert not reflector.check([uav(int(NOW + 10))], now=NOW)
    assert rec.scheduled == []


def test_reflector_defers_to_existing_schedule():
    """A server-side scheduled start (or an earlier reflection) owns the
    start time; the reflector never overrides it."""
    rec = _Recorder(make_config(start_time=NOW + 60))
    reflector = RCStartReflector(rec.get_configuration, rec.schedule_start)
    assert not reflector.check([uav(int(NOW + 10))], now=NOW)
    assert rec.scheduled == []


def test_reflector_idle_without_reports():
    rec = _Recorder(make_config())
    reflector = RCStartReflector(rec.get_configuration, rec.schedule_start)
    assert not reflector.check([uav(None)], now=NOW)
    assert rec.scheduled == []
