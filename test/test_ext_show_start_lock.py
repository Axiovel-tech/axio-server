from time import time
from types import SimpleNamespace

from pytest import raises

from flockwave.server.ext.show.clock import ShowClock
from flockwave.server.ext.show.extension import DroneShowExtension

hub = SimpleNamespace(
    acknowledge=lambda message, outcome=True, reason=None: (outcome, reason)
)


def setcfg(extension: DroneShowExtension, start: dict):
    message = SimpleNamespace(body={"configuration": {"start": start}})
    return extension.handle_SHOW_SETCFG(message, None, hub)


def show_starting_at(start_time: float, duration: float = 60) -> DroneShowExtension:
    extension = DroneShowExtension()
    extension._clock = ShowClock()
    extension._clock_sync.point_of_no_return_seconds = -10
    extension._config.update_from_json(
        {
            "start": {"time": start_time, "clock": None, "method": "auto"},
            "duration": duration,
        }
    )
    extension._clock.start_time = start_time
    return extension


def test_start_moves_are_refused_from_the_point_of_no_return_to_the_end():
    start_time = time() + 5
    extension = show_starting_at(start_time)
    assert extension._is_start_locked()

    outcome, reason = setcfg(extension, {"time": start_time + 30, "clock": None})
    assert not outcome
    assert reason == (
        "The start is locked from T-10 s; only abort (clear the start) is possible"
    )
    assert extension._config.start_time_on_clock == start_time

    with raises(RuntimeError, match="locked"):
        extension._schedule_start(start_time + 30)

    # Control re-sends the unchanged start with other changes
    assert setcfg(extension, {"time": start_time, "authorized": True}) == (True, None)
    assert extension._config.authorized_to_start

    # The abort
    assert setcfg(extension, {"time": None}) == (True, None)
    assert extension._config.start_time_on_clock is None


def test_start_moves_are_accepted_before_the_point_of_no_return_and_after_the_end():
    for start_time in (time() + 11, time() - 61):
        extension = show_starting_at(start_time)
        assert not extension._is_start_locked()
        assert setcfg(extension, {"time": start_time + 30}) == (True, None)
        assert extension._config.start_time_on_clock == start_time + 30

    extension = show_starting_at(time() - 61, duration=60)
    extension._schedule_start(time() + 60)
