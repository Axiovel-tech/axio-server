import logging
from time import time
from types import SimpleNamespace

from pytest import fixture, raises

from flockwave.server.ext.clocks import ClockRegistry
from flockwave.server.ext.show.clock import ClockSynchronizationHandler, ShowClock
from flockwave.server.ext.show.extension import DroneShowExtension
from flockwave.server.model.clock import TimeElapsedSinceReferenceClock

ClockPair = tuple[ShowClock, ShowClock]


@fixture
def clocks() -> ClockPair:
    return ShowClock(), ShowClock()


@fixture
def handler() -> ClockSynchronizationHandler:
    return ClockSynchronizationHandler()


@fixture
def handler_with_point_of_no_return() -> ClockSynchronizationHandler:
    return ClockSynchronizationHandler(point_of_no_return_seconds=-10)


def test_disable_and_stop(clocks: ClockPair, handler: ClockSynchronizationHandler):
    show_clock, _ = clocks
    show_clock.start_time = time()

    assert show_clock.running
    handler.secondary_clock = show_clock
    assert handler.secondary_clock is show_clock
    handler.disable_and_stop()
    assert not handler.enabled
    assert not show_clock.running

    show_clock.start_time = time()

    assert show_clock.running
    handler.secondary_clock = show_clock
    handler.disable_and_stop()
    assert not handler.enabled
    assert not show_clock.running


def test_synchronization_basic(clocks: ClockPair, handler: ClockSynchronizationHandler):
    primary, secondary = clocks

    now = time()
    primary.start_time = now

    assert primary.running
    assert not secondary.running

    handler.secondary_clock = secondary
    assert not secondary.running

    handler.synchronize_to(primary, 10)
    assert secondary.running
    assert secondary.start_time == primary.start_time + 10
    assert secondary.ticks_given_time(now + 10) == 0
    assert secondary.ticks_given_time(now) == -10 * secondary.ticks_per_second


def test_sync_stop_start(clocks: ClockPair, handler: ClockSynchronizationHandler):
    primary, secondary = clocks

    now = time()
    primary.start_time = now
    handler.secondary_clock = secondary

    handler.synchronize_to(primary, 10)
    assert secondary.running
    assert secondary.ticks_given_time(now + 10) == 0

    primary.start_time = None
    assert not secondary.running
    assert secondary.start_time is None

    primary.start_time = now - 5
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 0

    primary.start_time = now + 3
    assert secondary.running
    assert secondary.ticks_given_time(now) == -13 * secondary.ticks_per_second


def test_detach_primary_clock(clocks: ClockPair, handler: ClockSynchronizationHandler):
    primary, secondary = clocks

    now = time()
    primary.start_time = now
    handler.secondary_clock = secondary

    handler.synchronize_to(primary, 10)
    assert handler.primary_clock is primary
    assert secondary.running

    handler.disable_and_stop()
    assert handler.primary_clock is None
    assert not secondary.running
    assert secondary.ticks_given_time(now) == 0

    primary.start_time = now - 5
    assert not secondary.running
    assert secondary.ticks_given_time(now) == 0


def test_detach_primary_clock_without_stopping_secondary_clock(
    clocks: ClockPair, handler: ClockSynchronizationHandler
):
    primary, secondary = clocks

    now = time()
    primary.start_time = now
    handler.secondary_clock = secondary

    handler.synchronize_to(primary, 10)
    assert handler.primary_clock is primary
    assert secondary.running

    handler.disable()
    assert handler.primary_clock is None
    assert secondary.running
    assert secondary.ticks_given_time(now) == -10 * secondary.ticks_per_second


def test_secondary_clock_context_manager(
    clocks: ClockPair, handler: ClockSynchronizationHandler
):
    assert handler.secondary_clock is None

    with handler.use_secondary_clock(clocks[1]):
        assert handler.secondary_clock is clocks[1]

    assert handler.secondary_clock is None

    try:
        with handler.use_secondary_clock(clocks[1]):
            assert handler.secondary_clock is clocks[1]
            raise RuntimeError("foo")
    except RuntimeError:
        pass

    assert handler.secondary_clock is None

    with handler.use_secondary_clock(clocks[1]):
        with raises(RuntimeError):
            with handler.use_secondary_clock(clocks[0]):
                pass


class MockLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))

    def warning(self, message):
        self.messages.append(("warning", message))


def test_clock_adjustment_beyond_point_of_no_return(
    clocks: ClockPair, handler_with_point_of_no_return: ClockSynchronizationHandler
):
    handler = handler_with_point_of_no_return
    primary, secondary = clocks

    mock_logger = MockLogger()
    notifications = []

    now = time()
    primary.start_time = now

    # Configure the handler
    handler.secondary_clock = secondary
    handler.log = mock_logger  # type: ignore
    handler.notify = notifications.append
    handler.synchronize_to(primary, 5)
    assert secondary.running
    assert secondary.ticks_given_time(now) == -50
    assert secondary.ticks_given_time(now + 5) == 0
    # A start 5 s away is already too late for the drones
    [(level, message)] = mock_logger.messages
    assert level == "warning" and "only 5.0 s before" in message
    mock_logger.messages.clear()

    # Now simulate an adjustment to the primary clock
    primary.start_time = now - 10
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 0
    assert len(mock_logger.messages) == 1
    assert mock_logger.messages[0][0] == "warning"
    mock_logger.messages.clear()

    # Set the primary clock back to its original start time
    primary.start_time = now
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 0
    assert len(mock_logger.messages) == 1
    assert mock_logger.messages[0][0] == "warning"
    mock_logger.messages.clear()

    # Stop the primary clock -- the secondary clock freewheels
    primary.start_time = None
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 0
    assert len(mock_logger.messages) == 1
    assert mock_logger.messages[0][0] == "warning"
    assert "stopped" in notifications[-1]

    # Synchronizing again to the same start keeps the secondary clock untouched
    handler.synchronize_to(primary, 5)
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 0

    # A different start is an explicit decision and applies
    primary.start_time = now - 10
    handler.synchronize_to(primary, 6)
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 90

    # Disable the synchronization, then adjust the primary clock again
    handler.disable()
    primary.start_time = now
    assert secondary.running
    assert secondary.ticks_given_time(now + 5) == 90


def test_primary_clock_stop_before_point_of_no_return(
    clocks: ClockPair, handler_with_point_of_no_return: ClockSynchronizationHandler
):
    handler = handler_with_point_of_no_return
    primary, secondary = clocks

    now = time()
    primary.start_time = now
    handler.secondary_clock = secondary
    handler.synchronize_to(primary, 60)
    assert secondary.running

    primary.start_time = None
    assert not secondary.running


def test_primary_clock_followed_again_after_end_of_show(
    clocks: ClockPair, handler_with_point_of_no_return: ClockSynchronizationHandler
):
    handler = handler_with_point_of_no_return
    primary, secondary = clocks

    now = time()
    primary.start_time = now
    handler.secondary_clock = secondary
    handler.end_of_show_seconds = 30
    handler.synchronize_to(primary, -35)
    assert secondary.ticks_given_time(now) == 350

    # Past the end of the show the secondary clock follows the primary clock
    # again, e.g. when the timecode is rewound for another run
    primary.start_time = now + 60
    assert secondary.ticks_given_time(now) == -250


def test_show_keeps_freewheeling_when_the_timecode_clock_is_gone():
    """The timecode extension is down after T-10 s when Control re-sends the
    unchanged configuration: the show clock keeps running. A different start
    or a cancellation still stops it."""
    registry = ClockRegistry()
    timecode = TimeElapsedSinceReferenceClock(id="mtc")
    timecode.reference_time = time() - 100
    registry.add(timecode)

    extension = DroneShowExtension()
    extension.log = logging.getLogger("test")
    extension.app = SimpleNamespace(  # type: ignore
        import_api=lambda name: SimpleNamespace(registry=registry)
    )
    show_clock = ShowClock()
    extension._clock = show_clock
    extension._clock_sync.point_of_no_return_seconds = -10
    extension._clock_sync.secondary_clock = show_clock

    extension._sync_show_clocks_to("mtc", 105, 60)
    start_time = show_clock.start_time
    assert start_time is not None

    registry.remove(timecode)
    extension._sync_show_clocks_to("mtc", 105, 60)
    assert show_clock.running
    assert show_clock.start_time == start_time

    extension._sync_show_clocks_to("mtc", 110, 60)
    assert not show_clock.running

    registry.add(timecode)
    extension._sync_show_clocks_to("mtc", 105, 60)
    registry.remove(timecode)
    extension._sync_show_clocks_to("mtc", None, 60)
    assert not show_clock.running


def test_start_known_too_late_through_the_primary_clock_is_reported(
    clocks: ClockPair, handler: ClockSynchronizationHandler
):
    primary, secondary = clocks
    messages = []
    handler.notify = messages.append
    handler.secondary_clock = secondary
    handler.synchronize_to(primary, 100)
    assert not secondary.running

    # Timecode starts rolling at 70 s: the start is 30 s away
    primary.start_time = time() - 70
    assert secondary.running
    assert messages == []

    primary.start_time = None
    assert not secondary.running

    # Timecode starts rolling at 95 s: the start is 5 s away
    primary.start_time = time() - 95
    assert secondary.running
    [message] = messages
    assert "only 5.0 s before" in message
