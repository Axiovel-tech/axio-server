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

    # Switching to the RC start would stop the music but not the drones
    outcome, _ = setcfg(extension, {"time": start_time, "method": "rc"})
    assert not outcome
    assert extension._config.start_method.value == "auto"

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


def test_the_server_cancels_a_start_as_a_client_would_and_tells_the_clients():
    import logging

    from flockwave.server.model.log import Severity

    start_time = time() + 15
    extension = show_starting_at(start_time)
    extension._config.update_from_json(
        {"start": {"authorized": True, "authorizationScope": "live"}}
    )
    messages, notifications, signals = [], [], []
    extension.log = logging.getLogger("test_ext_show_start_lock")
    extension.app = SimpleNamespace(  # type: ignore[assignment]
        request_to_send_SYS_MSG_message=lambda message, severity: messages.append(
            (message, severity)
        ),
        import_api=lambda name: SimpleNamespace(
            get=lambda signal: SimpleNamespace(
                send=lambda sender, config: signals.append((signal, config))
            )
        ),
        message_hub=SimpleNamespace(
            create_notification=lambda body: body,
            enqueue_broadcast_message=notifications.append,
        ),
    )
    extension._config.updated.connect(
        extension._on_config_updated, sender=extension._config
    )

    extension._cancel_start("drone 3 is not connected")

    config = extension._config
    assert config.start_time_on_clock is None
    assert not config.authorized_to_start
    assert config.authorization_scope.value == "none"
    assert messages == [("Start cancelled: drone 3 is not connected", Severity.ERROR)]
    [(signal, sent)] = signals
    assert signal == "show:config_updated"
    assert sent.start_time_on_clock is None
    [notification] = notifications
    assert notification["type"] == "X-SHOW-CFG"
    assert notification["configuration"]["start"]["time"] is None
    assert notification["configuration"]["start"]["authorized"] is False
