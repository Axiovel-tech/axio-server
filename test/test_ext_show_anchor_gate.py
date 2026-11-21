"""The show start gate on the anchor cell lock: SHOW-SETCFG refuses to
authorize a flight, or to move an authorized start, while the rtls
extension says the anchors are not locked."""

from time import time
from types import SimpleNamespace

from flockwave.server.ext.show.extension import DroneShowExtension

hub = SimpleNamespace(
    acknowledge=lambda message, outcome=True, reason=None: (outcome, reason)
)

REFUSAL = "the anchors are not locked; lock them in Control first"


def show(refusal=None, *, rtls=True):
    """A show extension whose rtls extension answers ``refusal``."""
    extension = DroneShowExtension()
    api = SimpleNamespace(loaded=True, show_start_refusal=lambda: refusal)

    def import_api(name):
        if name != "rtls" or not rtls:
            raise KeyError(name)
        return api

    extension.app = SimpleNamespace(import_api=import_api)
    return extension


def setcfg(extension, **start):
    message = SimpleNamespace(body={"configuration": {"start": start}})
    return extension.handle_SHOW_SETCFG(message, None, hub)


def test_authorizing_a_flight_needs_locked_anchors():
    extension = show(REFUSAL)
    start = time() + 600

    for scope in ("live", "rehearsal"):
        outcome, reason = setcfg(
            extension, time=start, authorized=True, authorizationScope=scope
        )
        assert not outcome
        assert reason == f"Show start refused: {REFUSAL}"
        assert not extension._config.authorized_to_start

    # scheduling an unauthorized start, and lights only, are not refused
    assert setcfg(extension, time=start, authorized=False) == (True, None)
    assert setcfg(extension, authorized=True, authorizationScope="lights") == (
        True,
        None,
    )


def test_an_authorized_show_is_never_withheld():
    extension = show()
    start = time() + 600
    assert setcfg(
        extension, time=start, method="auto", authorized=True, authorizationScope="live"
    ) == (True, None)

    # the lock breaks after the authorization
    extension.app = show(REFUSAL).app
    # unchanged resends, revoking and clearing the start pass...
    assert setcfg(extension, time=start, authorized=True) == (True, None)
    assert setcfg(extension, time=None) == (True, None)
    assert extension._config.authorized_to_start
    # ...but a new start of the authorized show is refused
    outcome, reason = setcfg(extension, time=start + 60)
    assert not outcome and REFUSAL in reason
    assert setcfg(extension, authorized=False) == (True, None)


def test_without_an_rtls_cell_there_is_no_requirement():
    for extension in (show(None), show(REFUSAL, rtls=False)):
        assert setcfg(
            extension, time=time() + 600, authorized=True, authorizationScope="live"
        ) == (True, None)


def test_an_abort_switching_to_rc_is_never_refused():
    extension = show()
    start = time() + 600
    assert setcfg(
        extension, time=start, method="auto", authorized=True, authorizationScope="live"
    ) == (True, None)
    extension.app = show(REFUSAL).app

    # clearing the AUTO start and switching to RC in one update is an abort
    assert setcfg(extension, time=None, method="rc") == (True, None)
    assert extension._config.start_time_on_clock is None
    assert extension._config.start_method.value == "rc"
