"""Tests for the certified `.gdm` RTH request (``X-SHOW-RTH``)."""

from collections import defaultdict
from logging import getLogger
from time import time

from pytest import fixture, mark, raises

from flockwave.server.app import SkybrushServer
from flockwave.server.ext.show.clock import ShowClock
from flockwave.server.ext.show.extension import DroneShowExtension
from flockwave.server.ext.show.rth import (
    RTHPlan,
    RTHPlanRegistry,
    RTHRequestRefused,
    first_branch_time_at_or_after,
    plan_rth_dispatch,
)
from flockwave.server.message_hub import MessageHub
from flockwave.server.model.messages import FlockwaveMessage
from flockwave.server.model.uav import UAVBase, UAVDriver

TIMES_US = (2_000_000, 4_000_000, 6_000_000, 8_000_000)


def plan(show_hash: str = "show-a", times_us: tuple[int, ...] = TIMES_US) -> RTHPlan:
    return RTHPlan(show_hash=show_hash, times_us=times_us)


@fixture
def registry() -> RTHPlanRegistry:
    registry = RTHPlanRegistry()
    for uav_id in ("01", "02", "03"):
        registry.set(uav_id, plan())
    return registry


class TestRTHPlan:
    def test_rejects_empty_times(self):
        with raises(ValueError):
            plan(times_us=())

    def test_rejects_unsorted_or_repeated_times(self):
        with raises(ValueError):
            plan(times_us=(2_000_000, 2_000_000))
        with raises(ValueError):
            plan(times_us=(4_000_000, 2_000_000))

    def test_rejects_negative_times(self):
        with raises(ValueError):
            plan(times_us=(-1, 2_000_000))


class TestFirstBranchTime:
    def test_exact_match_is_taken(self):
        assert first_branch_time_at_or_after(TIMES_US, 4_000_000) == 4_000_000

    def test_next_time_is_taken(self):
        assert first_branch_time_at_or_after(TIMES_US, 4_000_001) == 6_000_000

    def test_none_after_last(self):
        assert first_branch_time_at_or_after(TIMES_US, 8_000_001) is None


class TestPlanRTHDispatch:
    def test_picks_first_branch_time_at_least_latency_ahead(self, registry):
        dispatch = plan_rth_dispatch(
            registry, ["01", "02"], show_time_us=3_100_000, latency_us=1_000_000
        )
        # 3.1 s + 1 s = 4.1 s, so the 4 s branch is too close.
        assert dispatch.branch_time_us == 6_000_000
        assert dispatch.uav_ids == ["01", "02"]
        assert dispatch.errors == {}

    def test_branch_time_exactly_latency_ahead_is_allowed(self, registry):
        dispatch = plan_rth_dispatch(
            registry, ["01"], show_time_us=3_000_000, latency_us=1_000_000
        )
        assert dispatch.branch_time_us == 4_000_000

    def test_drone_without_plan_is_reported_and_others_still_return(self, registry):
        dispatch = plan_rth_dispatch(
            registry, ["01", "99", "02"], show_time_us=0, latency_us=1_000_000
        )
        assert dispatch.uav_ids == ["01", "02"]
        assert set(dispatch.errors) == {"99"}

    def test_duplicates_are_commanded_once(self, registry):
        dispatch = plan_rth_dispatch(
            registry, ["01", "01"], show_time_us=0, latency_us=0
        )
        assert dispatch.uav_ids == ["01"]

    def test_refuses_empty_selection(self, registry):
        with raises(RTHRequestRefused, match="No drones"):
            plan_rth_dispatch(registry, [], show_time_us=0, latency_us=0)

    @mark.parametrize("show_time_us", [None, -1_000_000])
    def test_refuses_when_show_not_running(self, registry, show_time_us):
        with raises(RTHRequestRefused, match="not running"):
            plan_rth_dispatch(registry, ["01"], show_time_us=show_time_us, latency_us=0)

    def test_refuses_when_no_selected_drone_holds_a_plan(self, registry):
        with raises(RTHRequestRefused, match="certified RTH plan"):
            plan_rth_dispatch(registry, ["99"], show_time_us=0, latency_us=0)

    def test_refuses_when_no_branch_time_remains(self, registry):
        with raises(RTHRequestRefused, match="1000 ms ahead"):
            plan_rth_dispatch(
                registry, ["01"], show_time_us=7_500_000, latency_us=1_000_000
            )

    def test_refuses_mixed_shows(self, registry):
        registry.set("03", plan(show_hash="show-b"))
        with raises(RTHRequestRefused, match="different shows"):
            plan_rth_dispatch(registry, ["01", "03"], show_time_us=0, latency_us=0)


# --- Message handler ---------------------------------------------------------


class RecordingDriver(UAVDriver):
    """Driver whose vehicles accept every show RTH command."""

    def __init__(self) -> None:
        super().__init__()
        self.sent: list[tuple[str, int]] = []

    def _send_show_rth_signal_single(self, uav, *, branch_time_us: int) -> None:
        self.sent.append((uav.id, branch_time_us))


class FakeApp:
    """The parts of the server application that the RTH handler uses."""

    def __init__(self, uavs: dict[str, UAVBase]) -> None:
        self.uavs = uavs

    def sort_uavs_by_drivers(self, uav_ids, response=None):
        result = defaultdict(list)
        for uav_id in uav_ids:
            uav = self.uavs.get(uav_id)
            if uav is None:
                if response is not None:
                    response.add_error(uav_id, "No such UAV")
            else:
                result[uav.driver].append(uav)
        return result

    async def dispatch_driver_method(self, *args):
        # The real implementation; the fake only lacks the command manager,
        # which synchronous driver results never touch.
        return await SkybrushServer.dispatch_driver_method(self, *args)  # type: ignore


@fixture
def driver() -> RecordingDriver:
    return RecordingDriver()


@fixture
def extension(driver) -> DroneShowExtension:
    unsupported = UAVDriver()
    uavs = {
        "01": UAVBase("01", driver),
        "02": UAVBase("02", driver),
        "03": UAVBase("03", driver),
        "04": UAVBase("04", unsupported),
    }
    ext = DroneShowExtension()
    ext.app = FakeApp(uavs)  # type: ignore
    ext.log = getLogger("test.show.rth")
    ext._clock = ShowClock()
    for uav_id in ("01", "02", "03", "04"):
        ext._set_rth_plan(uav_id, "show-a", list(TIMES_US))
    return ext


def start_show_clock(ext: DroneShowExtension, show_time_s: float) -> None:
    assert ext._clock is not None
    ext._clock.start_time = time() - show_time_s


async def request(ext: DroneShowExtension, hub: MessageHub, body: dict):
    message = FlockwaveMessage.from_json(  # type: ignore
        {"$fw.version": "1.0", "id": "req-1", "body": {"type": "X-SHOW-RTH", **body}},
        validate=False,
    )
    return await ext.handle_SHOW_RTH(message, None, hub)


async def test_handler_sends_selected_drones_at_the_chosen_branch_time(
    extension, driver
):
    start_show_clock(extension, 2.5)
    response = await request(extension, MessageHub(), {"ids": ["01", "03"]})

    body = response.body
    assert body["type"] == "X-SHOW-RTH"
    assert body["branchTimeUs"] == 4_000_000
    assert driver.sent == [("01", 4_000_000), ("03", 4_000_000)]
    assert "error" not in body


async def test_handler_return_all_uses_show_assignment(extension, driver):
    start_show_clock(extension, 0.5)
    extension._config.uav_ids = ["01", None, "02"]
    response = await request(extension, MessageHub(), {"all": True})

    assert response.body["branchTimeUs"] == 2_000_000
    assert driver.sent == [("01", 2_000_000), ("02", 2_000_000)]


async def test_handler_return_all_without_assignment_uses_plan_holders(
    extension, driver
):
    start_show_clock(extension, 0.5)
    response = await request(extension, MessageHub(), {"all": True})

    assert [uav_id for uav_id, _ in driver.sent] == ["01", "02", "03"]
    # The firmware side is not implemented yet: reported per drone.
    assert "not implemented" in response.body["error"]["04"]


async def test_handler_reports_drones_without_plan_and_unknown_drones(
    extension, driver
):
    start_show_clock(extension, 0.5)
    extension._rth_plans.clear("02")
    response = await request(extension, MessageHub(), {"ids": ["01", "02", "77"]})

    assert driver.sent == [("01", 2_000_000)]
    assert set(response.body["error"]) == {"02", "77"}


async def test_handler_refuses_when_show_not_running(extension, driver):
    response = await request(extension, MessageHub(), {"ids": ["01"]})

    assert response.body["type"] == "ACK-NAK"
    assert "not running" in response.body["reason"]
    assert driver.sent == []


async def test_handler_refuses_when_no_branch_time_remains(extension, driver):
    start_show_clock(extension, 7.5)
    response = await request(extension, MessageHub(), {"all": True})

    assert response.body["type"] == "ACK-NAK"
    assert driver.sent == []


@mark.parametrize(
    "body", [{}, {"ids": ["01"], "all": True}, {"all": False}, {"ids": "01"}]
)
async def test_handler_refuses_malformed_requests(extension, driver, body):
    start_show_clock(extension, 0.5)
    response = await request(extension, MessageHub(), body)

    assert response.body["type"] == "ACK-NAK"
    assert driver.sent == []


def test_info_lists_plan_holders(extension):
    message = FlockwaveMessage.from_json(  # type: ignore
        {"$fw.version": "1.0", "id": "req-2", "body": {"type": "X-SHOW-RTH-INF"}},
        validate=False,
    )
    response = extension.handle_SHOW_RTH_INF(message, None, MessageHub())
    assert response.body["ids"] == ["01", "02", "03", "04"]
