"""PRM-SET on a MAVLink UAV: only a PARAM_VALUE carrying the written value
confirms the write, and the caller gets that value back or an error."""

from types import SimpleNamespace

import pytest
from trio import TooSlowError

from flockwave.server.ext.mavlink.driver import MAVLinkDriver, MAVLinkUAV

NAME = "MAV3_OPTIONS"
INT8 = 2


def param_value(value: float, name: str = NAME) -> SimpleNamespace:
    return SimpleNamespace(param_id=name, param_value=value, param_type=INT8)


class FakeLink:
    """The FC side: ``replies[i]`` arrive after the i-th PARAM_SET attempt;
    ``reads`` answer the PARAM_REQUEST_READs in order."""

    def __init__(self, replies, reads, retries=3):
        self.replies = list(replies)
        self.reads = list(reads)
        self.retries = retries
        self.attempts = 0

    async def send_packet_with_retries(
        self, spec, target, *, wait_for_response, timeout
    ):
        response_type, matches = wait_for_response
        assert response_type == "PARAM_VALUE"
        for attempt in range(self.retries + 1):
            self.attempts += 1
            arrived = self.replies[attempt] if attempt < len(self.replies) else []
            for message in arrived:
                if matches(message):
                    return message
        raise TooSlowError

    async def read(self, name):
        return self.reads.pop(0)


def make_uav(link: FakeLink):
    uav = SimpleNamespace(
        driver=link,
        _get_parameter=link.read,
        _autopilot=SimpleNamespace(
            encode_param_to_wire_representation=lambda value, type: float(value),
            decode_param_from_wire_representation=lambda value, type: float(value),
        ),
    )
    uav._set_parameter_single = MAVLinkUAV._set_parameter_single.__get__(uav)
    return uav


async def set_parameter(link: FakeLink, value: float) -> float:
    uav = make_uav(link)
    return await MAVLinkUAV.set_parameter(uav, NAME, value)  # type: ignore[arg-type]


async def test_the_confirmed_value_is_returned():
    link = FakeLink(replies=[[param_value(2.0)]], reads=[param_value(0.0)])
    assert await set_parameter(link, 2.0) == 2.0


async def test_a_late_reply_to_the_read_does_not_confirm_a_lost_write():
    # The bench case: the old value arrived after the read had already been
    # answered, the PARAM_SET never reached the FC, and the set reported success.
    link = FakeLink(
        replies=[[param_value(0.0)]],
        reads=[param_value(0.0), param_value(0.0)],
    )
    with pytest.raises(RuntimeError, match="tried to set 2.0, got 0.0"):
        await set_parameter(link, 2.0)
    assert link.attempts == link.retries + 1


async def test_a_retried_write_that_lands_after_a_stale_reply_is_confirmed():
    link = FakeLink(
        replies=[[param_value(0.0)], [], [param_value(2.0)]],
        reads=[param_value(0.0)],
    )
    assert await set_parameter(link, 2.0) == 2.0
    assert link.attempts == 3


async def test_another_parameter_with_the_same_value_does_not_confirm():
    link = FakeLink(
        replies=[[param_value(2.0, name="MAV2_OPTIONS")]],
        reads=[param_value(0.0), param_value(0.0)],
    )
    with pytest.raises(RuntimeError, match="got 0.0"):
        await set_parameter(link, 2.0)


async def test_a_silent_autopilot_is_confirmed_by_reading_the_value_back():
    # ArduPilot with SERIALx_OPTIONS bit 10 sends no PARAM_VALUE for a set.
    link = FakeLink(replies=[], reads=[param_value(0.0), param_value(2.0)])
    assert await set_parameter(link, 2.0) == 2.0


async def test_a_value_the_autopilot_changed_is_an_error_with_that_value():
    link = FakeLink(replies=[], reads=[param_value(0.0), param_value(1.0)])
    with pytest.raises(RuntimeError, match="tried to set 2.0, got 1.0"):
        await set_parameter(link, 2.0)


async def test_a_float_parameter_is_compared_as_its_32_bit_wire_value():
    # pymavlink hands back the float32 0.1 as 0.10000000149011612.
    link = FakeLink(
        replies=[[param_value(0.10000000149011612)]], reads=[param_value(0.0)]
    )
    assert await set_parameter(link, 0.1) == pytest.approx(0.1)


async def test_a_value_outside_float32_is_refused_before_anything_is_sent():
    # Packed inside the shared receive loop it would raise there instead.
    link = FakeLink(replies=[], reads=[param_value(0.0)])
    with pytest.raises(RuntimeError, match="does not fit"):
        await set_parameter(link, 1e39)
    assert link.attempts == 0


async def test_the_driver_passes_the_confirmed_value_to_the_prm_set_reply():
    async def confirmed_set(name, value):
        return value

    uav = SimpleNamespace(set_parameter=confirmed_set)
    result = await MAVLinkDriver._set_parameter_single(None, uav, NAME, "2")  # type: ignore[arg-type]
    assert result == 2.0
