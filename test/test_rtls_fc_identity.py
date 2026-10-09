import pytest

from flockwave.server.ext.rtls.fc_identity import flight_controller_claims

LIVE, REMEMBERED, AMBIGUOUS = 1, 2, 3


def claims_of(*devices, sleeping=False):
    return flight_controller_claims(
        {
            tag: ({"FC_SYS_ID": identity, "FC_STATE": state}, sleeping)
            for tag, identity, state in devices
        }
    )


@pytest.mark.parametrize(
    ("state", "expected"), [(LIVE, "live"), (REMEMBERED, "remembered")]
)
def test_associated_tag_reports_its_flight_controller(state, expected):
    assert claims_of((200, 11, state), (201, 255, state)) == {
        200: {"systemId": 11, "state": expected},
        201: {"systemId": 255, "state": expected},
    }


@pytest.mark.parametrize("state", [LIVE, REMEMBERED])
def test_sleeping_tag_reports_the_identity_as_remembered(state):
    assert claims_of((200, 11, state), sleeping=True) == {
        200: {"systemId": 11, "state": "remembered"}
    }


@pytest.mark.parametrize(
    "params",
    [
        {},
        {"FC_SYS_ID": 11},
        {"FC_STATE": LIVE},
        {"FC_SYS_ID": 0, "FC_STATE": 0},
    ],
)
def test_tag_without_identity_has_no_claim(params):
    assert flight_controller_claims({200: (params, False)}) == {}


def test_tag_hearing_conflicting_heartbeats_is_ambiguous():
    assert claims_of((200, 11, AMBIGUOUS), (201, 0, AMBIGUOUS)) == {
        200: {
            "systemId": 11,
            "state": "ambiguous",
            "reason": "the tag hears conflicting flight-controller heartbeats",
        },
        201: {
            "state": "ambiguous",
            "reason": "the tag hears conflicting flight-controller heartbeats",
        },
    }


@pytest.mark.parametrize(
    ("identity", "state"),
    [
        (0, LIVE),
        (0, REMEMBERED),
        (11, 0),
        (11, 7),
        (256, LIVE),
        (-1, LIVE),
        ("11", LIVE),
    ],
)
def test_inconsistent_identity_is_ambiguous_without_an_id(identity, state):
    assert claims_of((200, identity, state)) == {
        200: {"state": "ambiguous", "reason": "inconsistent flight-controller identity"}
    }


def test_shared_identity_goes_to_the_tag_that_hears_it_live():
    claims = claims_of((220, 19, REMEMBERED), (222, 17, LIVE), (199, 17, REMEMBERED))

    assert claims[222] == {"systemId": 17, "state": "live"}
    assert claims[199] == {
        "systemId": 17,
        "state": "ambiguous",
        "reason": "flight controller 17 is also claimed by tag 222",
    }
    assert claims[220] == {"systemId": 19, "state": "remembered"}


@pytest.mark.parametrize("state", [LIVE, REMEMBERED])
def test_shared_identity_without_a_single_live_holder_is_ambiguous_everywhere(
    state,
):
    claims = claims_of((222, 17, state), (199, 17, state), (250, 17, REMEMBERED))

    assert {tag: claim["state"] for tag, claim in claims.items()} == {
        222: "ambiguous",
        199: "ambiguous",
        250: "ambiguous",
    }
    assert claims[199]["reason"] == (
        "flight controller 17 is also claimed by tags 222, 250"
    )
