"""Focused tests for the pure four-tripod fit models."""

from math import cos, radians, sin

from flockwave.server.ext.rtls.anchor_geometry import (
    RangeObservation,
    fit_refined,
    fit_strict,
)


def _observations(positions, *, mad=0.008, count=80):
    return [
        RangeObservation(
            anchor_index=index,
            peer_mac=0x1000 + index,
            distance_m=sum(value * value for value in positions[index]) ** 0.5,
            mad_m=mad,
            count=count,
        )
        for index in range(1, 8)
    ]


def _strict_positions(length=20.0, width=16.0, height=2.5):
    return [
        (0.0, 0.0, 0.0),
        (length, 0.0, 0.0),
        (0.0, width, 0.0),
        (length, width, 0.0),
        (0.0, 0.0, -height),
        (length, 0.0, -height),
        (0.0, width, -height),
        (length, width, -height),
    ]


def _refined_positions(
    bottom_length=20.0,
    bottom_width=16.0,
    top_length=20.2,
    top_width=16.2,
    height=2.5,
    angle_deg=92.0,
):
    c, s = cos(radians(angle_deg)), sin(radians(angle_deg))
    return [
        (0.0, 0.0, 0.0),
        (bottom_length, 0.0, 0.0),
        (bottom_width * c, bottom_width * s, 0.0),
        (bottom_length + bottom_width * c, bottom_width * s, 0.0),
        (0.0, 0.0, -height),
        (top_length, 0.0, -height),
        (top_width * c, top_width * s, -height),
        (top_length + top_width * c, top_width * s, -height),
    ]


def test_strict_fit_recovers_absolute_canonical_geometry():
    result = fit_strict(_observations(_strict_positions()))

    assert result.accepted
    assert abs(result.parameters["lengthM"] - 20.0) < 1e-3
    assert abs(result.parameters["widthM"] - 16.0) < 1e-3
    assert abs(result.parameters["heightM"] - 2.5) < 1e-3
    # the canonical expansion places the far top corner from the fitted dims
    # (A0 is trivially the origin, so it is not worth asserting)
    assert abs(result.anchors[7]["xM"] - 20.0) < 1e-3
    assert result.anchors[7]["zM"] == -2.5


def test_refined_fit_explains_small_real_installation_deformation():
    observations = _observations(_refined_positions(), mad=0.005)
    strict = fit_strict(observations)
    refined = fit_refined(observations, strict=strict)

    assert refined.accepted, refined.reasons
    assert refined.rms_m < strict.rms_m
    assert abs(refined.parameters["topLengthM"] - 20.2) < 0.08
    assert abs(refined.parameters["topWidthM"] - 16.2) < 0.08
    assert abs(refined.parameters["angleDeg"] - 92.0) < 0.5
    assert refined.anchors[0]["xM"] == 0.0
    assert refined.anchors[4]["xM"] == 0.0


def test_refined_fit_does_not_claim_an_unneeded_improvement():
    observations = _observations(_strict_positions())
    strict = fit_strict(observations)
    refined = fit_refined(observations, strict=strict)

    assert strict.accepted
    assert not refined.accepted
    assert "noise floor" in refined.reasons[0]


def test_inconsistent_spoke_is_rejected_instead_of_hidden():
    observations = _observations(_strict_positions())
    observations[-1] = RangeObservation(
        anchor_index=7,
        peer_mac=observations[-1].peer_mac,
        distance_m=observations[-1].distance_m + 1.0,
        mad_m=0.005,
        count=80,
    )

    result = fit_strict(observations)

    assert not result.accepted
    assert "worst spoke residual" in result.reasons[0]


def test_strict_fit_recovers_dimensions_through_noise_and_an_outlier():
    """Deterministic per-spoke noise plus one biased spoke: the Huber loss
    must keep the dimension estimates on the tape-measured truth."""
    positions = _strict_positions()
    noise = (0.012, -0.009, 0.011, -0.007, 0.008, -0.012, 0.010)
    observations = [
        RangeObservation(
            anchor_index=index,
            peer_mac=0x1000 + index,
            distance_m=sum(v * v for v in positions[index]) ** 0.5
            + noise[index - 1]
            + (0.4 if index == 3 else 0.0),  # one multipath-biased spoke
            mad_m=0.010 if index != 3 else 0.080,
            count=80 if index != 3 else 25,
        )
        for index in range(1, 8)
    ]

    result = fit_strict(observations)

    # the biased diagonal fails the residual gate, so the result is not
    # offered for application -- but the estimate itself must stay sane
    assert abs(result.parameters["lengthM"] - 20.0) < 0.05
    assert abs(result.parameters["widthM"] - 16.0) < 0.05
    assert abs(result.parameters["heightM"] - 2.5) < 0.05
    assert not result.accepted
    assert any("residual" in reason for reason in result.reasons)


def test_excessive_deformation_is_rejected_not_hidden():
    """A 6-degree corner skew is outside the refined model's safety bounds:
    neither model may accept it, and the refined fit must not fabricate an
    in-bounds explanation that passes the residual gate."""
    observations = _observations(_refined_positions(angle_deg=96.0), mad=0.005)

    strict = fit_strict(observations)
    refined = fit_refined(observations, strict=strict)

    assert not strict.accepted
    assert not refined.accepted
    assert abs(refined.parameters["angleDeg"] - 90.0) <= 5.0 + 1e-6


def test_refined_rejection_names_the_safety_bound_it_reached():
    """The safety-bound check must be what rejects a bound-riding fit, not
    only the residual gate. A top plane materially larger than the bottom
    drives the upper/lower dimension difference to its clamp; the refined
    result must be rejected AND say which bound it hit -- deleting the
    bound-check block would leave this test failing even though the geometry
    is otherwise representable."""
    observations = _observations(
        _refined_positions(top_length=21.2, top_width=17.2), mad=0.004
    )

    refined = fit_refined(observations)

    assert not refined.accepted
    assert any("difference" in reason and "bound" in reason for reason in refined.reasons)


def test_height_rides_the_a0_to_a4_spoke_and_a_bias_is_not_caught():
    """Bench-critical characterization: fitted H rests almost entirely on the
    same-tripod A0->A4 vertical spoke (which measures height directly), so a
    bias on it flows into heightM while the diagonal spokes keep the worst
    residual well under the 0.15 m gate. This pins the documented limitation
    -- the bench must tape-measure H, not trust the fit here. If a future
    change made this spoke's bias detectable, this test would flag it."""
    positions = _strict_positions()
    bias = 0.15
    observations = [
        RangeObservation(
            anchor_index=index,
            peer_mac=0x1000 + index,
            distance_m=sum(v * v for v in positions[index]) ** 0.5
            + (bias if index == 4 else 0.0),  # A0->A4 measures H directly
            mad_m=0.004,
            count=80,
        )
        for index in range(1, 8)
    ]

    result = fit_strict(observations)

    # the bias rides into H (the spoke dominates it) while L and W are clean
    assert result.parameters["heightM"] > 2.5 + 0.5 * bias
    assert abs(result.parameters["lengthM"] - 20.0) < 0.02
    assert abs(result.parameters["widthM"] - 16.0) < 0.02
    # and the residual gate does NOT catch it -- the whole point of the caveat
    worst = max(abs(item["residualM"]) for item in result.residuals)
    assert worst < 0.15
    assert result.accepted
