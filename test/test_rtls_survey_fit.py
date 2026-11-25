"""The survey fit and its commit gate, on synthetic layouts."""

import json
import re
from itertools import combinations
from math import dist

import numpy as np
import pytest

from flockwave.server.ext.rtls.survey_fit import (
    START_HEIGHT_M,
    DirectedRange,
    Frame,
    SurveyFitError,
    _distinct,
    _pair_ranges,
    _Problem,
    _Solution,
    fit_survey,
    frame_refusal,
)

NOISE_M = 0.02

TWO_TIERS = {
    101: (0.0, 0.0, 0.0),
    102: (6.4, 0.0, 0.0),
    103: (0.2, 4.8, 0.0),
    104: (6.1, 5.0, 0.0),
    105: (0.1, -0.1, -2.7),
    106: (6.3, 0.2, -2.7),
    107: (0.3, 4.9, -2.7),
    108: (6.2, 4.7, -2.7),
}
TWO_TIERS_FRAME = Frame.two_tiers(list(TWO_TIERS))
TWO_TIERS_NO_SIDE = Frame(
    origin=101,
    x_axis=102,
    height_groups=((101, 102, 103, 104), (105, 106, 107, 108)),
    upper=(105, 106, 107, 108),
)

L_SHAPE = {
    201: (0.0, 0.0, 0.0),
    202: (7.0, 0.0, 0.0),
    203: (7.1, 3.0, 0.0),
    204: (3.0, 3.2, 0.0),
    205: (2.8, 6.0, 0.0),
    206: (0.1, 6.1, 0.0),
}
L_SHAPE_FRAME = Frame(origin=201, x_axis=202, height_groups=(tuple(L_SHAPE),))

FIVE = {
    301: (0.0, 0.0, 0.0),
    302: (5.0, 0.0, 0.0),
    303: (0.5, 4.0, 0.0),
    304: (4.6, 4.4, -2.2),
    305: (1.0, 1.5, -2.2),
}
FIVE_FRAME = Frame(
    origin=301,
    x_axis=302,
    height_groups=((301, 302, 303), (304, 305)),
    upper=(304, 305),
)
FIVE_FREE_HEIGHTS = Frame(
    origin=301, x_axis=302, height_groups=((301, 302, 303),), upper=(304,)
)

TRIANGLE = {1: (0.0, 0.0, 0.0), 2: (4.0, 0.0, 0.0), 3: (0.0, 3.0, 0.0)}
TRIANGLE_FRAME = Frame(origin=1, x_axis=2, height_groups=((1, 2, 3),))


def survey(layout, seed=0, noise=NOISE_M, drop=()):
    """Every pair of the layout ranged in both directions, except ``drop``.
    As on real anchors, most of a pair's error is common to its two
    directions."""
    rng = np.random.default_rng(seed)
    measurements = {}
    for a, b in combinations(layout, 2):
        if (a, b) in drop or (b, a) in drop:
            continue
        true = float(np.linalg.norm(np.subtract(layout[a], layout[b])))
        common = rng.normal(0.0, noise)
        for key in ((a, b), (b, a)):
            measurements[key] = DirectedRange(
                range_m=true + common + rng.normal(0.0, noise / 4),
                mad_m=0.005,
                inliers=96,
                share=0.96,
            )
    return measurements


def read_to_the_centimetre(rows):
    """Anchors 1, 2, ... at the rows, every pair ranged both ways without
    noise, as the centimetres an anchor reports."""
    layout = dict(enumerate(rows, 1))
    measurements = {
        (a, b): DirectedRange(round(dist(layout[a], layout[b]), 2), 0.01, 95, 0.95)
        for a in layout
        for b in layout
        if a != b
    }
    return layout, measurements


def rough(layout, seed=0, spread=0.5):
    """A top-down sketch: x and y only, off by about ``spread`` metres."""
    rng = np.random.default_rng(1000 + seed)
    return {
        a: (x + rng.normal(0, spread), y + rng.normal(0, spread))
        for a, (x, y, _) in layout.items()
    }


def mirrored(layout, x=1.0, y=1.0, z=1.0):
    return {a: (px * x, py * y, pz * z) for a, (px, py, pz) in layout.items()}


def worst_error(fit, layout):
    return max(
        float(np.linalg.norm(np.subtract(fit.anchors[a].position, layout[a])))
        for a in layout
    )


def sigma_of(fit, anchor) -> float:
    sigma = fit.anchors[anchor].sigma_m
    assert sigma is not None
    return sigma


def offset_of(fit, anchor) -> float:
    offset = fit.anchors[anchor].height_offset_m
    assert offset is not None
    return offset


def moved_of(fit, anchor) -> float:
    moved = fit.anchors[anchor].moved_m
    assert moved is not None
    return moved


def codes(entries):
    return [(entry.code, entry.device, entry.peer) for entry in entries]


def test_two_tiers_preset_needs_no_placement():
    fit = fit_survey(list(TWO_TIERS), survey(TWO_TIERS), TWO_TIERS_FRAME)

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert 0.5 * NOISE_M < fit.rms_m < 1.5 * NOISE_M
    assert fit.rms_m <= fit.worst_m < 3 * NOISE_M
    assert not fit.planar
    assert fit.blocked == () and fit.warnings == () and fit.notes == ()
    assert [report.links for report in fit.anchors.values()] == [7] * 8
    # two groups: every anchor has its own height, near its group's
    offsets = [offset_of(fit, a) for a in TWO_TIERS]
    assert len(set(offsets)) == 8 and max(map(abs, offsets)) < 0.05
    assert len(fit.pairs) == 28


def test_two_tiers_preset_takes_a_single_tier_and_refuses_two_anchors():
    assert Frame.two_tiers([1, 2, 3, 4]) == Frame(
        origin=1, x_axis=2, height_groups=((1, 2, 3, 4),), upper=(), y_positive=3
    )
    with pytest.raises(SurveyFitError, match="at least three"):
        Frame.two_tiers([1, 2])


def test_l_shape_from_a_rough_top_down_placement():
    fit = fit_survey(
        list(L_SHAPE), survey(L_SHAPE), L_SHAPE_FRAME, placement=rough(L_SHAPE)
    )

    assert not re.search(r"-0\.0(?!\d)", json.dumps(fit.json))
    assert worst_error(fit, L_SHAPE) < 0.10
    assert fit.planar
    assert all(report.position[2] == 0.0 for report in fit.anchors.values())
    assert fit.blocked == () and fit.warnings == () and fit.notes == ()


def test_five_anchors_with_a_missing_pair_fit_and_block_the_commit():
    fit = fit_survey(
        list(FIVE),
        survey(FIVE, drop={(302, 305)}),
        FIVE_FRAME,
        placement=rough(FIVE),
    )

    assert worst_error(fit, FIVE) < 0.20
    assert codes(fit.blocked) == [("pair-missing", 302, 305)]
    assert fit.blocked[0].detail == "not measured"
    assert fit.anchors[302].links == fit.anchors[305].links == 3
    assert fit.anchors[301].links == 4
    assert len(fit.pairs) == 9


def test_preset_with_a_missing_pair_still_fits_without_a_placement():
    fit = fit_survey(
        list(TWO_TIERS), survey(TWO_TIERS, drop={(103, 106)}), TWO_TIERS_FRAME
    )

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert codes(fit.blocked) == [("pair-missing", 103, 106)]


def test_heights_missing_from_the_placement_come_from_the_ranges():
    flat_sketch = {a: (x, y) for a, (x, y, _) in FIVE.items()}

    fit = fit_survey(list(FIVE), survey(FIVE), FIVE_FRAME, placement=flat_sketch)

    assert worst_error(fit, FIVE) < 0.20
    assert fit.blocked == ()


@pytest.mark.parametrize(
    "mirror", [{"y": -1.0}, {"z": -1.0}, {"y": -1.0, "z": -1.0}], ids=str
)
def test_frame_settles_a_mirrored_placement(mirror):
    fit = fit_survey(
        list(TWO_TIERS),
        survey(TWO_TIERS),
        TWO_TIERS_FRAME,
        placement=mirrored(TWO_TIERS, **mirror),
    )

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert fit.blocked == ()


def test_placement_settles_the_mirror_the_frame_leaves_open():
    measurements = survey(TWO_TIERS)
    other_side = mirrored(TWO_TIERS, y=-1.0)

    as_placed = fit_survey(
        list(TWO_TIERS), measurements, TWO_TIERS_NO_SIDE, placement=rough(TWO_TIERS)
    )
    as_mirrored = fit_survey(
        list(TWO_TIERS), measurements, TWO_TIERS_NO_SIDE, placement=rough(other_side)
    )

    assert worst_error(as_placed, TWO_TIERS) < 0.08
    assert worst_error(as_mirrored, other_side) < 0.08
    assert as_placed.rms_m == pytest.approx(as_mirrored.rms_m, abs=1e-9)
    assert as_placed.blocked == as_mirrored.blocked == ()


def test_mirror_nothing_settles_blocks_the_anchors_it_moves():
    fit = fit_survey(list(TWO_TIERS), survey(TWO_TIERS), TWO_TIERS_NO_SIDE)

    assert codes(fit.blocked) == [
        ("not-located", anchor, None) for anchor in (103, 104, 105, 106, 107, 108)
    ]
    assert "neither the placement nor the frame" in fit.blocked[0].detail
    assert (
        min(worst_error(fit, TWO_TIERS), worst_error(fit, mirrored(TWO_TIERS, y=-1.0)))
        < 0.05
    )


@pytest.mark.parametrize(
    ("layout", "frame", "pair"),
    [
        (TWO_TIERS, TWO_TIERS_NO_SIDE, (103, 104)),
        (TWO_TIERS, TWO_TIERS_NO_SIDE, (102, 107)),
        (TWO_TIERS, TWO_TIERS_NO_SIDE, (105, 108)),
        (TWO_TIERS, TWO_TIERS_FRAME, (101, 104)),
        (L_SHAPE, L_SHAPE_FRAME, (201, 204)),
        (L_SHAPE, L_SHAPE_FRAME, (202, 205)),
        (L_SHAPE, L_SHAPE_FRAME, (203, 206)),
    ],
    ids=lambda value: str(value) if isinstance(value, tuple) else "",
)
def test_ranges_overrule_a_swapped_placement(layout, frame, pair):
    first, second = pair
    sketch = rough(layout, spread=0.3)
    sketch[first], sketch[second] = sketch[second], sketch[first]

    fit = fit_survey(list(layout), survey(layout), frame, placement=sketch)

    assert worst_error(fit, layout) < 0.10
    assert fit.notes == (
        f"anchors {first} and {second} are swapped against the placement",
    )
    assert fit.blocked == ()


def test_square_cell_follows_its_sketch():
    # The mirror image of a square is the square with two corners exchanged,
    # so only the sketch as drawn tells which one stands.
    square = {
        1: (0.0, 0.0, 0.0),
        2: (5.0, 0.0, 0.0),
        3: (0.0, 5.0, 0.0),
        4: (5.0, 5.0, 0.0),
    }
    frame = Frame(origin=1, x_axis=2, height_groups=((1, 2, 3, 4),))

    for seed in range(8):
        fit = fit_survey(
            list(square),
            survey(square, seed=seed),
            frame,
            placement=rough(square, seed=seed, spread=0.3),
        )

        assert worst_error(fit, square) < 0.15
        assert fit.notes == () and fit.blocked == ()


def test_sketch_drawn_flat_at_zero_height_still_finds_the_tiers():
    layout, measurements = read_to_the_centimetre(
        [(x, y, z) for z in (0.0, -2.0) for y in (0.0, 5.0) for x in (0.0, 6.0)]
    )
    flat = {a: (x, y, 0.0) for a, (x, y, _) in layout.items()}

    fit = fit_survey(
        list(layout), measurements, Frame.two_tiers(list(layout)), placement=flat
    )

    assert worst_error(fit, layout) < 0.02
    assert fit.rms_m < 0.01
    assert fit.blocked == ()


def test_flat_sketch_finds_a_height_group_the_frame_does_not_call_upper():
    two_heights = Frame(
        origin=301, x_axis=302, height_groups=((301, 302, 303), (304, 305))
    )
    flat = {a: (x, y, 0.0) for a, (x, y, _) in FIVE.items()}

    fit = fit_survey(list(FIVE), survey(FIVE, noise=0.005), two_heights, placement=flat)

    assert fit.rms_m < 0.02
    assert abs(fit.anchors[304].position[2]) == pytest.approx(2.2, abs=0.1)
    # the mirror image also turns over the few millimetres an anchor of the
    # lower group stands off it, so one of those can be named too
    assert {("not-located", 304, None), ("not-located", 305, None)} <= set(
        codes(fit.blocked)
    )
    assert {code for code, _, _ in codes(fit.blocked)} == {"not-located"}
    assert "neither the placement nor the frame" in fit.blocked[0].detail


def test_reference_anchor_nearly_on_the_x_axis_leaves_the_mirror_open():
    layout, measurements = read_to_the_centimetre(
        [(0.0, 0.0), (8.0, 0.0), (4.0, 0.01), (0.0, 6.0)]
    )
    measurements[3, 4] = measurements[4, 3] = DirectedRange(7.22, 0.01, 95, 0.95)
    frame = Frame.two_tiers(list(layout))

    fit = fit_survey(list(layout), measurements, frame)

    assert ("not-located", 4, None) in codes(fit.blocked)
    assert {code for code, _, _ in codes(fit.blocked)} == {"not-located"}
    assert "nearly in line" in fit.blocked[-1].detail
    assert abs(fit.anchors[4].position[1]) == pytest.approx(6.0, abs=0.05)

    sketched = fit_survey(list(layout), measurements, frame, placement=dict(layout))

    assert sketched.anchors[4].position[1] == pytest.approx(6.0, abs=0.05)
    assert sketched.blocked == ()


def test_reference_anchor_within_three_sigmas_of_the_x_axis_leaves_the_mirror_open():
    thin = {1: (0.0, 0.0, 0.0), 2: (6.0, 0.0, 0.0), 3: (3.0, 0.45, 0.0)}

    fit = fit_survey(list(thin), survey(thin, noise=0.0), Frame.two_tiers(list(thin)))

    assert 0.45 / 3 < sigma_of(fit, 3) < 0.20
    assert codes(fit.blocked) == [("not-located", 3, None)]
    assert "nearly in line" in fit.blocked[0].detail


def test_upper_tier_barely_above_the_lower_leaves_the_mirror_open():
    layout, measurements = read_to_the_centimetre(
        [(x, y, z) for z in (0.0, -0.25) for y in (0.0, 5.0) for x in (0.0, 6.0)]
    )
    frame = Frame.two_tiers(list(layout))

    fit = fit_survey(list(layout), measurements, frame)

    assert {("not-located", a, None) for a in (5, 6, 7, 8)} <= set(codes(fit.blocked))
    assert {code for code, _, _ in codes(fit.blocked)} == {"not-located"}
    assert "nearly level" in fit.blocked[0].detail
    assert abs(fit.anchors[5].position[2]) == pytest.approx(0.25, abs=0.03)

    stored = fit_survey(list(layout), measurements, frame, stored=layout)

    assert worst_error(stored, layout) < 0.03
    assert stored.blocked == ()


def test_triangle_of_a_few_metres_is_settled_by_the_preset():
    fit = fit_survey(
        list(TRIANGLE), survey(TRIANGLE, noise=0.0), Frame.two_tiers(list(TRIANGLE))
    )

    assert worst_error(fit, TRIANGLE) < 1e-6
    assert fit.blocked == ()


def test_top_down_sketch_cannot_tell_above_from_below():
    undeclared = Frame(origin=301, x_axis=302, height_groups=((301, 302, 303),))
    flat_sketch = {a: (x, y) for a, (x, y, _) in FIVE.items()}

    fit = fit_survey(list(FIVE), survey(FIVE), undeclared, placement=flat_sketch)

    assert codes(fit.blocked) == [
        ("not-located", 304, None),
        ("not-located", 305, None),
    ]
    assert abs(fit.anchors[304].position[2]) == pytest.approx(2.2, abs=0.2)


def test_ranges_overrule_an_anchor_placed_on_the_wrong_side_of_its_neighbours():
    sketch = dict(FIVE)
    sketch[305] = (1.0, 1.5, 2.2)

    fit = fit_survey(
        list(FIVE), survey(FIVE, noise=0.005), FIVE_FREE_HEIGHTS, placement=sketch
    )

    assert worst_error(fit, FIVE) < 0.08
    assert fit.blocked == () and fit.notes == ()


WITH_ONE_BELOW = {**FIVE, 305: (1.0, 1.5, 1.2), 306: (2.5, 0.8, -1.9)}


def test_anchor_declared_upper_but_ranged_below_is_noted():
    frame = Frame(
        origin=301,
        x_axis=302,
        height_groups=((301, 302, 303),),
        upper=(304, 305, 306),
    )

    fit = fit_survey(
        list(WITH_ONE_BELOW),
        survey(WITH_ONE_BELOW, noise=0.005),
        frame,
        placement=rough(WITH_ONE_BELOW),
    )

    assert worst_error(fit, WITH_ONE_BELOW) < 0.08
    assert fit.notes == (
        "anchor 305 is declared upper but the ranges put it below the origin's group",
    )
    assert fit.blocked == ()


def test_upper_declaration_the_ranges_contradict_evenly_settles_nothing():
    frame = Frame(
        origin=301, x_axis=302, height_groups=((301, 302, 303),), upper=(304, 305)
    )
    layout = {a: WITH_ONE_BELOW[a] for a in FIVE}

    fit = fit_survey(
        list(layout), survey(layout, noise=0.005), frame, placement=rough(layout)
    )

    assert codes(fit.blocked) == [
        ("not-located", 304, None),
        ("not-located", 305, None),
    ]


def test_anchor_with_two_links_is_not_located():
    fit = fit_survey(
        list(FIVE),
        survey(FIVE, drop={(301, 305), (302, 305)}),
        FIVE_FRAME,
        placement=rough(FIVE),
    )

    assert codes(fit.blocked)[:2] == [
        ("pair-missing", 301, 305),
        ("pair-missing", 302, 305),
    ]
    assert codes(fit.blocked)[-1] == ("not-located", 305, None)
    assert fit.blocked[-1].detail == "2 links, a position needs three"
    assert fit.anchors[305].links == 2


def test_three_anchors_in_a_plane_are_located_by_two_links_each():
    fit = fit_survey(
        list(TRIANGLE), survey(TRIANGLE, noise=0.0), TRIANGLE_FRAME, placement=TRIANGLE
    )

    assert worst_error(fit, TRIANGLE) < 1e-6
    assert fit.planar
    assert fit.blocked == ()
    # No spare measurement: the assumed range noise, on the range that is x.
    assert sigma_of(fit, 2) == pytest.approx(0.03, rel=1e-6)


def test_heights_the_frame_leaves_free_to_tilt_are_not_located():
    untied = Frame(origin=101, x_axis=102)

    fit = fit_survey(list(TWO_TIERS), survey(TWO_TIERS), untied, placement=TWO_TIERS)

    assert codes(fit.blocked) == [
        ("not-located", anchor, None) for anchor in list(TWO_TIERS)[1:]
    ]
    assert fit.blocked[0].detail == "the ranges do not fix its position"
    assert fit.anchors[103].sigma_m is None
    assert fit.json["fit"]["anchors"]["103"]["sigmaM"] is None


def test_anchor_the_ranges_hold_weakly_is_not_located():
    stretched = {
        1: (0.0, 0.0, 0.0),
        2: (5.0, 0.0, 0.0),
        3: (10.0, 7.0, 0.0),
        4: (15.0, 3.0, 0.0),
    }
    frame = Frame(origin=1, x_axis=2, height_groups=((1, 2, 3, 4),))

    fit = fit_survey(list(stretched), survey(stretched), frame, placement=stretched)

    sigma = sigma_of(fit, 4)
    assert 0.20 < sigma < 0.40
    assert sigma_of(fit, 3) < 0.20
    assert codes(fit.blocked) == [("not-located", 4, None)]
    assert fit.blocked[0].detail == f"predicted uncertainty {sigma:.2f} m"


def test_anchor_held_by_three_links_is_not_located_when_its_mirror_fits_as_well():
    # 305 ranged only 301, 302 and 304: its image through their plane fits
    # those three ranges too, and nothing here says which side it is on.
    frame = Frame(
        origin=301,
        x_axis=302,
        height_groups=((301, 302, 303),),
        upper=(304, 305),
        y_positive=303,
    )

    fit = fit_survey(list(FIVE), survey(FIVE, drop={(303, 305)}), frame)

    assert codes(fit.blocked) == [
        ("pair-missing", 303, 305),
        ("not-located", 305, None),
    ]
    assert fit.anchors[305].links == 3
    assert "another layout fits the ranges as well" in fit.blocked[1].detail


def test_one_spare_range_is_enough_to_estimate_the_noise():
    square = {
        1: (0.0, 0.0, 0.0),
        2: (5.0, 0.0, 0.0),
        3: (0.0, 5.0, 0.0),
        4: (5.0, 5.0, 0.0),
    }
    frame = Frame(origin=1, x_axis=2, height_groups=((1, 2, 3, 4),), y_positive=3)
    exact = survey(square, noise=0.0)
    off = dict(exact)
    off[1, 4] = off[4, 1] = DirectedRange(exact[1, 4].range_m + 0.2, 0.005, 96, 0.96)

    clean = fit_survey(list(square), exact, frame)
    strained = fit_survey(list(square), off, frame)

    # Six ranges for five unknowns: the one spare range carries all of the
    # residual, and a clean fit is held at the 2.5 cm floor.
    assert clean.rms_m < 1e-6
    noise = strained.rms_m * np.sqrt(6)
    assert noise > 0.05
    assert sigma_of(strained, 2) / sigma_of(clean, 2) == pytest.approx(
        noise / 0.025, rel=0.1
    )


def test_square_cell_surveyed_again_keeps_its_side():
    square = {
        1: (0.0, 0.0, 0.0),
        2: (5.0, 0.0, 0.0),
        3: (0.0, 5.0, 0.0),
        4: (5.0, 5.0, 0.0),
    }
    frame = Frame(origin=1, x_axis=2, height_groups=((1, 2, 3, 4),))

    fit = fit_survey(list(square), survey(square, noise=0.0), frame, stored=square)

    assert worst_error(fit, square) < 1e-6
    assert fit.notes == () and fit.blocked == ()


def test_x_anchor_sketched_on_the_origin_still_lands_on_plus_x():
    half_turn = {a: (-x, -y) for a, (x, y, _) in L_SHAPE.items()}
    half_turn[202] = half_turn[201]

    fit = fit_survey(list(L_SHAPE), survey(L_SHAPE), L_SHAPE_FRAME, placement=half_turn)

    assert worst_error(fit, L_SHAPE) < 0.10
    assert fit.blocked == ()


def test_cell_taller_than_wide_needs_no_placement():
    tall = {a: (x * 0.5, y, z * 2.5) for a, (x, y, z) in TWO_TIERS.items()}

    fit = fit_survey(list(tall), survey(tall), TWO_TIERS_FRAME)

    assert worst_error(fit, tall) < 0.10
    assert fit.blocked == ()


def test_stored_positions_in_another_frame_start_the_fit():
    quarter_turn = {
        a: (-y + 3.0, x - 2.0, z + 1.0) for a, (x, y, z) in TWO_TIERS.items()
    }

    fit = fit_survey(
        list(TWO_TIERS), survey(TWO_TIERS), TWO_TIERS_NO_SIDE, stored=quarter_turn
    )

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert fit.blocked == ()
    assert moved_of(fit, 101) == pytest.approx(np.sqrt(14.0), abs=0.01)


# Above the 2.5 cm floor of the prediction, which would otherwise be what is
# compared with the spread.
MONTE_CARLO_NOISE_M = 0.04
#: the cell with heights the ranges alone decide: one exact group, the rest
#: free. In two groups the tolerance of a height adds what no noise shows.
LEVEL_AND_FREE = Frame(
    origin=101,
    x_axis=102,
    height_groups=((101, 102, 103, 104),),
    upper=(105, 106, 107, 108),
    y_positive=103,
)


def test_predicted_sigma_matches_the_monte_carlo_spread():
    runs = 100
    positions = np.empty((runs, len(TWO_TIERS), 3))
    predicted = np.empty((runs, len(TWO_TIERS)))
    for run in range(runs):
        fit = fit_survey(
            list(TWO_TIERS),
            survey(TWO_TIERS, seed=run, noise=MONTE_CARLO_NOISE_M),
            LEVEL_AND_FREE,
            placement=TWO_TIERS,
        )
        positions[run] = [report.position for report in fit.anchors.values()]
        predicted[run] = [report.sigma_m for report in fit.anchors.values()]

    spread = np.sqrt(
        ((positions - positions.mean(axis=0)) ** 2).sum(axis=2).mean(axis=0)
    )

    assert predicted[:, 0].max() == 0.0 and spread[0] == 0.0
    assert predicted.mean(axis=0)[1:] == pytest.approx(spread[1:], rel=0.15)
    # The +x anchor only moves along x; the rest also carry the cell's turn.
    assert spread[1] < 0.7 * spread[2:].min()


def test_ranges_that_agree_exactly_are_not_taken_for_a_perfect_survey():
    fit = fit_survey(list(TWO_TIERS), survey(TWO_TIERS, noise=0.0), TWO_TIERS_FRAME)

    assert fit.rms_m < 1e-6
    sigmas = [report.sigma_m for report in fit.anchors.values()]
    assert 0.015 < min(sigmas) and max(sigmas) < 0.06


def test_stored_positions_start_the_fit_and_give_the_move():
    stored = dict(TWO_TIERS)
    stored[104] = (6.1, 5.3, 0.0)

    fit = fit_survey(
        list(TWO_TIERS), survey(TWO_TIERS), TWO_TIERS_NO_SIDE, stored=stored
    )

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert fit.blocked == ()
    assert moved_of(fit, 104) == pytest.approx(0.3, abs=0.05)
    assert moved_of(fit, 107) < 0.05
    # the origin keeps its x and y; its height is its own, near its group's
    assert fit.anchors[101].position[:2] == (0.0, 0.0)
    assert moved_of(fit, 101) < 0.03


def test_partly_stored_positions_give_the_move_but_no_start():
    fit = fit_survey(
        list(TWO_TIERS),
        survey(TWO_TIERS),
        TWO_TIERS_FRAME,
        stored={102: TWO_TIERS[102]},
    )

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert moved_of(fit, 102) < 0.05
    assert fit.anchors[103].moved_m is None
    assert "movedM" in fit.json["fit"]["anchors"]["102"]
    assert "movedM" not in fit.json["fit"]["anchors"]["103"]


def test_placement_that_misses_an_anchor_is_not_used():
    sketch = rough(mirrored(TWO_TIERS, y=-1.0))
    del sketch[108]

    fit = fit_survey(
        list(TWO_TIERS), survey(TWO_TIERS), TWO_TIERS_FRAME, placement=sketch
    )

    assert worst_error(fit, TWO_TIERS) < 0.08
    assert fit.notes == ("the placement does not cover every anchor and was not used",)


def test_nothing_to_start_from_is_refused():
    apart = survey(FIVE, drop={(a, b) for a in (301, 302, 303) for b in (304, 305)})

    with pytest.raises(SurveyFitError, match="Place the anchors roughly first"):
        fit_survey(list(FIVE), apart, FIVE_FRAME)


@pytest.mark.parametrize(
    ("frame", "reason"),
    [
        (
            Frame(origin=301, x_axis=309),
            "The frame names anchor 309, which is not in the survey",
        ),
        (
            Frame(origin=301, x_axis=302, upper=(309,)),
            "The frame names anchor 309, which is not in the survey",
        ),
        (
            Frame(origin=301, x_axis=302, y_positive=309),
            "The frame names anchor 309, which is not in the survey",
        ),
        (
            Frame(origin=301, x_axis=301),
            "The origin and the +x anchor must be two anchors",
        ),
        (
            Frame(origin=301, x_axis=302, height_groups=((301, 303), (303, 304))),
            "An anchor is in two height groups",
        ),
        (
            Frame(origin=301, x_axis=302, height_groups=((301, 303),), upper=(303,)),
            "An upper anchor cannot be level with the origin",
        ),
        (
            Frame(origin=301, x_axis=302, upper=(301,)),
            "An upper anchor cannot be level with the origin",
        ),
        (
            Frame(origin=301, x_axis=302, y_positive=302),
            "The anchor at y > 0 cannot be on the x axis",
        ),
    ],
)
def test_frame_that_does_not_fit_the_group_is_refused(frame, reason):
    assert frame_refusal(frame, list(FIVE)) == reason
    with pytest.raises(SurveyFitError, match=re.escape(reason)):
        fit_survey(list(FIVE), survey(FIVE), frame, placement=FIVE)


def test_group_listing_an_anchor_twice_is_refused():
    assert frame_refusal(FIVE_FRAME, [*FIVE, 303]) == "An anchor is listed twice"
    assert frame_refusal(FIVE_FRAME, list(FIVE)) is None


def test_noisy_survey_warns_about_the_rms():
    fit = fit_survey(list(TWO_TIERS), survey(TWO_TIERS, noise=0.09), TWO_TIERS_FRAME)

    assert fit.rms_m > 0.05
    assert ("rms", None, None) in codes(fit.warnings)
    assert fit.warnings[-1].detail == f"fit rms {fit.rms_m:.3f} m"
    assert fit.blocked == ()


def test_range_that_disagrees_with_the_rest_warns_about_its_pair():
    measurements = survey(TWO_TIERS, noise=0.0)
    for key in ((103, 106), (106, 103)):
        measurements[key] = DirectedRange(
            measurements[key].range_m + 0.4, 0.005, 96, 0.96
        )

    fit = fit_survey(list(TWO_TIERS), measurements, TWO_TIERS_FRAME)

    assert [c for c in codes(fit.warnings) if c[0] == "pair"] == [("pair", 103, 106)]
    pair = next(p for p in fit.pairs if (p.a, p.b) == (103, 106))
    assert pair.residual_m < -0.10
    assert pair.fitted_m == pytest.approx(pair.range_m + pair.residual_m)
    assert fit.warnings[0].detail == f"residual {pair.residual_m:+.3f} m"
    assert fit.worst_m == pytest.approx(-pair.residual_m)
    assert fit.anchors[103].worst_pair_m == pytest.approx(-pair.residual_m)
    assert fit.anchors[103].residual_rms_m > fit.anchors[101].residual_rms_m


def test_directions_that_disagree_warn_about_asymmetry():
    measurements = survey(TWO_TIERS, noise=0.0)
    measurements[104, 107] = DirectedRange(
        measurements[104, 107].range_m + 0.04, 0.005, 96, 0.96
    )

    fit = fit_survey(list(TWO_TIERS), measurements, TWO_TIERS_FRAME)

    assert codes(fit.warnings) == [("asymmetry", 104, 107)]
    assert fit.warnings[0].detail == "the two directions differ by 0.040 m"
    pair = next(p for p in fit.pairs if (p.a, p.b) == (104, 107))
    assert pair.range_m == pytest.approx(measurements[107, 104].range_m + 0.02)


def test_low_valid_share_warns_about_the_direction():
    measurements = survey(TWO_TIERS, noise=0.0)
    measurements[102, 105] = DirectedRange(
        measurements[102, 105].range_m, 0.005, 80, 0.8
    )

    fit = fit_survey(list(TWO_TIERS), measurements, TWO_TIERS_FRAME)

    assert codes(fit.warnings) == [("low-share", 105, 102)]
    assert fit.warnings[0].detail == "valid share 0.80 of its ranges to 102"


@pytest.mark.parametrize(
    "poor",
    [
        DirectedRange(range_m=9.0, mad_m=0.2, inliers=19, share=0.95),
        DirectedRange(range_m=9.0, mad_m=0.2, inliers=40, share=0.45),
    ],
    ids=["few inliers", "low share"],
)
def test_direction_too_poor_to_use_leaves_the_pair_to_the_other(poor):
    measurements = survey(TWO_TIERS, noise=0.0)
    measurements[101, 108] = poor

    fit = fit_survey(list(TWO_TIERS), measurements, TWO_TIERS_FRAME)

    assert worst_error(fit, TWO_TIERS) < 1e-6
    pair = next(p for p in fit.pairs if (p.a, p.b) == (101, 108))
    assert pair.range_m == measurements[108, 101].range_m
    assert pair.ab == poor
    assert "asymmetry" not in [code for code, _, _ in codes(fit.warnings)]
    assert fit.blocked == ()


def test_pair_with_no_usable_direction_is_missing():
    measurements = survey(TWO_TIERS, noise=0.0)
    measurements[101, 108] = DirectedRange(9.0, 0.2, 19, 0.95)
    del measurements[108, 101]

    fit = fit_survey(list(TWO_TIERS), measurements, TWO_TIERS_FRAME)

    assert codes(fit.blocked) == [("pair-missing", 101, 108)]
    assert fit.blocked[0].detail == "too few valid ranges"
    assert len(fit.pairs) == 27


def test_usable_direction_needs_twenty_inliers_and_half_the_ranges():
    assert DirectedRange(1.0, 0.0, 20, 0.5).usable
    assert not DirectedRange(1.0, 0.0, 19, 0.5).usable
    assert not DirectedRange(1.0, 0.0, 20, 0.49).usable


def test_result_fills_the_survey_state_object():
    measurements = survey(TRIANGLE, noise=0.0)
    measurements[2, 1] = DirectedRange(4.04, 0.01234567, 80, 0.8004)
    del measurements[3, 2]

    fit = fit_survey(
        list(TRIANGLE),
        measurements,
        TRIANGLE_FRAME,
        placement=TRIANGLE,
        stored={3: (0.0, 3.5, 0.0)},
    )

    assert fit.json == {
        "pairs": [
            {
                "a": 1,
                "b": 2,
                "rangeM": 4.02,
                "ab": {"rangeM": 4.0, "madM": 0.005, "n": 96, "share": 0.96},
                "ba": {"rangeM": 4.04, "madM": 0.0123, "n": 80, "share": 0.8},
                "fittedM": 4.02,
                "residualM": 0.0,
            },
            {
                "a": 1,
                "b": 3,
                "rangeM": 3.0,
                "ab": {"rangeM": 3.0, "madM": 0.005, "n": 96, "share": 0.96},
                "ba": {"rangeM": 3.0, "madM": 0.005, "n": 96, "share": 0.96},
                "fittedM": 3.0,
                "residualM": 0.0,
            },
            {
                "a": 2,
                "b": 3,
                "rangeM": 5.0,
                "ab": {"rangeM": 5.0, "madM": 0.005, "n": 96, "share": 0.96},
                "ba": None,
                "fittedM": 5.0,
                "residualM": 0.0,
            },
        ],
        "fit": {
            "rmsM": 0.0,
            "worstM": 0.0,
            "planar": True,
            "anchors": {
                "1": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "links": 2,
                    "residualRmsM": 0.0,
                    "worstPairM": 0.0,
                    "sigmaM": 0.0,
                    "heightOffsetM": 0.0,
                },
                "2": {
                    "x": 4.02,
                    "y": 0.0,
                    "z": 0.0,
                    "links": 2,
                    "residualRmsM": 0.0,
                    "worstPairM": 0.0,
                    "sigmaM": 0.03,
                    "heightOffsetM": 0.0,
                },
                "3": {
                    "x": 0.02,
                    "y": 2.9999,
                    "z": 0.0,
                    "links": 2,
                    "residualRmsM": 0.0,
                    "worstPairM": 0.0,
                    "sigmaM": 0.0606,
                    "heightOffsetM": 0.0,
                    "movedM": 0.5005,
                },
            },
            "notes": [],
        },
        "gate": {
            "blocked": [],
            "warnings": [
                {
                    "code": "asymmetry",
                    "device": 1,
                    "peer": 2,
                    "detail": "the two directions differ by 0.040 m",
                },
                {
                    "code": "low-share",
                    "device": 1,
                    "peer": 2,
                    "detail": "valid share 0.80 of its ranges to 2",
                },
            ],
        },
    }


# The fit tries many starts and most layouts are found from several of them,
# so what each start is made of is checked here, on its own.


def problem_of(layout, frame, **survey_options):
    anchors = tuple(layout)
    ranges = _pair_ranges(anchors, survey(layout, **survey_options))
    return _Problem(anchors, frame, ranges)


def rows(layout):
    return np.array(list(layout.values()), dtype=float)


TALL = {a: (x * 0.5, y, z * 2.5) for a, (x, y, z) in TWO_TIERS.items()}


@pytest.mark.parametrize("layout", [TWO_TIERS, TALL], ids=["wide", "tall"])
def test_start_without_a_placement_is_the_measured_shape_set_level(layout):
    problem = problem_of(layout, TWO_TIERS_FRAME)

    start = problem.aligned(problem.scaled_start())

    assert np.abs(start[:4, 2]).max() < 0.05
    assert np.abs(start[4:, 2]) == pytest.approx(-rows(layout)[4:, 2], abs=0.10)
    assert np.abs(start[:, :2]) == pytest.approx(np.abs(rows(layout)[:, :2]), abs=0.10)


def test_placement_is_moved_and_turned_into_the_declared_frame():
    problem = problem_of(TWO_TIERS, TWO_TIERS_FRAME)
    elsewhere = np.array(
        [(-y + 3.0, x - 2.0, z + 1.0) for x, y, z in TWO_TIERS.values()]
    )
    elsewhere[5, 2] = np.nan

    moved = problem.aligned(elsewhere)

    expected = rows(TWO_TIERS)
    expected[5, 2] = np.nan
    assert moved == pytest.approx(expected, abs=1e-9, nan_ok=True)


def test_heights_the_placement_leaves_out_start_above_the_origin_group():
    problem = problem_of(TWO_TIERS, TWO_TIERS_FRAME)
    sketch = rows(TWO_TIERS)
    sketch[:, 2] = np.nan

    assert list(problem.with_heights(sketch)[:, 2]) == [0.0] * 4 + [-START_HEIGHT_M] * 4

    sketch[[2, 6], 2] = [0.4, -3.0]

    assert list(problem.with_heights(sketch)[:, 2]) == [0.0] * 4 + [-3.0] * 4


@pytest.mark.parametrize(
    ("drawn", "started"),
    [
        (0.0, -START_HEIGHT_M),
        (0.2, -START_HEIGHT_M),
        (3.0, -3.0),
        (-0.2, -START_HEIGHT_M),
    ],
)
def test_upper_anchors_start_above_the_origin_group_whatever_the_sketch_says(
    drawn, started
):
    problem = problem_of(TWO_TIERS, TWO_TIERS_FRAME)
    sketch = rows(TWO_TIERS)
    sketch[:, 2] = drawn

    assert list(problem.with_heights(sketch)[:, 2]) == [0.0] * 4 + [started] * 4


@pytest.mark.parametrize(
    ("drawn", "started"),
    [(0.0, -START_HEIGHT_M), (0.2, START_HEIGHT_M), (3.0, 3.0), (-3.0, -3.0)],
)
def test_height_the_frame_does_not_call_upper_starts_on_the_side_it_was_drawn(
    drawn, started
):
    two_heights = Frame(
        origin=301, x_axis=302, height_groups=((301, 302, 303), (304, 305))
    )
    problem = problem_of(FIVE, two_heights)
    sketch = rows(FIVE)
    sketch[:, 2] = drawn

    assert list(problem.with_heights(sketch)[:, 2]) == [0.0] * 3 + [started] * 2


def test_variants_exchange_every_pair_and_reflect_through_the_nearest_ranged():
    problem = problem_of(FIVE, FIVE_FREE_HEIGHTS)
    start = rows(FIVE)

    variants = list(problem.variants(start))

    assert len(variants) == 1 + 10 + 5
    assert (variants[0] == start).all()
    assert (variants[1] == start[[1, 0, 2, 3, 4]]).all()

    def ranges_from_305(layout):
        return np.linalg.norm(layout[:4] - layout[4], axis=1)

    # 305 is nearest to 301, 303 and 304, and 302 is off their plane.
    reflected = variants[-1]
    assert (reflected[:4] == start[:4]).all()
    assert np.linalg.norm(reflected[4] - start[4]) > 1.0
    assert ranges_from_305(reflected)[[0, 2, 3]] == pytest.approx(
        ranges_from_305(start)[[0, 2, 3]]
    )
    assert abs(ranges_from_305(reflected)[1] - ranges_from_305(start)[1]) > 0.5


def test_every_solution_comes_with_its_mirror_images():
    in_space = _distinct([_Solution(rows(FIVE), 0.01)])
    in_a_plane = _distinct([_Solution(rows(L_SHAPE), 0.01)])

    sides_of_304 = {tuple(np.sign(layout.positions[3, 1:])) for layout in in_space}
    assert sides_of_304 == {(1.0, -1.0), (-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)}
    assert len(in_space) == 4
    assert [layout.positions[2, 1] for layout in in_a_plane] == [3.0, -3.0]


# ---- height groups: about the same height ---------------------------------


def levelled_offsets(layout, groups):
    """How far each anchor stands above the common height of its group once
    the cell is tilted to leave the least of that: all that ranges can show
    of the heights, since they do not know which way is up."""
    anchors = [anchor for group in groups for anchor in group]
    design = np.array(
        [
            [*(float(anchor in group) for group in groups), *layout[anchor][:2]]
            for anchor in anchors
        ]
    )
    height = np.array([-layout[anchor][2] for anchor in anchors])
    level = design @ np.linalg.lstsq(design, height, rcond=None)[0]
    return dict(zip(anchors, height - level))


def test_an_anchor_off_its_tier_keeps_its_own_height():
    off = dict(TWO_TIERS)
    x, y, z = off[107]
    off[107] = (x, y, z - 0.04)

    fit = fit_survey(list(off), survey(off, noise=0.0), TWO_TIERS_FRAME)

    # the cell is found as it stands: no pair is bent to level the tier
    for a, b in combinations(off, 2):
        found = dist(fit.anchors[a].position, fit.anchors[b].position)
        assert found == pytest.approx(dist(off[a], off[b]), abs=0.005)
    assert fit.rms_m < 0.002 and fit.blocked == () and fit.warnings == ()
    # a tilt of the whole cell explains half of one anchor's 4 cm, so level
    # is the plane that leaves the least: 2 cm on that anchor, less on others
    truth = levelled_offsets(off, TWO_TIERS_FRAME.height_groups)
    assert truth[107] == pytest.approx(0.02, abs=0.002)
    offsets = {a: offset_of(fit, a) for a in off}
    assert offsets == pytest.approx(truth, abs=0.01)
    assert offsets[107] == max(offsets.values()) > 0.015
    mates = np.mean([fit.anchors[a].position[2] for a in (105, 106, 108)])
    assert mates - fit.anchors[107].position[2] > 0.02


def test_the_exact_fit_cannot_show_an_anchor_off_its_tier():
    off = dict(TWO_TIERS)
    x, y, z = off[107]
    off[107] = (x, y, z - 0.04)
    one_group = Frame(
        origin=101, x_axis=102, height_groups=((101, 102, 103, 104),), y_positive=103
    )

    fit = fit_survey(list(off), survey(off, noise=0.0), LEVEL_AND_FREE)
    flat = fit_survey(
        list(off)[:4], survey({a: off[a] for a in list(off)[:4]}), one_group
    )

    # a single group shares its height exactly, whatever the ranges say
    assert [fit.anchors[a].position[2] for a in (101, 102, 103, 104)] == [0.0] * 4
    assert [fit.anchors[a].height_offset_m for a in (101, 102, 103, 104)] == [0.0] * 4
    assert [fit.anchors[a].height_offset_m for a in (105, 106, 107, 108)] == [None] * 4
    assert flat.planar and not fit.planar
    assert {report.height_offset_m for report in flat.anchors.values()} == {0.0}
    assert {report.position[2] for report in flat.anchors.values()} == {0.0}


def test_a_group_of_its_own_away_from_the_origin_shares_one_height_exactly():
    apart = Frame(origin=301, x_axis=302, height_groups=((304, 305),), upper=(304,))

    fit = fit_survey(list(FIVE), survey(FIVE), apart, placement=FIVE)

    assert fit.anchors[304].position[2] == fit.anchors[305].position[2]
    assert (fit.anchors[304].height_offset_m, fit.anchors[305].height_offset_m) == (
        0.0,
        0.0,
    )
    assert fit.anchors[303].height_offset_m is None


def test_the_pulls_are_not_in_the_residuals():
    # level tiers, exact ranges: the pulls have nothing to pull
    exact = survey(TWO_TIERS, noise=0.0)
    similar = fit_survey(list(TWO_TIERS), exact, TWO_TIERS_FRAME)
    free = fit_survey(list(TWO_TIERS), exact, LEVEL_AND_FREE)

    assert similar.rms_m == pytest.approx(free.rms_m, abs=1e-6)
    assert similar.rms_m < 1e-6 and similar.worst_m < 1e-6
    assert worst_error(similar, TWO_TIERS) < 1e-4
    assert max(abs(offset_of(similar, a)) for a in TWO_TIERS) < 1e-4

    # noisy ranges: the rms is of the pairs alone, though the pulls are at work
    noisy = fit_survey(list(TWO_TIERS), survey(TWO_TIERS), TWO_TIERS_FRAME)
    residuals = np.array([pair.residual_m for pair in noisy.pairs])
    assert len(residuals) == 28
    assert noisy.rms_m == pytest.approx(float(np.sqrt(np.mean(residuals**2))))
    assert noisy.worst_m == pytest.approx(float(np.abs(residuals).max()))
    assert max(abs(offset_of(noisy, a)) for a in TWO_TIERS) > 0.005
    for report, anchor in zip(noisy.anchors.values(), TWO_TIERS):
        own = [p.residual_m for p in noisy.pairs if anchor in (p.a, p.b)]
        assert report.residual_rms_m == pytest.approx(
            float(np.sqrt(np.mean(np.square(own))))
        )


def test_a_noisy_survey_is_not_blocked_by_the_heights_it_cannot_see():
    fit = fit_survey(list(TWO_TIERS), survey(TWO_TIERS, noise=0.09), TWO_TIERS_FRAME)

    # the tolerance of a height holds whatever the ranges' noise
    assert fit.blocked == ()
    assert max(sigma_of(fit, anchor) for anchor in TWO_TIERS) < 0.20
