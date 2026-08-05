import math

import pytest

from nav2_apps.loaded_scan_profile import build_profile, compare_profiles


def _sample(ranges):
    return {
        "frame_id": "laser",
        "angle_min": -0.3,
        "angle_increment": 0.1,
        "range_min": 0.05,
        "range_max": 10.0,
        "ranges": ranges,
    }


def test_build_profile_rejects_geometry_change():
    first = _sample([1.0, 2.0])
    second = _sample([1.0, 2.0, 3.0])
    with pytest.raises(ValueError, match="geometry changed"):
        build_profile([first, second])


def test_build_profile_ignores_invalid_ranges_and_tracks_spread():
    profile = build_profile(
        [
            _sample([math.inf, 0.50, 2.0]),
            _sample([math.nan, 0.52, 2.2]),
            _sample([0.01, 0.51, 2.1]),
        ]
    )
    assert profile["beams"][0]["valid_fraction"] == 0.0
    assert profile["beams"][1]["median"] == pytest.approx(0.51)
    assert profile["beams"][1]["spread"] == pytest.approx(0.016)


def test_compare_reports_stable_new_near_interval_as_filter_candidate():
    baseline = build_profile(
        [_sample([3.0] * 8), _sample([3.1] * 8), _sample([2.9] * 8)]
    )
    loaded_ranges = [3.0, 3.0, 0.55, 0.56, 0.57, 3.0, 3.0, 3.0]
    loaded = build_profile(
        [
            _sample(loaded_ranges),
            _sample(loaded_ranges),
            _sample(loaded_ranges),
        ]
    )
    result = compare_profiles(baseline, loaded)
    assert result["filter_candidates"] == [
        {
            "start_index": 2,
            "end_index": 4,
            "beam_count": 3,
            "angle_min_rad": pytest.approx(-0.1),
            "angle_max_rad": pytest.approx(0.1),
            "angle_min_deg": pytest.approx(math.degrees(-0.1)),
            "angle_max_deg": pytest.approx(math.degrees(0.1)),
        }
    ]


def test_compare_does_not_suggest_unstable_or_short_changes():
    baseline = build_profile([_sample([3.0] * 5)] * 3)
    loaded = build_profile(
        [
            _sample([3.0, 0.3, 0.3, 3.0, 3.0]),
            _sample([3.0, 0.8, 0.8, 3.0, 3.0]),
            _sample([3.0, 0.5, 0.5, 3.0, 3.0]),
        ]
    )
    assert compare_profiles(baseline, loaded)["filter_candidates"] == []


def test_compare_reports_lost_beams_separately_from_filter_candidates():
    baseline = build_profile([_sample([2.0] * 5)] * 3)
    loaded = build_profile([_sample([math.inf] * 5)] * 3)
    result = compare_profiles(baseline, loaded)
    assert result["filter_candidates"] == []
    assert result["lost_beam_intervals"][0]["beam_count"] == 5
