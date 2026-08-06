import importlib.util
import math
from pathlib import Path

from geometry_msgs.msg import Point32


SCRIPT = Path(__file__).parents[1] / "scripts" / "loaded_scan_filter.py"
SPEC = importlib.util.spec_from_file_location("loaded_scan_filter", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


SECTORS = ((-2.3562, -1.111), (0.843, 2.3562))


def test_unloaded_scan_is_exact_pass_through():
    values = [0.3, 1.2, math.inf, 0.4]
    result = MODULE.filter_ranges(values, -2.0, 1.0, False, SECTORS, 0.60)

    assert result == values
    assert result is not values


def test_loaded_filter_removes_only_near_returns_in_measured_sectors():
    values = [0.4, 0.8, 0.3, 0.4, 0.5]
    result = MODULE.filter_ranges(values, -2.0, 1.0, True, SECTORS, 0.60)

    assert math.isinf(result[0])  # -2.0 rad, near shelf return
    assert result[1] == 0.8  # sector match, but beyond self-return range
    assert result[2] == 0.3  # near, but forward environmental sector
    assert math.isinf(result[3])  # +1.0 rad, near shelf return
    assert math.isinf(result[4])  # +2.0 rad, near shelf return


def test_existing_invalid_ranges_remain_invalid():
    values = [math.inf, math.nan]
    result = MODULE.filter_ranges(values, -2.0, 4.0, True, SECTORS, 0.60)

    assert math.isinf(result[0])
    assert math.isnan(result[1])


def _square(half_extent):
    return [
        Point32(x=half_extent, y=half_extent),
        Point32(x=-half_extent, y=half_extent),
        Point32(x=-half_extent, y=-half_extent),
        Point32(x=half_extent, y=-half_extent),
    ]


def test_footprint_edge_distinguishes_loaded_and_unloaded_profiles():
    assert math.isclose(MODULE.maximum_polygon_edge(_square(0.30)), 0.60)
    assert math.isclose(MODULE.maximum_polygon_edge(_square(0.25)), 0.50)


def test_incomplete_footprint_cannot_disable_filtering():
    assert math.isinf(MODULE.maximum_polygon_edge(_square(0.25)[:2]))
