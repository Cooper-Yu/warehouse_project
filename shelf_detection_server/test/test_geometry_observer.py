import ast
import math
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import LaserScan


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.leg_geometry import (  # noqa: E402
    detect_leg_pair,
    shelf_normal_yaw,
)


def _scan_with_two_legs():
    scan = LaserScan()
    scan.header.frame_id = "laser"
    scan.angle_min = -0.5
    scan.angle_increment = 0.01
    scan.range_min = 0.01
    scan.range_max = 20.0
    scan.ranges = [math.inf] * 101
    scan.intensities = [0.0] * 101
    for index in (19, 20, 21, 79, 80, 81):
        scan.ranges[index] = 1.0
        scan.intensities[index] = 9000.0
    return scan


def test_detect_leg_pair_reports_midpoint_and_separation():
    result = detect_leg_pair(
        _scan_with_two_legs(),
        intensity_threshold=8000.0,
        min_cluster_size=2,
        max_x_difference=0.75,
        min_leg_separation=0.25,
    )

    assert result is not None
    assert result.frame_id == "laser"
    assert result.left_x == pytest.approx(math.cos(-0.29))
    assert result.left_y == pytest.approx(math.sin(-0.29))
    assert result.right_x == pytest.approx(math.cos(0.29))
    assert result.right_y == pytest.approx(math.sin(0.29))
    assert result.midpoint_y == pytest.approx(0.0, abs=0.02)
    assert result.euclidean_separation > 0.5
    assert result.euclidean_separation == pytest.approx(
        math.hypot(
            result.right_x - result.left_x,
            result.right_y - result.left_y,
        )
    )
    assert result.outer_separation > result.center_separation
    assert result.center_separation > result.euclidean_separation
    assert result.midpoint_bearing == pytest.approx(0.0, abs=0.02)
    assert result.shelf_normal_yaw == pytest.approx(0.0, abs=0.02)


def test_shelf_normal_is_distinct_from_midpoint_bearing():
    midpoint_bearing = math.atan2(-0.46, 1.20)
    normal_yaw = shelf_normal_yaw(1.15, -0.80, 1.25, -0.13)

    assert midpoint_bearing == pytest.approx(-0.366, abs=0.01)
    assert normal_yaw == pytest.approx(-0.148, abs=0.01)
    assert normal_yaw != pytest.approx(midpoint_bearing, abs=0.05)


def test_shelf_normal_selects_the_branch_toward_midpoint():
    normal_yaw = shelf_normal_yaw(1.231, 0.412, 1.174, -0.254)

    assert normal_yaw == pytest.approx(-0.085, abs=0.01)
    midpoint_x = (1.231 + 1.174) / 2.0
    midpoint_y = (0.412 - 0.254) / 2.0
    assert (
        math.cos(normal_yaw) * midpoint_x
        + math.sin(normal_yaw) * midpoint_y
    ) > 0.0


def test_shelf_normal_is_invariant_to_leg_endpoint_order():
    forward = shelf_normal_yaw(1.231, 0.412, 1.174, -0.254)
    reversed_order = shelf_normal_yaw(1.174, -0.254, 1.231, 0.412)

    assert reversed_order == pytest.approx(forward)


def test_below_threshold_scan_returns_no_measurement():
    scan = _scan_with_two_legs()
    scan.intensities = [0.0] * len(scan.intensities)
    assert detect_leg_pair(scan, 8000.0, 2, 0.75, 0.25) is None


def test_observer_source_has_no_publishers_or_mutation_clients():
    source = (
        Path(__file__).parents[1]
        / "shelf_detection_server"
        / "geometry_observer.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }

    assert "create_publisher" not in called_attributes
    assert "create_client" not in called_attributes
    assert "/cmd_vel" not in source
    assert "/elevator_up" not in source
    assert "set_parameters" not in source
