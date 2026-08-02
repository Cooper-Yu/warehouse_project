import ast
import math
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import LaserScan


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.leg_geometry import detect_leg_pair  # noqa: E402


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
    assert result.midpoint_y == pytest.approx(0.0, abs=0.02)
    assert result.euclidean_separation > 0.5
    assert result.euclidean_separation == pytest.approx(
        math.hypot(
            result.right_x - result.left_x,
            result.right_y - result.left_y,
        )
    )


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
