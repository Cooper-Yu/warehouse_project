import ast
import math
from pathlib import Path
import sys
import time

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.server import (  # noqa: E402
    bounded_yaw_correction,
    c9_center_lock_enabled,
    c9_locked_drive_distance,
    shelf_heading_aligned,
    shelf_staging_error,
)


def _server_tree():
    source = (
        Path(__file__).parents[1]
        / "shelf_detection_server"
        / "server.py"
    )
    return ast.parse(source.read_text(encoding="utf-8"))


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _AlignmentHarness:
    def __init__(self, recovered_targets, retry_count=6):
        self.parameters = {
            "alignment_retry_count": retry_count,
            "alignment_standoff_distance": 1.0,
            "alignment_position_tolerance": 0.08,
            "alignment_max_drive_distance": 0.75,
            "yaw_tolerance": 0.03,
            "max_detected_yaw": 0.60,
            "alignment_heading_gain": 1.0,
            "alignment_max_yaw_correction": 0.40,
            "movement_timeout": 55.0,
        }
        self.recovered_targets = list(recovered_targets)
        self.rotations = []
        self.drives = []
        self.stop_count = 0
        self.elevator_count = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _current_scan_sequence(self):
        return 1

    def _rotate_open_loop(self, yaw, _deadline):
        self.rotations.append(yaw)
        return True

    def _drive_forward_measured(self, distance, _deadline):
        self.drives.append(distance)
        return True

    def _recover_cart_frame_after_motion(self, _sequence, _deadline):
        return self.recovered_targets.pop(0)

    def _publish_stop(self):
        self.stop_count += 1

    def _publish_elevator_up(self):
        self.elevator_count += 1

    def _align_at_safe_standoff(self, target, deadline):
        from shelf_detection_server.server import ShelfDetectionServer

        return ShelfDetectionServer._align_at_safe_standoff(
            self, target, deadline
        )


def test_c9_policy_parameters_are_declared():
    tree = _server_tree()
    declared = {
        node.args[0].value: node.args[1].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
        and len(node.args) >= 2
        and isinstance(node.args[0], ast.Constant)
        and isinstance(node.args[1], ast.Constant)
    }

    assert declared["alignment_heading_gain"] == 1.0
    assert declared["alignment_max_yaw_correction"] == 0.40
    assert declared["alignment_standoff_distance"] == 1.00
    assert declared["alignment_position_tolerance"] == 0.08
    assert declared["alignment_retry_count"] == 6
    assert declared["alignment_max_drive_distance"] == 0.75
    assert declared["movement_timeout"] == 55.0
    assert declared["center_lock_distance"] == 0.35
    assert declared["center_lock_min_steps"] == 2
    assert declared["center_drive_scale"] == 1.0
    assert declared["cart_frame_retry_count"] == 6
    assert declared["odom_frame"] == "odom"
    assert declared["odom_lookup_timeout"] == 1.0
    assert declared["measured_drive_timeout_scale"] == 3.0


def test_attach_loop_contains_center_lock_and_recovery_paths():
    tree = _server_tree()
    attach = _function(tree, "_perform_stepwise_attach")
    calls = {
        node.func.attr
        for node in ast.walk(attach)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }

    assert "_recover_cart_frame_after_motion" in calls
    assert "_drive_forward_measured" in calls
    assert "_align_at_safe_standoff" in calls
    assert "_publish_elevator_up" in calls


def test_recovery_is_bounded_by_retry_count():
    tree = _server_tree()
    recovery = _function(tree, "_recover_cart_frame_after_motion")

    assert any(
        isinstance(node, ast.For)
        and isinstance(node.iter, ast.Call)
        and isinstance(node.iter.func, ast.Name)
        and node.iter.func.id == "range"
        for node in ast.walk(recovery)
    )


def test_cloud_failure_sample_enters_c9_center_lock():
    assert c9_center_lock_enabled(
        step=32,
        x=0.292,
        min_steps=2,
        lock_distance=0.35,
    )
    assert c9_locked_drive_distance(0.292, 1.0) == pytest.approx(0.292)


def test_collision_sample_requires_staging_and_heading_alignment():
    error_x, error_y = shelf_staging_error(
        midpoint_x=1.460,
        midpoint_y=0.176,
        shelf_heading=-0.066,
        standoff_distance=1.0,
    )

    assert error_x == pytest.approx(0.462, abs=0.01)
    assert error_y == pytest.approx(0.242, abs=0.01)
    assert not shelf_heading_aligned(-0.066, 0.03)


def test_staging_vector_reaches_centered_pose_in_ideal_kinematics():
    midpoint_x = 1.460
    midpoint_y = 0.176
    shelf_heading = -0.066
    error_x, error_y = shelf_staging_error(
        midpoint_x, midpoint_y, shelf_heading, 1.0
    )
    travel_yaw = math.atan2(error_y, error_x)

    remaining_x = midpoint_x - error_x
    remaining_y = midpoint_y - error_y
    new_midpoint_x = (
        math.cos(travel_yaw) * remaining_x
        + math.sin(travel_yaw) * remaining_y
    )
    new_midpoint_y = (
        -math.sin(travel_yaw) * remaining_x
        + math.cos(travel_yaw) * remaining_y
    )
    new_shelf_heading = shelf_heading - travel_yaw
    final_error = shelf_staging_error(
        new_midpoint_x,
        new_midpoint_y,
        new_shelf_heading,
        1.0,
    )

    assert final_error == pytest.approx((0.0, 0.0), abs=1e-9)


def test_alignment_sequence_reobserves_until_staging_and_heading_pass():
    staging_x, staging_y = shelf_staging_error(
        1.460, 0.176, -0.066, 1.0
    )
    travel_yaw = math.atan2(staging_y, staging_x)
    heading_after_staging = -0.066 - travel_yaw
    heading_after_first_correction = heading_after_staging + 0.40
    targets = [
        (
            "base",
            math.cos(heading_after_staging),
            math.sin(heading_after_staging),
            heading_after_staging,
        ),
        (
            "base",
            math.cos(heading_after_first_correction),
            math.sin(heading_after_first_correction),
            heading_after_first_correction,
        ),
        ("base", 1.0, 0.0, 0.0),
    ]
    harness = _AlignmentHarness(targets)

    result = harness._align_at_safe_standoff(
        ("base", 1.460, 0.176, -0.066),
        time.monotonic() + 10.0,
    )

    assert result == ("base", 1.0, 0.0, 0.0)
    assert harness.drives == pytest.approx(
        [math.hypot(staging_x, staging_y)]
    )
    assert harness.rotations == pytest.approx(
        [travel_yaw, -0.40, heading_after_first_correction]
    )


def test_alignment_exhaustion_stops_and_blocks_attach_elevator():
    harness = _AlignmentHarness(
        [("base", 1.460, 0.176, -0.066)], retry_count=1
    )

    result = harness._align_at_safe_standoff(
        ("base", 1.460, 0.176, -0.066),
        time.monotonic() + 10.0,
    )

    assert result is None
    assert harness.stop_count == 1

    from shelf_detection_server.server import ShelfDetectionServer

    harness._align_at_safe_standoff = lambda _target, _deadline: None
    attached = ShelfDetectionServer._perform_stepwise_attach(
        harness, ("base", 1.460, 0.176, -0.066)
    )
    assert not attached
    assert harness.elevator_count == 0


def test_heading_correction_uses_shelf_normal_and_is_capped():
    assert bounded_yaw_correction(-0.066, 1.0, 0.40) == pytest.approx(
        -0.066
    )
    assert bounded_yaw_correction(1.0, 1.0, 0.40) == 0.40
    assert bounded_yaw_correction(-1.0, 1.0, 0.40) == -0.40
    assert shelf_heading_aligned(0.03, 0.03)
    assert not shelf_heading_aligned(0.031, 0.03)


def test_measured_drive_reads_odom_and_always_stops():
    tree = _server_tree()
    measured = _function(tree, "_drive_forward_measured")
    calls = {
        node.func.attr
        for node in ast.walk(measured)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }

    assert "_wait_for_odom_xy" in calls
    assert "_lookup_odom_xy" in calls
    assert "_publish_stop" in calls
    assert any(isinstance(node, ast.Try) for node in ast.walk(measured))


def test_final_measured_drive_precedes_elevator_publish():
    tree = _server_tree()
    attach = _function(tree, "_perform_stepwise_attach")
    method_calls = [
        node
        for node in ast.walk(attach)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]
    measured_lines = [
        node.lineno
        for node in method_calls
        if node.func.attr == "_drive_forward_measured"
    ]
    elevator_line = next(
        node.lineno
        for node in method_calls
        if node.func.attr == "_publish_elevator_up"
    )

    assert measured_lines
    assert max(measured_lines) < elevator_line


def test_safe_standoff_alignment_reobserves_after_each_motion():
    tree = _server_tree()
    alignment = _function(tree, "_align_at_safe_standoff")
    calls = {
        node.func.attr
        for node in ast.walk(alignment)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }
    named_calls = {
        node.func.id
        for node in ast.walk(alignment)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    }

    assert "_rotate_open_loop" in calls
    assert "_drive_forward_measured" in calls
    assert "_recover_cart_frame_after_motion" in calls
    assert "shelf_staging_error" in named_calls
    assert "bounded_yaw_correction" in named_calls


def test_attach_calls_alignment_before_any_entry_drive():
    tree = _server_tree()
    attach = _function(tree, "_perform_stepwise_attach")
    method_calls = [
        node
        for node in ast.walk(attach)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]
    alignment_line = next(
        node.lineno
        for node in method_calls
        if node.func.attr == "_align_at_safe_standoff"
    )
    drive_lines = [
        node.lineno
        for node in method_calls
        if node.func.attr == "_drive_forward_measured"
    ]

    assert drive_lines
    assert alignment_line < min(drive_lines)
