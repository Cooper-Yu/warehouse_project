import ast
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.server import (  # noqa: E402
    c9_center_lock_enabled,
    c9_locked_drive_distance,
    c9_pre_lock_yaw_correction,
    c9_yaw_correction_enabled,
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

    assert declared["yaw_correction_steps"] == 3
    assert declared["min_yaw_correction_distance"] == 0.55
    assert declared["center_lock_distance"] == 0.35
    assert declared["center_lock_min_steps"] == 2
    assert declared["center_drive_scale"] == 1.0
    assert declared["cart_frame_retry_count"] == 6
    assert declared["pre_lock_max_yaw_correction"] == 0.08
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
    assert "_apply_pre_lock_yaw" in calls
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


def test_late_yaw_is_disabled_but_early_yaw_is_allowed():
    assert c9_yaw_correction_enabled(1, 1.272, 3, 0.55)
    assert not c9_yaw_correction_enabled(3, 1.263, 3, 0.55)
    assert not c9_yaw_correction_enabled(2, 0.50, 3, 0.55)


def test_last_trusted_cloud_sample_gets_one_capped_pre_lock_correction():
    correction = c9_pre_lock_yaw_correction(-0.091, 0.4, 0.08)
    assert correction == pytest.approx(-0.0364)
    assert c9_pre_lock_yaw_correction(1.0, 0.4, 0.08) == 0.08
    assert c9_pre_lock_yaw_correction(-1.0, 0.4, 0.08) == -0.08


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


def test_heading_observability_does_not_replace_midpoint_yaw_control():
    tree = _server_tree()
    attach = _function(tree, "_perform_stepwise_attach")
    assignments = [
        node
        for node in ast.walk(attach)
        if isinstance(node, ast.Assign)
    ]

    yaw_assignment = next(
        node
        for node in assignments
        if any(
            isinstance(target, ast.Name) and target.id == "yaw"
            for target in node.targets
        )
    )
    assert isinstance(yaw_assignment.value, ast.Call)
    assert isinstance(yaw_assignment.value.func, ast.Attribute)
    assert yaw_assignment.value.func.attr == "atan2"
