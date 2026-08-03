import ast
import math
from pathlib import Path
import sys
import time

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.server import (  # noqa: E402
    alignment_yaw_command,
    bounded_yaw_correction,
    c9_center_lock_enabled,
    c9_locked_drive_distance,
    normalize_angle,
    planar_yaw_from_quaternion,
    shelf_heading_aligned,
    shelf_staging_error,
    staging_motion_command,
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

    def warning(self, _message):
        pass


class _AlignmentHarness:
    def __init__(
        self,
        recovered_targets,
        retry_count=6,
        stable_yaws=None,
        failed_rotation_index=None,
    ):
        self.parameters = {
            "rotate_speed": 0.20,
            "alignment_retry_count": retry_count,
            "alignment_standoff_distance": 1.0,
            "alignment_position_tolerance": 0.08,
            "alignment_max_drive_distance": 0.75,
            "alignment_max_travel_yaw": 1.20,
            "alignment_short_drive_distance": 0.20,
            "alignment_short_forward_speed": 0.05,
            "alignment_max_reverse_distance": 0.15,
            "alignment_max_reverse_yaw": 1.20,
            "alignment_reverse_speed": 0.03,
            "yaw_tolerance": 0.03,
            "max_detected_yaw": 0.60,
            "alignment_heading_gain": 1.0,
            "alignment_max_yaw_correction": 0.40,
            "alignment_fine_yaw_threshold": 0.20,
            "alignment_fine_heading_gain": 0.50,
            "alignment_fine_rotate_speed": 0.05,
            "alignment_settle_timeout": 2.0,
            "alignment_settle_sample_count": 3,
            "alignment_settle_yaw_tolerance": 0.01,
            "alignment_required_consecutive_samples": 2,
            "movement_timeout": 75.0,
        }
        self.recovered_targets = list(recovered_targets)
        self.rotations = []
        self.drives = []
        self.drive_speeds = []
        self.stop_count = 0
        self.elevator_count = 0
        self.accepted_odom_yaw = 3.053559
        self.settle_count = 0
        self.stable_yaws = list(stable_yaws or [])
        self.failed_rotation_index = failed_rotation_index

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _current_scan_sequence(self):
        return 1

    def _rotate_measured(self, yaw, _deadline, _speed_override=None):
        self.rotations.append(yaw)
        return len(self.rotations) != self.failed_rotation_index

    def _drive_forward_measured(
        self, distance, _deadline, speed_override=None
    ):
        self.drives.append(distance)
        self.drive_speeds.append(speed_override)
        return True

    def _recover_cart_frame_after_motion(self, _sequence, _deadline):
        return self.recovered_targets.pop(0)

    def _publish_stop(self):
        self.stop_count += 1

    def _publish_elevator_up(self):
        self.elevator_count += 1

    def _wait_for_odom_yaw(self, _deadline):
        return self.accepted_odom_yaw

    def _wait_for_stable_odom_yaw(self, _deadline):
        self.settle_count += 1
        if self.stable_yaws:
            return self.stable_yaws.pop(0)
        return self.accepted_odom_yaw

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
    assert declared["alignment_fine_yaw_threshold"] == 0.20
    assert declared["alignment_fine_heading_gain"] == 0.50
    assert declared["alignment_fine_rotate_speed"] == 0.05
    assert declared["alignment_standoff_distance"] == 1.00
    assert declared["alignment_position_tolerance"] == 0.08
    assert declared["alignment_retry_count"] == 6
    assert declared["alignment_max_drive_distance"] == 0.75
    assert declared["alignment_max_travel_yaw"] == 1.20
    assert declared["alignment_short_drive_distance"] == 0.20
    assert declared["alignment_short_forward_speed"] == 0.05
    assert declared["alignment_max_reverse_distance"] == 0.15
    assert declared["alignment_max_reverse_yaw"] == 1.20
    assert declared["alignment_reverse_speed"] == 0.03
    assert declared["movement_timeout"] == 75.0
    assert declared["center_lock_distance"] == 0.35
    assert declared["center_lock_min_steps"] == 2
    assert declared["center_drive_scale"] == 1.0
    assert declared["cart_frame_retry_count"] == 6
    assert declared["odom_topic"] == "/odom"
    assert declared["odom_frame"] == "odom"
    assert declared["odom_lookup_timeout"] == 1.0
    assert declared["measured_drive_timeout_scale"] == 3.0
    assert declared["measured_rotation_timeout_scale"] == 3.0
    assert declared["measured_yaw_tolerance"] == 0.01
    assert declared["alignment_settle_timeout"] == 2.0
    assert declared["alignment_settle_sample_count"] == 3
    assert declared["alignment_settle_yaw_tolerance"] == 0.01
    assert declared["alignment_required_consecutive_samples"] == 2
    assert declared["final_drive_distance"] == pytest.approx(0.3703)
    assert declared["entry_odom_yaw_tolerance"] == 0.03


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
    assert "_accepted_odom_heading_ok" in calls
    assert "_publish_elevator_up" in calls

    named_calls = {
        node.func.id
        for node in ast.walk(attach)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    }
    assert "shelf_heading_aligned" not in named_calls


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
    restored_heading = -0.066
    heading_after_first_correction = restored_heading + 0.033
    heading_after_second_correction = (
        heading_after_first_correction + 0.0165
    )
    targets = [
        (
            "base",
            math.cos(restored_heading),
            math.sin(restored_heading),
            restored_heading,
        ),
        (
            "base",
            math.cos(heading_after_first_correction),
            math.sin(heading_after_first_correction),
            heading_after_first_correction,
        ),
        (
            "base",
            math.cos(heading_after_second_correction),
            math.sin(heading_after_second_correction),
            heading_after_second_correction,
        ),
        (
            "base",
            math.cos(heading_after_second_correction),
            math.sin(heading_after_second_correction),
            heading_after_second_correction,
        ),
    ]
    harness = _AlignmentHarness(
        targets,
        stable_yaws=[
            0.0,
            travel_yaw,
            travel_yaw,
            0.0,
            0.0,
            0.0,
            3.053559,
            3.053559,
        ],
    )

    result = harness._align_at_safe_standoff(
        ("base", 1.460, 0.176, -0.066),
        time.monotonic() + 10.0,
    )

    assert result == (
        targets[-1],
        harness.accepted_odom_yaw,
    )
    assert harness.drives == pytest.approx(
        [math.hypot(staging_x, staging_y)]
    )
    assert harness.rotations == pytest.approx(
        [travel_yaw, -travel_yaw, -0.033, -0.0165]
    )
    assert harness.settle_count == 8


def test_staging_restores_pre_motion_odom_heading_before_reobservation():
    error_x, error_y = shelf_staging_error(
        1.397, 0.235, -0.004, 1.0
    )
    travel_yaw = math.atan2(error_y, error_x)
    restored_target = ("base", 1.0, 0.0, 0.0)
    harness = _AlignmentHarness(
        [restored_target, restored_target],
        stable_yaws=[
            -3.068,
            -2.527,
            -2.527,
            -3.068,
            3.053559,
            3.053559,
        ],
    )

    result = harness._align_at_safe_standoff(
        ("base", 1.397, 0.235, -0.004),
        time.monotonic() + 10.0,
    )

    assert result == (restored_target, harness.accepted_odom_yaw)
    assert harness.drives == pytest.approx(
        [math.hypot(error_x, error_y)]
    )
    assert harness.rotations == pytest.approx(
        [travel_yaw, -0.541], abs=0.01
    )
    assert harness.settle_count == 6


def test_staging_restore_failure_stops_before_reobservation():
    error_x, error_y = shelf_staging_error(
        1.397, 0.235, -0.004, 1.0
    )
    travel_yaw = math.atan2(error_y, error_x)
    harness = _AlignmentHarness(
        [],
        stable_yaws=[0.0, travel_yaw, travel_yaw],
        failed_rotation_index=2,
    )

    result = harness._align_at_safe_standoff(
        ("base", 1.397, 0.235, -0.004),
        time.monotonic() + 10.0,
    )

    assert result is None
    assert harness.drives == pytest.approx(
        [math.hypot(error_x, error_y)]
    )
    assert harness.rotations == pytest.approx(
        [travel_yaw, -travel_yaw]
    )
    assert harness.recovered_targets == []
    assert harness.elevator_count == 0


def test_cloud_lateral_residual_uses_independent_yaw_and_short_speed():
    shelf_heading = -0.059
    error_x, error_y = shelf_staging_error(
        1.062, -0.188, shelf_heading, 1.0
    )
    travel_yaw = math.atan2(error_y, error_x)
    aligned_target = ("base", 1.0, 0.0, 0.0)
    harness = _AlignmentHarness(
        [aligned_target, aligned_target],
        stable_yaws=[
            0.0,
            travel_yaw,
            travel_yaw,
            0.0,
            3.053559,
            3.053559,
        ],
    )

    result = harness._align_at_safe_standoff(
        ("base", 1.062, -0.188, shelf_heading),
        time.monotonic() + 10.0,
    )

    assert abs(travel_yaw) == pytest.approx(1.113, abs=0.01)
    assert result == (aligned_target, harness.accepted_odom_yaw)
    assert harness.drives == pytest.approx([math.hypot(error_x, error_y)])
    assert harness.drive_speeds == [0.05]
    assert harness.rotations == pytest.approx(
        [travel_yaw, -travel_yaw]
    )


def test_staging_travel_yaw_still_has_an_independent_bound():
    error_distance = 0.20
    travel_yaw = 1.30
    harness = _AlignmentHarness([])

    result = harness._align_at_safe_standoff(
        (
            "base",
            1.0 + error_distance * math.cos(travel_yaw),
            error_distance * math.sin(travel_yaw),
            0.0,
        ),
        time.monotonic() + 10.0,
    )

    assert result is None
    assert harness.rotations == []
    assert harness.drives == []


def test_run2_side_rear_residual_uses_bounded_reverse_equivalent():
    shelf_heading = -0.005
    error_x, error_y = shelf_staging_error(
        0.944, 0.078, shelf_heading, 1.0
    )
    forward_yaw = math.atan2(error_y, error_x)
    reverse_yaw = normalize_angle(forward_yaw + math.pi)
    distance = math.hypot(error_x, error_y)
    aligned_target = ("base", 1.0, 0.0, 0.0)
    harness = _AlignmentHarness(
        [aligned_target, aligned_target],
        stable_yaws=[
            0.0,
            reverse_yaw,
            reverse_yaw,
            0.0,
            3.053559,
            3.053559,
        ],
    )

    result = harness._align_at_safe_standoff(
        ("base", 0.944, 0.078, shelf_heading),
        time.monotonic() + 10.0,
    )

    assert forward_yaw == pytest.approx(2.161, abs=0.01)
    assert reverse_yaw == pytest.approx(-0.981, abs=0.01)
    assert result == (aligned_target, harness.accepted_odom_yaw)
    assert harness.drives == pytest.approx([-distance])
    assert harness.drive_speeds == [0.03]
    assert harness.rotations == pytest.approx(
        [reverse_yaw, -reverse_yaw]
    )


def test_reverse_equivalent_rejects_distance_above_independent_bound():
    travel_yaw = 2.20
    distance = 0.16

    assert staging_motion_command(
        distance * math.cos(travel_yaw),
        distance * math.sin(travel_yaw),
        max_forward_yaw=1.20,
        max_reverse_distance=0.15,
        max_reverse_yaw=1.20,
    ) is None


def test_reverse_equivalent_rejects_shelf_inside_safe_standoff():
    shelf_heading = 0.55
    travel_yaw = 2.20
    distance = 0.10
    midpoint_x = math.cos(shelf_heading) + distance * math.cos(
        travel_yaw
    )
    midpoint_y = math.sin(shelf_heading) + distance * math.sin(
        travel_yaw
    )
    harness = _AlignmentHarness([])

    result = harness._align_at_safe_standoff(
        ("base", midpoint_x, midpoint_y, shelf_heading),
        time.monotonic() + 10.0,
    )

    assert midpoint_x < 0.85
    assert result is None
    assert harness.rotations == []
    assert harness.drives == []


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


def test_alignment_rejects_missing_accepted_odom_yaw():
    harness = _AlignmentHarness([])
    harness.accepted_odom_yaw = None

    result = harness._align_at_safe_standoff(
        ("base", 1.0, 0.0, 0.0),
        time.monotonic() + 10.0,
    )

    assert result is None
    assert harness.stop_count == 1


class _SettlingHarness:
    def __init__(self, samples):
        self.parameters = {
            "alignment_settle_timeout": 1.0,
            "alignment_settle_sample_count": 3,
            "alignment_settle_yaw_tolerance": 0.01,
        }
        self.samples = list(samples)
        self.stop_count = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _publish_stop(self):
        self.stop_count += 1

    def _lookup_odom_yaw_sample(self):
        if not self.samples:
            return None
        return self.samples.pop(0)

    def settle(self, deadline):
        from shelf_detection_server.server import ShelfDetectionServer

        return ShelfDetectionServer._wait_for_stable_odom_yaw(
            self, deadline
        )


def test_post_rotation_settling_rejects_motion_then_accepts_stable_yaw(
    monkeypatch,
):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _SettlingHarness(
        [
            (1, 3.081),
            (2, 3.035),
            (3, 2.989),
            (4, 2.989),
            (5, 2.989),
        ]
    )
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert harness.settle(10.0) == pytest.approx(2.989)
    assert harness.stop_count == 1


def test_post_rotation_settling_requires_fresh_timestamps(monkeypatch):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _SettlingHarness(
        [(1, 2.989), (1, 2.989), (1, 2.989)]
    )
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert harness.settle(10.0) is None
    assert harness.stop_count == 2


def test_post_rotation_settling_is_wrap_safe(monkeypatch):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _SettlingHarness(
        [(1, 3.140), (2, -3.139), (3, -3.138)]
    )
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert harness.settle(10.0) == pytest.approx(-3.138)


def test_odom_callback_caches_stamped_yaw_for_settling():
    from shelf_detection_server.server import ShelfDetectionServer

    class _Stamp:
        sec = 12
        nanosec = 345

    class _Header:
        stamp = _Stamp()

    class _Orientation:
        x = 0.0
        y = 0.0
        z = math.sin(-0.4 / 2.0)
        w = math.cos(-0.4 / 2.0)

    class _PoseValue:
        orientation = _Orientation()

    class _Pose:
        pose = _PoseValue()

    class _Odom:
        header = _Header()
        pose = _Pose()

    class _OdomHarness:
        def __init__(self):
            import threading

            self._odom_lock = threading.Lock()
            self._latest_odom_yaw_sample = None

    harness = _OdomHarness()
    ShelfDetectionServer._odom_callback(harness, _Odom())

    assert ShelfDetectionServer._lookup_odom_yaw_sample(harness) == (
        12_000_000_345,
        pytest.approx(-0.4),
    )


def test_server_subscribes_to_raw_odom_for_settle_samples():
    tree = _server_tree()
    init = _function(tree, "__init__")
    callback = _function(tree, "_odom_callback")
    lookup = _function(tree, "_lookup_odom_yaw_sample")

    init_names = {
        node.id for node in ast.walk(init) if isinstance(node, ast.Name)
    }
    callback_attributes = {
        node.attr
        for node in ast.walk(callback)
        if isinstance(node, ast.Attribute)
    }
    lookup_calls = {
        node.attr
        for node in ast.walk(lookup)
        if isinstance(node, ast.Attribute)
        and isinstance(node.ctx, ast.Load)
    }

    assert "Odometry" in init_names
    assert "stamp" in callback_attributes
    assert "orientation" in callback_attributes
    assert "lookup_transform" not in lookup_calls


def test_heading_correction_uses_shelf_normal_and_is_capped():
    assert bounded_yaw_correction(-0.066, 1.0, 0.40) == pytest.approx(
        -0.066
    )
    assert bounded_yaw_correction(1.0, 1.0, 0.40) == 0.40
    assert bounded_yaw_correction(-1.0, 1.0, 0.40) == -0.40
    assert shelf_heading_aligned(0.03, 0.03)
    assert not shelf_heading_aligned(0.031, 0.03)


@pytest.mark.parametrize(
    "yaw,expected_correction,expected_speed,expected_regime",
    [
        (0.043, 0.0215, 0.05, "fine"),
        (-0.170, -0.085, 0.05, "fine"),
        (0.151, 0.0755, 0.05, "fine"),
        (-0.460, -0.40, 0.20, "coarse"),
    ],
)
def test_alignment_yaw_command_damps_cloud_oscillation_samples(
    yaw,
    expected_correction,
    expected_speed,
    expected_regime,
):
    correction, speed, regime = alignment_yaw_command(
        yaw=yaw,
        coarse_gain=1.0,
        max_abs_correction=0.40,
        coarse_speed=0.20,
        fine_threshold=0.20,
        fine_gain=0.50,
        fine_speed=0.05,
    )

    assert correction == pytest.approx(expected_correction)
    assert speed == pytest.approx(expected_speed)
    assert regime == expected_regime


class _EntryHarness:
    def __init__(self, recovered_targets, current_yaw=3.053559):
        self.parameters = {
            "movement_timeout": 55.0,
            "max_detected_yaw": 0.60,
            "center_distance_tolerance": 0.20,
            "center_lateral_tolerance": 0.08,
            "center_lock_distance": 0.35,
            "center_lock_min_steps": 2,
            "center_drive_scale": 1.0,
            "forward_step_distance": 0.20,
            "final_drive_distance": 0.3703,
            "entry_odom_yaw_tolerance": 0.03,
        }
        self.accepted_yaw = 3.053559
        self.current_yaw = current_yaw
        self.recovered_targets = list(recovered_targets)
        self.drives = []
        self.stop_count = 0
        self.elevator_count = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _align_at_safe_standoff(self, target, _deadline):
        return target, self.accepted_yaw

    def _wait_for_odom_yaw(self, _deadline):
        return self.current_yaw

    def _accepted_odom_heading_ok(self, accepted_yaw, deadline):
        from shelf_detection_server.server import ShelfDetectionServer

        return ShelfDetectionServer._accepted_odom_heading_ok(
            self, accepted_yaw, deadline
        )

    def _publish_cart_frame(self, *_args):
        pass

    def _current_scan_sequence(self):
        return 1

    def _drive_forward_measured(self, distance, _deadline):
        self.drives.append(distance)
        return True

    def _recover_cart_frame_after_motion(self, _sequence, _deadline):
        return self.recovered_targets.pop(0)

    def _publish_stop(self):
        self.stop_count += 1

    def _publish_elevator_up(self):
        self.elevator_count += 1

    def attach(self, target):
        from shelf_detection_server.server import ShelfDetectionServer

        return ShelfDetectionServer._perform_stepwise_attach(self, target)


def test_cloud_close_range_heading_jump_uses_constant_odom_guard(
    monkeypatch,
):
    import shelf_detection_server.server as server_module

    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)
    harness = _EntryHarness(
        [
            ("base", 0.801, 0.055, 0.098),
            ("base", 0.200, 0.050, 0.200),
        ]
    )

    assert harness.attach(("base", 1.006, -0.024, 0.011))
    assert harness.drives == pytest.approx([0.20, 0.20, 0.3703])
    assert harness.elevator_count == 1


def test_constrained_entry_odom_drift_stops_before_drive_and_elevator(
    monkeypatch,
):
    import shelf_detection_server.server as server_module

    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)
    harness = _EntryHarness([], current_yaw=3.093559)

    assert not harness.attach(("base", 1.006, -0.024, 0.011))
    assert harness.stop_count == 1
    assert harness.drives == []
    assert harness.elevator_count == 0


def test_constrained_entry_stale_odom_stops_before_elevator(monkeypatch):
    import shelf_detection_server.server as server_module

    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)
    harness = _EntryHarness([], current_yaw=None)

    assert not harness.attach(("base", 1.006, -0.024, 0.011))
    assert harness.stop_count == 1
    assert harness.elevator_count == 0


def test_constrained_entry_odom_guard_is_wrap_safe():
    harness = _EntryHarness([], current_yaw=-3.13)
    harness.accepted_yaw = 3.13

    assert harness._accepted_odom_heading_ok(
        harness.accepted_yaw, time.monotonic() + 1.0
    )
    assert harness.stop_count == 0


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


def test_measured_yaw_reads_odom_and_always_stops():
    tree = _server_tree()
    measured = _function(tree, "_rotate_measured")
    calls = {
        node.func.attr
        for node in ast.walk(measured)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    }
    named_calls = {
        node.func.id
        for node in ast.walk(measured)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    }

    assert "_wait_for_odom_yaw" in calls
    assert "_lookup_odom_yaw" in calls
    assert "_publish_stop" in calls
    assert "normalize_angle" in named_calls
    assert any(isinstance(node, ast.Try) for node in ast.walk(measured))


class _Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _DriveHarness:
    def __init__(self, xy_samples):
        self.parameters = {
            "forward_speed": 0.10,
            "measured_drive_timeout_scale": 3.0,
            "odom_lookup_timeout": 0.10,
        }
        self.xy_samples = list(xy_samples)
        self.publisher = _Publisher()
        self._cmd_vel_pub = self.publisher
        self.stop_count = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _wait_for_odom_xy(self, _deadline):
        return 0.0, 0.0

    def _lookup_odom_xy(self):
        if not self.xy_samples:
            return None
        return self.xy_samples.pop(0)

    def _publish_stop(self):
        self.stop_count += 1

    def drive(self, distance, deadline, speed_override=None):
        from shelf_detection_server.server import ShelfDetectionServer

        return ShelfDetectionServer._drive_forward_measured(
            self, distance, deadline, speed_override
        )


def test_measured_signed_reverse_uses_negative_cmd_and_always_stops(
    monkeypatch,
):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _DriveHarness([(0.03, 0.0), (0.07, 0.0), (0.11, 0.0)])
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert harness.drive(-0.10, deadline=10.0, speed_override=0.03)
    assert harness.stop_count == 1
    assert harness.publisher.messages
    assert all(
        message.linear.x == pytest.approx(-0.03)
        for message in harness.publisher.messages
    )


class _RotationHarness:
    def __init__(self, start_yaw, yaw_samples):
        self.parameters = {
            "rotate_speed": 0.20,
            "measured_yaw_tolerance": 0.01,
            "measured_rotation_timeout_scale": 3.0,
            "odom_lookup_timeout": 0.10,
        }
        self.start_yaw = start_yaw
        self.yaw_samples = list(yaw_samples)
        self.publisher = _Publisher()
        self._cmd_vel_pub = self.publisher
        self.stop_count = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _wait_for_odom_yaw(self, _deadline):
        return self.start_yaw

    def _lookup_odom_yaw(self):
        if not self.yaw_samples:
            return None
        return self.yaw_samples.pop(0)

    def _publish_stop(self):
        self.stop_count += 1

    def rotate(self, yaw, deadline, speed_override=None):
        from shelf_detection_server.server import ShelfDetectionServer

        return ShelfDetectionServer._rotate_measured(
            self, yaw, deadline, speed_override
        )


@pytest.mark.parametrize(
    "target,start,samples,command_sign",
    [
        (0.20, 3.10, [-3.12, -3.05, -2.98], 1.0),
        (-0.20, -3.10, [3.12, 3.05, 2.98], -1.0),
    ],
)
def test_measured_yaw_accumulates_wrap_safe_rotation(
    monkeypatch, target, start, samples, command_sign
):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _RotationHarness(start, samples)
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert harness.rotate(target, deadline=10.0)
    assert harness.stop_count == 1
    assert harness.publisher.messages
    assert all(
        math.copysign(1.0, message.angular.z) == command_sign
        for message in harness.publisher.messages
    )


def test_measured_yaw_stale_odom_stops_and_fails(monkeypatch):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _RotationHarness(0.0, [])
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert not harness.rotate(0.20, deadline=10.0)
    assert harness.stop_count == 1


def test_measured_yaw_uses_fine_speed_override(monkeypatch):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _RotationHarness(0.0, [0.01, 0.02, 0.03, 0.04, 0.05])
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert harness.rotate(0.05, deadline=10.0, speed_override=0.05)
    assert harness.publisher.messages
    assert all(
        message.angular.z == pytest.approx(0.05)
        for message in harness.publisher.messages
    )


def test_measured_yaw_unavailable_initial_odom_stops(monkeypatch):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _RotationHarness(None, [])
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert not harness.rotate(0.20, deadline=10.0)
    assert harness.stop_count == 1
    assert not harness.publisher.messages


def test_measured_yaw_deadline_stops_before_target(monkeypatch):
    import shelf_detection_server.server as server_module

    clock = _Clock()
    harness = _RotationHarness(0.0, [0.0] * 20)
    monkeypatch.setattr(server_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(server_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)

    assert not harness.rotate(0.20, deadline=0.20)
    assert harness.stop_count == 1
    assert harness.publisher.messages


def test_planar_yaw_from_quaternion():
    class Quaternion:
        x = 0.0
        y = 0.0
        z = math.sin(-0.70 / 2.0)
        w = math.cos(-0.70 / 2.0)

    assert planar_yaw_from_quaternion(Quaternion()) == pytest.approx(-0.70)


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
    guard_lines = [
        node.lineno
        for node in method_calls
        if node.func.attr == "_accepted_odom_heading_ok"
    ]
    elevator_line = next(
        node.lineno
        for node in method_calls
        if node.func.attr == "_publish_elevator_up"
    )

    assert measured_lines
    assert guard_lines
    assert max(measured_lines) < elevator_line
    assert max(guard_lines) < elevator_line


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

    assert "_rotate_measured" in calls
    assert "_drive_forward_measured" in calls
    assert "_recover_cart_frame_after_motion" in calls
    assert "_wait_for_stable_odom_yaw" in calls
    assert "shelf_staging_error" in named_calls
    assert "alignment_yaw_command" in named_calls


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
