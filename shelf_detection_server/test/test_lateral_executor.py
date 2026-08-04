import math
from pathlib import Path
from types import SimpleNamespace
import sys
import time

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.server import ShelfDetectionServer


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _ExecutorHarness:
    def __init__(
        self,
        recovered_targets,
        stable_yaws,
        failed_rotation_index=None,
        failed_drive=False,
    ):
        self.parameters = {
            "lateral_rotate_speed": 0.05,
            "lateral_drive_speed": 0.05,
            "lateral_clearance_min_range": 0.90,
            "max_detected_yaw": 0.60,
        }
        self.recovered_targets = list(recovered_targets)
        self.stable_yaws = list(stable_yaws)
        self.rotations = []
        self.drives = []
        self.stops = 0
        self.sequence = 0
        self.failed_rotation_index = failed_rotation_index
        self.failed_drive = failed_drive

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return _Logger()

    def _current_scan_sequence(self):
        self.sequence += 1
        return self.sequence

    def _rotate_measured(self, yaw, _deadline, speed=None):
        self.rotations.append((yaw, speed))
        return len(self.rotations) != self.failed_rotation_index

    def _drive_forward_measured(self, distance, _deadline, speed=None):
        self.drives.append((distance, speed))
        return not self.failed_drive

    def _publish_stop(self):
        self.stops += 1

    def _wait_for_stable_odom_yaw(self, _deadline):
        if not self.stable_yaws:
            return None
        return self.stable_yaws.pop(0)

    def _recover_cart_frame_after_motion(self, _sequence, _deadline):
        if not self.recovered_targets:
            return None
        return self.recovered_targets.pop(0)

    def _lateral_clearance_accepted(self, target):
        return ShelfDetectionServer._lateral_clearance_accepted(
            self, target
        )

    def _lateral_action_clearance_accepted(
        self, target, signed_yaw, drive_distance
    ):
        return ShelfDetectionServer._lateral_action_clearance_accepted(
            self, target, signed_yaw, drive_distance
        )

    def _settle_and_refresh_lateral(
        self, evidence, scan_before_motion, deadline
    ):
        return ShelfDetectionServer._settle_and_refresh_lateral(
            self, evidence, scan_before_motion, deadline
        )

    def execute(self, target, yaw, distance, entry_yaw=0.0):
        return ShelfDetectionServer._execute_lateral_action(
            self,
            target,
            yaw,
            distance,
            entry_yaw,
            time.monotonic() + 5.0,
        )


def test_lateral_executor_turn_drive_restore_with_fresh_rechecks():
    targets = [
        ("robot_base_footprint", 1.02, 0.12, 0.40),
        ("robot_base_footprint", 1.01, 0.06, 0.42),
        ("robot_base_footprint", 1.00, 0.05, 0.01),
    ]
    harness = _ExecutorHarness(
        targets,
        stable_yaws=[math.pi / 6, math.pi / 6, math.pi / 6, 0.0],
    )

    result = harness.execute(
        targets[0], math.pi / 6, 0.12, entry_yaw=0.0
    )

    assert result == targets[-1]
    assert len(harness.rotations) == 2
    assert harness.rotations[0] == (pytest.approx(math.pi / 6), 0.05)
    assert harness.rotations[1] == (pytest.approx(-math.pi / 6), 0.05)
    assert harness.drives == [(0.12, 0.05)]
    assert harness.stops == 3
    assert harness.recovered_targets == []


def test_lateral_executor_fails_closed_before_drive_without_fresh_scan():
    harness = _ExecutorHarness([], stable_yaws=[math.pi / 6])

    result = harness.execute(
        ("robot_base_footprint", 1.0, 0.12, 0.0),
        math.pi / 6,
        0.12,
    )

    assert result is None
    assert harness.rotations == [(math.pi / 6, 0.05)]
    assert harness.drives == []
    assert harness.stops == 1


def test_lateral_executor_fails_closed_when_first_rotation_fails():
    harness = _ExecutorHarness(
        [],
        stable_yaws=[],
        failed_rotation_index=1,
    )

    result = harness.execute(
        ("robot_base_footprint", 1.0, 0.12, 0.0),
        math.pi / 6,
        0.12,
    )

    assert result is None
    assert harness.rotations == [(math.pi / 6, 0.05)]
    assert harness.drives == []
    assert harness.recovered_targets == []


def test_lateral_executor_fails_closed_after_drive_without_restore():
    target_after_rotation = (
        "robot_base_footprint",
        math.cos(math.pi / 6),
        math.sin(math.pi / 6),
        math.pi / 6,
    )
    harness = _ExecutorHarness(
        [target_after_rotation],
        stable_yaws=[math.pi / 6],
        failed_drive=True,
    )

    result = harness.execute(
        ("robot_base_footprint", 1.0, 0.12, 0.0),
        math.pi / 6,
        0.12,
    )

    assert result is None
    assert harness.rotations == [(math.pi / 6, 0.05)]
    assert harness.drives == [(0.12, 0.05)]
    assert harness.recovered_targets == []


def test_lateral_clearance_rejects_close_or_invalid_geometry():
    harness = _ExecutorHarness([], [])

    assert harness._lateral_clearance_accepted(
        (
            "robot_base_footprint",
            0.90 * math.cos(math.pi / 6),
            0.90 * math.sin(math.pi / 6),
            0.60,
        )
    )
    assert not harness._lateral_clearance_accepted(
        ("robot_base_footprint", 0.89, 0.0, 0.0)
    )
    assert not harness._lateral_clearance_accepted(
        ("robot_base_footprint", 1.0, math.nan, 0.0)
    )
    assert not harness._lateral_clearance_accepted(
        ("robot_base_footprint", 1.0, 0.0, 0.61)
    )


def test_lateral_action_clearance_checks_predicted_endpoint():
    harness = _ExecutorHarness([], [])
    target = ("robot_base_footprint", 1.0, 0.18, 0.0)

    assert not harness._lateral_action_clearance_accepted(
        target, math.pi / 6, 0.18
    )
    assert harness._lateral_action_clearance_accepted(
        ("robot_base_footprint", 1.20, 0.18, 0.0),
        math.pi / 6,
        0.18,
    )
    assert not harness._lateral_action_clearance_accepted(
        target, math.nan, 0.18
    )
