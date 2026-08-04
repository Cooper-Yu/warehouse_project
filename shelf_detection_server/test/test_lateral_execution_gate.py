import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.lateral_action_plan import LateralActionPlan
from shelf_detection_server.lateral_execution_gate import (
    plan_lateral_action_if_safe,
)


VALID_ARGS = dict(
    lateral_error=0.12,
    correction_ratio=0.5,
    temporary_yaw=math.pi / 6,
    max_abs_yaw=0.60,
    max_drive_distance=0.20,
)


@pytest.mark.parametrize(
    "observation_fresh,clearance_accepted,robot_stopped",
    [
        (False, True, True),
        (True, False, True),
        (True, True, False),
        (False, False, False),
    ],
)
def test_missing_execution_evidence_fails_closed(
    observation_fresh,
    clearance_accepted,
    robot_stopped,
):
    assert plan_lateral_action_if_safe(
        observation_fresh,
        clearance_accepted,
        robot_stopped,
        **VALID_ARGS,
    ) is None


def test_all_execution_evidence_delegates_to_pure_planner():
    plan = plan_lateral_action_if_safe(True, True, True, **VALID_ARGS)
    assert plan == LateralActionPlan(
        signed_yaw=pytest.approx(math.pi / 6),
        drive_distance=pytest.approx(0.12),
    )


def test_planner_rejection_remains_fail_closed_after_gate_passes():
    assert plan_lateral_action_if_safe(
        True,
        True,
        True,
        **{**VALID_ARGS, "max_drive_distance": 0.10},
    ) is None
