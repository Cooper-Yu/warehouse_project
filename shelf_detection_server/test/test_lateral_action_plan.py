import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.lateral_action_plan import (
    LateralActionPlan,
    plan_lateral_action,
)


def test_positive_error_plans_left_yaw_and_expected_distance():
    plan = plan_lateral_action(0.12, 0.5, math.pi / 6, 0.60, 0.20)
    assert plan == LateralActionPlan(
        signed_yaw=pytest.approx(math.pi / 6),
        drive_distance=pytest.approx(0.12),
    )


def test_negative_error_plans_right_yaw():
    plan = plan_lateral_action(-0.12, 0.5, math.pi / 6, 0.60, 0.20)
    assert plan is not None
    assert plan.signed_yaw == pytest.approx(-math.pi / 6)
    assert plan.drive_distance == pytest.approx(0.12)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"temporary_yaw": 0.70},
        {"max_drive_distance": 0.10},
        {"temporary_yaw": 1.0e-12},
        {"correction_ratio": 0.0},
        {"lateral_error": math.nan},
    ],
)
def test_invalid_or_over_limit_candidate_rejects(kwargs):
    inputs = {
        "lateral_error": 0.12,
        "correction_ratio": 0.5,
        "temporary_yaw": math.pi / 6,
        "max_abs_yaw": 0.60,
        "max_drive_distance": 0.20,
    }
    inputs.update(kwargs)
    assert plan_lateral_action(**inputs) is None
