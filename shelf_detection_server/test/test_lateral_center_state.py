from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.lateral_center_policy import LateralDecision
from shelf_detection_server.lateral_center_state import (
    LateralState,
    next_state_from_decision,
)


@pytest.mark.parametrize(
    ("decision", "expected"),
    [
        (LateralDecision.SHIFT_LEFT, LateralState.LATERAL_CORRECTION_LEFT),
        (LateralDecision.SHIFT_RIGHT, LateralState.LATERAL_CORRECTION_RIGHT),
        (LateralDecision.CENTERED, LateralState.JOINT_ACCEPTANCE),
        (
            LateralDecision.REJECT,
            LateralState.CLEARANCE_ACCEPTANCE_PENDING,
        ),
    ],
)
def test_decision_to_state_mapping(decision, expected):
    assert next_state_from_decision(decision) is expected


def test_unknown_value_fails_closed():
    with pytest.raises((TypeError, ValueError)):
        next_state_from_decision("shift_left")
