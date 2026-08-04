import math
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.lateral_center_policy import (
    LateralDecision,
    classify_lateral_center,
)


def decide(error, tolerance=0.03, **overrides):
    inputs = {"fresh": True, "safe_zone": True, "heading_ok": True}
    inputs.update(overrides)
    return classify_lateral_center(error, tolerance, **inputs)


def test_centered_inside_and_on_tolerance():
    assert decide(0.0) is LateralDecision.CENTERED
    assert decide(0.03) is LateralDecision.CENTERED
    assert decide(-0.03) is LateralDecision.CENTERED


def test_positive_error_requires_left_shift():
    assert decide(0.12) is LateralDecision.SHIFT_LEFT


def test_negative_error_requires_right_shift():
    assert decide(-0.12) is LateralDecision.SHIFT_RIGHT


def test_fail_closed_preconditions():
    assert decide(0.12, fresh=False) is LateralDecision.REJECT
    assert decide(0.12, safe_zone=False) is LateralDecision.REJECT
    assert decide(0.12, heading_ok=False) is LateralDecision.REJECT
    assert decide(math.nan) is LateralDecision.REJECT
    assert decide(0.12, tolerance=-0.01) is LateralDecision.REJECT
