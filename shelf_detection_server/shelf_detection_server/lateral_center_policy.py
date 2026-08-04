"""Pure lateral-centering policy with no ROS dependencies."""

from enum import Enum
import math


class LateralDecision(Enum):
    REJECT = "reject"
    CENTERED = "centered"
    SHIFT_LEFT = "shift_left"
    SHIFT_RIGHT = "shift_right"


def classify_lateral_center(
    lateral_error: float,
    tolerance: float,
    *,
    fresh: bool,
    safe_zone: bool,
    heading_ok: bool,
) -> LateralDecision:
    """Classify the next geometry state without creating motion commands."""
    if (
        not fresh
        or not safe_zone
        or not heading_ok
        or not math.isfinite(lateral_error)
        or not math.isfinite(tolerance)
        or tolerance < 0.0
    ):
        return LateralDecision.REJECT

    if abs(lateral_error) <= tolerance:
        return LateralDecision.CENTERED
    if lateral_error > tolerance:
        return LateralDecision.SHIFT_LEFT
    return LateralDecision.SHIFT_RIGHT
