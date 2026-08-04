"""Pure one-round lateral action candidate planner."""

from dataclasses import dataclass
import math
from typing import Optional


SIN_EPSILON = 1.0e-6


@dataclass(frozen=True)
class LateralActionPlan:
    signed_yaw: float
    drive_distance: float


def plan_lateral_action(
    lateral_error: float,
    correction_ratio: float,
    temporary_yaw: float,
    max_abs_yaw: float,
    max_drive_distance: float,
) -> Optional[LateralActionPlan]:
    """Return one bounded geometry candidate or None when unsafe/invalid."""
    values = (
        lateral_error,
        correction_ratio,
        temporary_yaw,
        max_abs_yaw,
        max_drive_distance,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    if (
        lateral_error == 0.0
        or not 0.0 < correction_ratio <= 1.0
        or temporary_yaw == 0.0
        or max_abs_yaw <= 0.0
        or max_drive_distance <= 0.0
    ):
        return None

    target_lateral = abs(lateral_error) * correction_ratio
    sin_yaw = math.sin(temporary_yaw)
    if abs(sin_yaw) < SIN_EPSILON:
        return None

    drive_distance = target_lateral / abs(sin_yaw)
    signed_yaw = math.copysign(abs(temporary_yaw), lateral_error)
    if (
        abs(signed_yaw) > max_abs_yaw
        or drive_distance > max_drive_distance
    ):
        return None
    return LateralActionPlan(signed_yaw, drive_distance)
