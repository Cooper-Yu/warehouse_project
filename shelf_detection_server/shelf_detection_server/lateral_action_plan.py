"""Pure one-round lateral action candidate planner."""

from dataclasses import dataclass, field
import math
from typing import Optional


SIN_EPSILON = 1.0e-6


@dataclass(frozen=True)
class LateralActionPlan:
    signed_yaw: float
    drive_distance: float
    error_magnitude: float = field(default=0.0, compare=False)


def plan_lateral_action(
    lateral_error: float,
    correction_ratio: float,
    temporary_yaw: float,
    max_abs_yaw: float,
    max_drive_distance: float,
    min_drive_distance: float = 0.0,
    target_tolerance: float = 0.0,
    min_temporary_yaw: float = 0.0,
    yaw_error_gain: float = 0.0,
) -> Optional[LateralActionPlan]:
    """Return one bounded geometry candidate or None when unsafe/invalid."""
    values = (
        lateral_error,
        correction_ratio,
        temporary_yaw,
        max_abs_yaw,
        max_drive_distance,
        min_drive_distance,
        target_tolerance,
        min_temporary_yaw,
        yaw_error_gain,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    if (
        lateral_error == 0.0
        or not 0.0 < correction_ratio <= 1.0
        or temporary_yaw == 0.0
        or max_abs_yaw <= 0.0
        or max_drive_distance <= 0.0
        or min_drive_distance < 0.0
        or target_tolerance < 0.0
        or min_temporary_yaw < 0.0
        or yaw_error_gain < 0.0
        or abs(lateral_error) <= target_tolerance
    ):
        return None

    target_lateral = abs(lateral_error) * correction_ratio
    effective_yaw = abs(temporary_yaw)
    if yaw_error_gain > 0.0:
        effective_yaw = min(
            effective_yaw,
            max(min_temporary_yaw, abs(lateral_error) * yaw_error_gain),
        )
    sin_yaw = math.sin(effective_yaw)
    if abs(sin_yaw) < SIN_EPSILON:
        return None

    drive_distance = max(target_lateral / abs(sin_yaw), min_drive_distance)
    signed_yaw = math.copysign(effective_yaw, lateral_error)
    if (
        abs(signed_yaw) > max_abs_yaw
        or drive_distance > max_drive_distance
    ):
        return None
    return LateralActionPlan(signed_yaw, drive_distance, abs(lateral_error))
