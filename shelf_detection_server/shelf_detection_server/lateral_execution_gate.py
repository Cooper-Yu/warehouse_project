"""Pure execution-precondition gate with no motion output."""

from typing import Optional

from .lateral_action_plan import LateralActionPlan, plan_lateral_action


def plan_lateral_action_if_safe(
    observation_fresh: bool,
    clearance_accepted: bool,
    robot_stopped: bool,
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
    """Delegate only when all execution evidence is currently accepted."""
    if not (
        observation_fresh
        and clearance_accepted
        and robot_stopped
    ):
        return None
    return plan_lateral_action(
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
