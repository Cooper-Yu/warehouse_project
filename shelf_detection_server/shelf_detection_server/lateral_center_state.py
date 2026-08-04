"""Pure lateral decision-to-state mapping with no ROS dependencies."""

from enum import Enum

from .lateral_center_policy import LateralDecision


class LateralState(Enum):
    LATERAL_CORRECTION_LEFT = "lateral_correction_left"
    LATERAL_CORRECTION_RIGHT = "lateral_correction_right"
    JOINT_ACCEPTANCE = "joint_acceptance"
    CLEARANCE_ACCEPTANCE_PENDING = "clearance_acceptance_pending"


def next_state_from_decision(decision: LateralDecision) -> LateralState:
    """Map one geometry decision to the next explicit state."""
    match decision:
        case LateralDecision.SHIFT_LEFT:
            return LateralState.LATERAL_CORRECTION_LEFT
        case LateralDecision.SHIFT_RIGHT:
            return LateralState.LATERAL_CORRECTION_RIGHT
        case LateralDecision.CENTERED:
            return LateralState.JOINT_ACCEPTANCE
        case LateralDecision.REJECT:
            return LateralState.CLEARANCE_ACCEPTANCE_PENDING
        case _:
            raise ValueError(f"Unknown decision: {decision}")
