"""Pure immutable evidence transitions for lateral execution."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LateralExecutionEvidence:
    observation_fresh: bool
    clearance_accepted: bool
    robot_stopped: bool


def invalidate_on_action_start(
    evidence: LateralExecutionEvidence,
) -> LateralExecutionEvidence:
    """Invalidate all pre-action evidence when physical execution begins."""
    return LateralExecutionEvidence(False, False, False)


def restore_stopped_after_settle(
    evidence: LateralExecutionEvidence,
) -> LateralExecutionEvidence:
    """Restore stopped-state only; fresh geometry is still unavailable."""
    return LateralExecutionEvidence(False, False, True)


def record_fresh_observation(
    evidence: LateralExecutionEvidence,
) -> LateralExecutionEvidence:
    """Record fresh geometry without granting clearance acceptance."""
    return LateralExecutionEvidence(
        observation_fresh=True,
        clearance_accepted=False,
        robot_stopped=evidence.robot_stopped,
    )


def accept_clearance_if_ready(
    evidence: LateralExecutionEvidence,
) -> LateralExecutionEvidence:
    """Grant clearance only for a fresh observation of a stopped robot."""
    return LateralExecutionEvidence(
        observation_fresh=evidence.observation_fresh,
        clearance_accepted=(
            evidence.observation_fresh and evidence.robot_stopped
        ),
        robot_stopped=evidence.robot_stopped,
    )
