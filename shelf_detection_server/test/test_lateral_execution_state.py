from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))

from shelf_detection_server.lateral_execution_state import (
    LateralExecutionEvidence,
    accept_clearance_if_ready,
    invalidate_on_action_start,
    record_fresh_observation,
    restore_stopped_after_settle,
)


@pytest.mark.parametrize(
    "before",
    [
        LateralExecutionEvidence(True, True, True),
        LateralExecutionEvidence(True, False, True),
        LateralExecutionEvidence(False, True, False),
        LateralExecutionEvidence(False, False, False),
    ],
)
def test_action_start_invalidates_all_pre_action_evidence(before):
    assert invalidate_on_action_start(before) == LateralExecutionEvidence(
        False, False, False
    )


def test_action_start_does_not_mutate_frozen_input():
    before = LateralExecutionEvidence(True, True, True)
    invalidate_on_action_start(before)
    assert before == LateralExecutionEvidence(True, True, True)


@pytest.mark.parametrize(
    "before",
    [
        LateralExecutionEvidence(False, False, False),
        LateralExecutionEvidence(True, True, False),
        LateralExecutionEvidence(True, False, True),
    ],
)
def test_stop_settle_restores_only_stopped_state(before):
    assert restore_stopped_after_settle(before) == LateralExecutionEvidence(
        False, False, True
    )


def test_stop_settle_does_not_mutate_frozen_input():
    before = LateralExecutionEvidence(False, False, False)
    restore_stopped_after_settle(before)
    assert before == LateralExecutionEvidence(False, False, False)


@pytest.mark.parametrize(
    "before",
    [
        LateralExecutionEvidence(False, False, True),
        LateralExecutionEvidence(True, True, True),
        LateralExecutionEvidence(False, True, False),
    ],
)
def test_fresh_observation_does_not_grant_clearance(before):
    assert record_fresh_observation(before) == LateralExecutionEvidence(
        True, False, before.robot_stopped
    )


def test_fresh_observation_does_not_mutate_frozen_input():
    before = LateralExecutionEvidence(False, False, True)
    record_fresh_observation(before)
    assert before == LateralExecutionEvidence(False, False, True)


@pytest.mark.parametrize(
    "before",
    [
        LateralExecutionEvidence(False, False, True),
        LateralExecutionEvidence(True, False, False),
        LateralExecutionEvidence(False, True, False),
    ],
)
def test_clearance_acceptance_fails_closed_without_fresh_stopped_context(before):
    assert accept_clearance_if_ready(before) == LateralExecutionEvidence(
        before.observation_fresh,
        False,
        before.robot_stopped,
    )


def test_clearance_acceptance_passes_for_fresh_stopped_context():
    before = LateralExecutionEvidence(True, False, True)
    assert accept_clearance_if_ready(before) == LateralExecutionEvidence(
        True, True, True
    )


def test_clearance_acceptance_does_not_mutate_frozen_input():
    before = LateralExecutionEvidence(True, False, True)
    accept_clearance_if_ready(before)
    assert before == LateralExecutionEvidence(True, False, True)
