"""Pure result-gate logic for the first Checkpoint 12 vertical slice."""

from enum import IntEnum
from typing import Any


class ExitCode(IntEnum):
    """Process outcomes exposed to local and cloud evidence collection."""

    SUCCEEDED = 0
    GOAL_REJECTED = 2
    FAILED = 3
    CANCELED = 4
    UNKNOWN = 5


def classify_task_result(result: Any, task_result: Any) -> ExitCode:
    """Map Nav2 TaskResult values to a bounded process exit code."""
    if result == task_result.SUCCEEDED:
        return ExitCode.SUCCEEDED
    if result == task_result.FAILED:
        return ExitCode.FAILED
    if result == task_result.CANCELED:
        return ExitCode.CANCELED
    return ExitCode.UNKNOWN
