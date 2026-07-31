from enum import IntEnum

from nav2_apps.result_gate import ExitCode, classify_task_result


class FakeTaskResult(IntEnum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


def test_success_advances_the_slice():
    result = classify_task_result(FakeTaskResult.SUCCEEDED, FakeTaskResult)
    assert result == ExitCode.SUCCEEDED


def test_non_success_results_stop_the_slice():
    failed = classify_task_result(FakeTaskResult.FAILED, FakeTaskResult)
    canceled = classify_task_result(FakeTaskResult.CANCELED, FakeTaskResult)
    unknown = classify_task_result(FakeTaskResult.UNKNOWN, FakeTaskResult)

    assert failed == ExitCode.FAILED
    assert canceled == ExitCode.CANCELED
    assert unknown == ExitCode.UNKNOWN
