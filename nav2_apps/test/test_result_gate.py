from enum import IntEnum
from types import SimpleNamespace

from nav2_apps.pose_config import optional_initial_pose
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


def test_initial_pose_can_be_omitted_for_existing_amcl_localization():
    args = SimpleNamespace(
        initial_x=None,
        initial_y=None,
        initial_yaw=None,
    )
    assert optional_initial_pose(args) is None


def test_partial_initial_pose_is_rejected():
    args = SimpleNamespace(
        initial_x=0.0,
        initial_y=None,
        initial_yaw=0.0,
    )
    try:
        optional_initial_pose(args)
    except ValueError:
        return
    raise AssertionError("partial initial pose should be rejected")
