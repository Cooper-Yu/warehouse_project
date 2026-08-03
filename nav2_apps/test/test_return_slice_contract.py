from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

from nav2_apps import move_shelf_to_ship
from nav2_apps.pose_config import SIM_INIT_POSE
from nav2_apps.result_gate import ExitCode


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "nav2_apps"
    / "move_shelf_to_ship.py"
)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakeNavigator:
    def __init__(self, complete, result):
        self.complete = complete
        self.result = result
        self.cancelled = False
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger

    def goToPose(self, _pose):
        return True

    def isTaskComplete(self):
        return self.complete

    def getResult(self):
        return self.result

    def cancelTask(self):
        self.cancelled = True


def test_return_defaults_and_required_confirmations_are_explicit():
    args = move_shelf_to_ship._parser().parse_args(["--return-only"])
    source = SOURCE_PATH.read_text(encoding="utf-8")

    assert (args.return_x, args.return_y, args.return_yaw) == SIM_INIT_POSE
    assert args.return_timeout == 180.0
    assert "confirm_clear_of_shelf" in source
    assert "confirm_unloaded_footprint" in source
    assert "confirm_robot_stopped" in source
    assert "_apply_unloaded_footprint_verified" in source


def test_return_success_stops_at_at_init(monkeypatch):
    task_result = SimpleNamespace(SUCCEEDED=1, FAILED=2, CANCELED=3)
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    navigator = FakeNavigator(True, task_result.SUCCEEDED)

    result = move_shelf_to_ship._navigate_to_init(
        navigator,
        PoseStamped(),
        10.0,
    )

    assert result == ExitCode.SUCCEEDED
    assert not navigator.cancelled
    assert any(
        "AT_INIT" in message
        for _level, message in navigator.logger.messages
    )


def test_return_timeout_cancels_without_at_init(monkeypatch):
    times = iter([100.0, 102.0])
    monkeypatch.setattr(
        move_shelf_to_ship.time,
        "monotonic",
        lambda: next(times),
    )
    navigator = FakeNavigator(False, None)

    result = move_shelf_to_ship._navigate_to_init(
        navigator,
        PoseStamped(),
        1.0,
    )

    assert result == ExitCode.CANCELED
    assert navigator.cancelled
    assert not any(
        "AT_INIT" in message
        for _level, message in navigator.logger.messages
    )
