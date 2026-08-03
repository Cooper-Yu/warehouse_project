import ast
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

from nav2_apps import move_shelf_to_ship
from nav2_apps.result_gate import ExitCode


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_shipping_slice_stops_at_shipping_without_unload_actions():
    source_path = (
        Path(__file__).parents[1]
        / "nav2_apps"
        / "move_shelf_to_ship.py"
    )
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    shipping = _function(tree, "_navigate_to_shipping")

    calls = [
        node.func.attr
        for node in ast.walk(shipping)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]
    assert "goToPose" in calls
    assert "isTaskComplete" in calls
    assert "getResult" in calls
    assert "cancelTask" in calls
    assert "AT_SHIPPING" in source
    assert "--shipping-timeout" in source
    assert "elevator_down" not in source
    assert "elevator-down" not in source


def test_shipping_mode_requires_confirmations_and_loaded_footprint():
    source_path = (
        Path(__file__).parents[1]
        / "nav2_apps"
        / "move_shelf_to_ship.py"
    )
    source = source_path.read_text(encoding="utf-8")

    assert "--shipping-only" in source
    assert "confirm_lift_accepted" in source
    assert "confirm_robot_stopped" in source
    assert "_apply_loaded_footprint_verified" in source
    assert "initial pose override is not allowed" in source


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


def test_shipping_success_stops_at_at_shipping(monkeypatch):
    task_result = SimpleNamespace(
        SUCCEEDED=1,
        FAILED=2,
        CANCELED=3,
    )
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    navigator = FakeNavigator(True, task_result.SUCCEEDED)

    result = move_shelf_to_ship._navigate_to_shipping(
        navigator,
        PoseStamped(),
        10.0,
    )

    assert result == ExitCode.SUCCEEDED
    assert not navigator.cancelled
    assert any(
        "AT_SHIPPING" in message
        for _level, message in navigator.logger.messages
    )


def test_shipping_timeout_cancels_without_advancing(monkeypatch):
    times = iter([100.0, 102.0])
    monkeypatch.setattr(
        move_shelf_to_ship.time,
        "monotonic",
        lambda: next(times),
    )
    navigator = FakeNavigator(False, None)

    result = move_shelf_to_ship._navigate_to_shipping(
        navigator,
        PoseStamped(),
        1.0,
    )

    assert result == ExitCode.CANCELED
    assert navigator.cancelled
    assert not any(
        "AT_SHIPPING" in message
        for _level, message in navigator.logger.messages
    )
