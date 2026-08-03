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
    function_source = ast.get_source_segment(source, shipping)
    assert "elevator_down" not in function_source
    assert "elevator-down" not in function_source


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


def test_shipping_alignment_only_has_no_navigation_goal():
    source_path = (
        Path(__file__).parents[1]
        / "nav2_apps"
        / "move_shelf_to_ship.py"
    )
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    alignment_only = _function(tree, "_align_at_shipping")
    function_source = ast.get_source_segment(source, alignment_only)
    calls = [
        node.func.attr
        for node in ast.walk(alignment_only)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]

    assert "--shipping-alignment-only" in source
    assert "_accept_or_align_shipping_pose" in function_source
    assert "goToPose" not in calls
    assert "elevator_down" not in function_source
    assert "AT_SHIPPING" in function_source


def test_shipping_alignment_only_accepts_without_navigation(monkeypatch):
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_accept_or_align_shipping_pose",
        lambda *_args, **_kwargs: True,
    )
    navigator = FakeNavigator(True, 1)

    result = move_shelf_to_ship._align_at_shipping(
        navigator,
        PoseStamped(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.40,
        20.0,
        1.0,
        5.0,
        0.5,
        3,
    )

    assert result == ExitCode.SUCCEEDED
    assert any(
        "no goToPose goal will be sent" in message
        for _level, message in navigator.logger.messages
    )
    assert any(
        "AT_SHIPPING" in message
        for _level, message in navigator.logger.messages
    )


def test_shipping_alignment_only_fails_closed(monkeypatch):
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_accept_or_align_shipping_pose",
        lambda *_args, **_kwargs: False,
    )
    navigator = FakeNavigator(True, 1)

    result = move_shelf_to_ship._align_at_shipping(
        navigator,
        PoseStamped(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.40,
        20.0,
        1.0,
        5.0,
        0.5,
        3,
    )

    assert result == ExitCode.UNKNOWN
    assert any(
        "SHIPPING_ALIGNMENT_PENDING" in message
        for _level, message in navigator.logger.messages
    )


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))

    def warning(self, message):
        self.messages.append(("warning", message))


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


class FakeAlignmentNavigator(FakeNavigator):
    def __init__(self, result=1):
        super().__init__(True, result)
        self.spin_requests = []

    def spin(self, spin_dist, time_allowance):
        self.spin_requests.append((spin_dist, time_allowance))
        return True


def _transform(x, y, yaw):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=__import__("math").sin(yaw / 2.0),
                w=__import__("math").cos(yaw / 2.0),
            ),
        )
    )


def _shipping_pose(x=1.0, y=2.0, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = __import__("math").sin(yaw / 2.0)
    pose.pose.orientation.w = __import__("math").cos(yaw / 2.0)
    return pose


def test_shipping_success_stops_at_at_shipping(monkeypatch):
    task_result = SimpleNamespace(
        SUCCEEDED=1,
        FAILED=2,
        CANCELED=3,
    )
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_accept_or_align_shipping_pose",
        lambda *_args, **_kwargs: True,
    )
    navigator = FakeNavigator(True, task_result.SUCCEEDED)

    result = move_shelf_to_ship._navigate_to_shipping(
        navigator,
        PoseStamped(),
        10.0,
        "robot_base_footprint",
        0.25,
        0.10,
        0.30,
        15.0,
        1.0,
        5.0,
    )

    assert result == ExitCode.SUCCEEDED
    assert not navigator.cancelled
    assert any(
        "AT_SHIPPING" in message
        for _level, message in navigator.logger.messages
    )


def test_shipping_success_requires_final_pose_acceptance(monkeypatch):
    task_result = SimpleNamespace(
        SUCCEEDED=1,
        FAILED=2,
        CANCELED=3,
    )
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_accept_or_align_shipping_pose",
        lambda *_args, **_kwargs: False,
    )
    navigator = FakeNavigator(True, task_result.SUCCEEDED)

    result = move_shelf_to_ship._navigate_to_shipping(
        navigator,
        PoseStamped(),
        10.0,
        "robot_base_footprint",
        0.25,
        0.10,
        0.30,
        15.0,
        1.0,
        5.0,
    )

    assert result == ExitCode.UNKNOWN
    assert any(
        "SHIPPING_ALIGNMENT_PENDING" in message
        for _level, message in navigator.logger.messages
    )
    assert not any(
        "shipping pose accepted" in message
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
        "robot_base_footprint",
        0.25,
        0.10,
        0.30,
        15.0,
        1.0,
        5.0,
    )

    assert result == ExitCode.CANCELED
    assert navigator.cancelled
    assert not any(
        "AT_SHIPPING" in message
        for _level, message in navigator.logger.messages
    )


def test_shipping_pose_error_is_wrap_safe():
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )

    position_error, yaw_error = move_shelf_to_ship._shipping_pose_error(
        transform,
        1.3,
        2.4,
        2.0 * 3.141592653589793 - 0.05,
    )

    assert abs(position_error - 0.5) < 1e-9
    assert abs(yaw_error + 0.05) < 1e-9


def test_shipping_accepts_pose_without_spin_inside_tolerance(monkeypatch):
    navigator = FakeAlignmentNavigator()
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_lookup_fresh_transform",
        lambda *_args: _transform(1.05, 2.0, 0.05),
    )

    accepted = move_shelf_to_ship._accept_or_align_shipping_pose(
        navigator,
        _shipping_pose(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.30,
        15.0,
        1.0,
        5.0,
    )

    assert accepted
    assert navigator.spin_requests == []


def test_shipping_rejects_unbounded_yaw_without_spin(monkeypatch):
    navigator = FakeAlignmentNavigator()
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_lookup_fresh_transform",
        lambda *_args: _transform(1.0, 2.0, -0.5),
    )

    accepted = move_shelf_to_ship._accept_or_align_shipping_pose(
        navigator,
        _shipping_pose(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.30,
        15.0,
        1.0,
        5.0,
    )

    assert not accepted
    assert navigator.spin_requests == []


def test_shipping_runs_partial_spin_and_rechecks_fresh_pose(monkeypatch):
    task_result = SimpleNamespace(SUCCEEDED=1, FAILED=2, CANCELED=3)
    navigator = FakeAlignmentNavigator(task_result.SUCCEEDED)
    observations = iter([
        _transform(1.0, 2.0, -0.2),
        _transform(1.0, 2.0, -0.02),
    ])
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_lookup_fresh_transform",
        lambda *_args: next(observations),
    )

    accepted = move_shelf_to_ship._accept_or_align_shipping_pose(
        navigator,
        _shipping_pose(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.30,
        15.0,
        1.0,
        5.0,
    )

    assert accepted
    assert len(navigator.spin_requests) == 1
    assert abs(navigator.spin_requests[0][0] - 0.1) < 1e-9


def test_shipping_runs_up_to_three_partial_spins(monkeypatch):
    task_result = SimpleNamespace(SUCCEEDED=1, FAILED=2, CANCELED=3)
    navigator = FakeAlignmentNavigator(task_result.SUCCEEDED)
    observations = iter([
        _transform(1.0, 2.0, -0.30),
        _transform(1.0, 2.0, -0.20),
        _transform(1.0, 2.0, -0.12),
        _transform(1.0, 2.0, -0.08),
    ])
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_lookup_fresh_transform",
        lambda *_args: next(observations),
    )

    accepted = move_shelf_to_ship._accept_or_align_shipping_pose(
        navigator,
        _shipping_pose(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.40,
        15.0,
        1.0,
        5.0,
    )

    assert accepted
    requested_angles = [
        request[0] for request in navigator.spin_requests
    ]
    assert len(requested_angles) == 3
    assert abs(requested_angles[0] - 0.15) < 1e-9
    assert abs(requested_angles[1] - 0.10) < 1e-9
    assert abs(requested_angles[2] - 0.06) < 1e-9


def test_shipping_rejects_after_three_partial_spins(monkeypatch):
    task_result = SimpleNamespace(SUCCEEDED=1, FAILED=2, CANCELED=3)
    navigator = FakeAlignmentNavigator(task_result.SUCCEEDED)
    observations = iter([
        _transform(1.0, 2.0, -0.30),
        _transform(1.0, 2.0, -0.25),
        _transform(1.0, 2.0, -0.20),
        _transform(1.0, 2.0, -0.15),
    ])
    monkeypatch.setattr(move_shelf_to_ship, "TaskResult", task_result)
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_lookup_fresh_transform",
        lambda *_args: next(observations),
    )

    accepted = move_shelf_to_ship._accept_or_align_shipping_pose(
        navigator,
        _shipping_pose(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.40,
        15.0,
        1.0,
        5.0,
    )

    assert not accepted
    assert len(navigator.spin_requests) == 3


def test_shipping_accepts_before_requesting_tiny_spin(monkeypatch):
    navigator = FakeAlignmentNavigator()
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_lookup_fresh_transform",
        lambda *_args: _transform(1.0, 2.0, -0.08),
    )

    accepted = move_shelf_to_ship._accept_or_align_shipping_pose(
        navigator,
        _shipping_pose(),
        "robot_base_footprint",
        0.25,
        0.10,
        0.40,
        15.0,
        1.0,
        5.0,
    )

    assert accepted
    assert navigator.spin_requests == []
