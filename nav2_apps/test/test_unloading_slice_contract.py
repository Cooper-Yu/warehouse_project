import ast
import math
from pathlib import Path
from types import SimpleNamespace

from nav2_apps import move_shelf_to_ship


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "nav2_apps"
    / "move_shelf_to_ship.py"
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


class FakeNavigator:
    def __init__(self):
        self.publisher = FakePublisher()
        self.logger = FakeLogger()
        self.destroyed = False

    def create_publisher(self, *_args):
        return self.publisher

    def destroy_publisher(self, _publisher):
        self.destroyed = True

    def get_logger(self):
        return self.logger


def test_lower_only_publishes_bounded_down_commands(monkeypatch):
    navigator = FakeNavigator()
    times = iter([10.0, 10.0, 11.0])
    monkeypatch.setattr(
        move_shelf_to_ship.time,
        "monotonic",
        lambda: next(times),
    )
    monkeypatch.setattr(
        move_shelf_to_ship.rclpy,
        "ok",
        lambda: True,
    )
    monkeypatch.setattr(
        move_shelf_to_ship.rclpy,
        "spin_once",
        lambda *_args, **_kwargs: None,
    )

    result = move_shelf_to_ship._publish_elevator_down_and_wait(
        navigator,
        "/elevator_down",
        5,
        0.0,
        0.5,
    )

    assert result
    assert navigator.publisher.messages == ["down"] * 5
    assert navigator.destroyed
    assert any(
        "no programmatic completion feedback" in message
        for message in navigator.logger.messages
    )


def test_clearance_gate_requires_fresh_transform_and_minimum_x():
    low = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.359),
        )
    )
    accepted = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.360),
        )
    )

    assert not move_shelf_to_ship._clearance_passes(None, 0.36)
    assert not move_shelf_to_ship._clearance_passes(low, 0.36)
    assert move_shelf_to_ship._clearance_passes(accepted, 0.36)


def test_exit_progress_uses_starting_local_negative_x_axis():
    reverse, lateral = move_shelf_to_ship._exit_progress(
        2.0,
        3.0,
        math.pi / 2.0,
        2.0,
        2.25,
    )

    assert reverse == 0.75
    assert abs(lateral) < 1e-9


def test_exit_restore_defaults_match_reviewed_simulation_plan():
    args = move_shelf_to_ship._parser().parse_args(
        ["--exit-restore-only"]
    )

    assert args.exit_distance == 0.75
    assert args.exit_speed == 0.05
    assert args.exit_timeout == 25.0
    assert args.exit_heading_tolerance == 0.03
    assert args.clearance_x == 0.36


def test_unloading_modes_are_separate_and_stop_before_return():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    assert "--lower-only" in source
    assert "--exit-restore-only" in source
    assert "lower_acceptance_pending" in source
    assert "EXIT_ACCEPTANCE_PENDING" in source
    assert "UNLOADED_FOOTPRINT_VERIFIED" in source
    assert "confirm_shelf_lowered" in source
    assert "CLEAR_OF_SHELF" in source
    assert "return navigation" in source
    assert any(
        isinstance(node, ast.UnaryOp)
        and isinstance(node.op, ast.USub)
        for node in ast.walk(tree)
    )
