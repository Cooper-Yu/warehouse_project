import ast
from pathlib import Path
from types import SimpleNamespace

import yaml

from shelf_detection_server.server import ShelfDetectionServer


PACKAGE = Path(__file__).parents[1]
SERVER_PATH = PACKAGE / "shelf_detection_server" / "server.py"


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_real_entry_profile_is_bounded_and_excludes_final_push():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "real_entry.yaml").read_text()
    )["shelf_detection_server"]["ros__parameters"]

    assert profile["use_sim_time"] is False
    assert profile["staging_only"] is False
    assert profile["entry_only"] is True
    assert profile["final_push_only"] is False
    assert profile["final_push_enabled"] is False
    assert profile["elevator_up_enabled"] is False
    assert profile["intensity_threshold"] == 4000.0
    assert profile["odom_frame"] == "robot_odom"
    assert profile["yaw_tolerance"] == 0.04
    assert profile["alignment_required_consecutive_samples"] == 2
    assert profile["forward_speed"] <= 0.05
    assert profile["forward_step_distance"] <= 0.15
    assert profile["entry_odom_yaw_tolerance"] == 0.04
    assert profile["final_drive_distance"] == 0.0


def test_entry_only_wrapper_reuses_stepwise_core_with_hard_stop_boundary():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    entry = _function(tree, "_perform_entry_only")
    entry_source = ast.get_source_segment(source, entry)

    assert "_perform_stepwise_attach" in entry_source
    assert "stop_before_final_push=True" in entry_source
    assert "require_standoff_observation_only=True" in entry_source
    assert "_publish_stop" in entry_source
    assert "_publish_elevator_up" not in entry_source
    assert "_drive_forward_measured" not in entry_source


def test_stepwise_core_stops_before_final_push_and_elevator():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    attach = _function(tree, "_perform_stepwise_attach")
    attach_source = ast.get_source_segment(source, attach)

    boundary = attach_source.index("if stop_before_final_push:")
    final_drive = attach_source.index('get_parameter("final_drive_distance")')
    elevator = attach_source.index("_publish_elevator_up")
    assert boundary < final_drive < elevator


def test_entry_only_runtime_passes_only_safe_boundary_flags():
    server = object.__new__(ShelfDetectionServer)
    calls = []
    stops = []

    def stepwise(target, **kwargs):
        calls.append((target, kwargs))
        return True

    server._perform_stepwise_attach = stepwise
    server._publish_stop = lambda: stops.append(True)
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
    )

    target = ("robot_base_footprint", 1.0, 0.0, 0.0)
    assert server._perform_entry_only(target) is True
    assert calls == [(
        target,
        {
            "stop_before_final_push": True,
            "require_standoff_observation_only": True,
        },
    )]
    assert len(stops) == 1


def test_stepwise_entry_boundary_runtime_cannot_drive_final_push_or_lift(
    monkeypatch,
):
    import shelf_detection_server.server as server_module

    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)
    server = object.__new__(ShelfDetectionServer)
    target = ("robot_base_footprint", 0.15, 0.0, 0.0)
    parameters = {
        "movement_timeout": 45.0,
        "max_detected_yaw": 0.60,
        "center_distance_tolerance": 0.20,
        "center_lateral_tolerance": 0.08,
    }
    drives = []
    elevators = []
    stops = []

    server.get_parameter = lambda name: SimpleNamespace(
        value=parameters[name]
    )
    server._verify_safe_standoff_without_motion = (
        lambda initial_target, deadline: (initial_target, 0.0)
    )
    server._publish_cart_frame = lambda *args: None
    server._accepted_odom_heading_ok = lambda yaw, deadline: True
    server._drive_forward_measured = (
        lambda distance, deadline, speed=None: drives.append(distance) or True
    )
    server._publish_stop = lambda: stops.append(True)
    server._publish_elevator_up = lambda: elevators.append(True)
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
        warning=lambda message: None,
    )

    assert server._perform_stepwise_attach(
        target,
        stop_before_final_push=True,
        require_standoff_observation_only=True,
    ) is True
    assert drives == []
    assert elevators == []
    assert len(stops) == 1


def test_request_rejects_multiple_real_modes_together():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    handler = _function(tree, "_handle_request")
    handler_source = ast.get_source_segment(source, handler)

    assert "final_center_only" in handler_source
    assert '"complete=false: staging_only, entry_only, and "' in (
        handler_source
    )
    assert '"entry_refine_only, final_center_only, final_push_only, and "' in (
        handler_source
    )
    rejection = handler_source.index("if sum(")
    final_center = handler_source.index("if final_center_only:")
    final_push = handler_source.index("if final_push_only:")
    refine = handler_source.index("if entry_refine_only:")
    detection = handler_source.index("target = self._wait_for_cart_frame()")
    staging = handler_source.index("if staging_only:")
    entry = handler_source.index("if entry_only:")
    assert rejection < final_push < final_center < refine < detection < staging < entry


def test_final_push_profile_is_explicit_and_never_lifts():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "real_final_push.yaml").read_text()
    )["shelf_detection_server"]["ros__parameters"]

    assert profile["final_push_only"] is True
    assert profile["final_push_enabled"] is True
    assert profile["elevator_up_enabled"] is False
    assert profile["final_drive_distance"] == 0.3703


def test_final_push_only_runtime_stops_without_detection_or_lift():
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "final_drive_distance": 0.3703,
        "movement_timeout": 20.0,
        "final_push_enabled": True,
    }
    drives = []
    stops = []
    server.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    server._drive_forward_measured = (
        lambda distance, deadline: drives.append(distance) or True
    )
    server._publish_stop = lambda: stops.append(True)
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
        warning=lambda message: None,
    )

    assert server._perform_final_push_only() is True
    assert drives == [0.3703]
    assert len(stops) == 1


def test_real_entry_launch_loads_only_real_entry_profile():
    source = (
        PACKAGE / "launch" / "real_entry.launch.py"
    ).read_text(encoding="utf-8")

    assert '"real_entry.yaml"' in source
    assert 'executable="shelf_detection_server"' in source
    assert "path_planner_server" not in source
