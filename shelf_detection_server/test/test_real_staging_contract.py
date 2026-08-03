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


def test_real_profile_is_low_speed_and_hard_gated():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "real_staging.yaml").read_text()
    )["shelf_detection_server"]["ros__parameters"]

    assert profile["use_sim_time"] is False
    assert profile["staging_only"] is True
    assert profile["intensity_threshold"] == 4000.0
    assert profile["odom_frame"] == "robot_odom"
    assert profile["target_base_frame"] == "robot_base_footprint"
    assert profile["forward_speed"] <= 0.05
    assert profile["rotate_speed"] <= 0.10
    assert profile["alignment_max_drive_distance"] <= 0.40
    assert profile["alignment_max_travel_yaw"] <= 0.70
    assert profile["final_drive_distance"] == 0.0


def test_staging_only_path_cannot_enter_or_publish_elevator():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    staging = _function(tree, "_perform_staging_only")
    staging_source = ast.get_source_segment(source, staging)

    assert "_align_at_safe_standoff" in staging_source
    assert "_publish_stop" in staging_source
    assert "_publish_cart_frame" in staging_source
    assert "_perform_stepwise_attach" not in staging_source
    assert "_drive_forward_measured" not in staging_source
    assert "_publish_elevator_up" not in staging_source


def test_request_routes_staging_only_before_full_attach():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    handler = _function(tree, "_handle_request")
    handler_source = ast.get_source_segment(source, handler)

    staging_position = handler_source.index("_perform_staging_only")
    full_attach_position = handler_source.index("_perform_stepwise_attach")
    assert staging_position < full_attach_position
    assert 'get_parameter("staging_only")' in handler_source


def test_staging_only_runtime_stops_without_full_attach_or_elevator():
    server = object.__new__(ShelfDetectionServer)
    stops = []
    published_frames = []

    server.get_parameter = lambda name: SimpleNamespace(value=45.0)
    server._align_at_safe_standoff = lambda target, deadline: (target, 0.0)
    server._publish_stop = lambda: stops.append(True)
    server._publish_cart_frame = lambda *args: published_frames.append(args)
    server._perform_stepwise_attach = lambda target: (_ for _ in ()).throw(
        AssertionError("full attach must remain unreachable")
    )
    server._publish_elevator_up = lambda: (_ for _ in ()).throw(
        AssertionError("elevator must remain unreachable")
    )
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
    )

    target = ("robot_base_footprint", 1.0, 0.0, 0.0)
    assert server._perform_staging_only(target) is True
    assert len(stops) == 1
    assert published_frames == [target[:3]]


def test_real_launch_loads_only_real_staging_profile():
    source = (
        PACKAGE / "launch" / "real_staging.launch.py"
    ).read_text(encoding="utf-8")

    assert '"real_staging.yaml"' in source
    assert 'executable="shelf_detection_server"' in source
    assert "path_planner_server" not in source
