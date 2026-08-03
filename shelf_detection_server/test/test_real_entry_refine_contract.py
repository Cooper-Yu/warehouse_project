import ast
from pathlib import Path
from types import SimpleNamespace

import pytest
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


def test_real_entry_refine_profile_matches_verified_partial_entry():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "real_entry_refine.yaml").read_text()
    )["shelf_detection_server"]["ros__parameters"]

    assert profile["use_sim_time"] is False
    assert profile["staging_only"] is False
    assert profile["entry_only"] is False
    assert profile["entry_refine_only"] is True
    assert profile["odom_frame"] == "robot_odom"
    assert profile["entry_refine_required_completed_distance"] == 0.303
    assert profile["entry_refine_confirmed_completed_distance"] == 0.303
    assert profile["entry_refine_confirmation_tolerance"] <= 0.01
    assert profile["entry_refine_distance"] == 0.47
    assert profile["entry_refine_max_distance"] <= 0.50
    assert profile["entry_refine_speed"] <= 0.03
    assert profile["entry_odom_yaw_tolerance"] == 0.04
    assert profile["final_drive_distance"] == 0.0


def test_entry_refine_handler_bypasses_unavailable_cart_detection():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    handler = _function(tree, "_handle_request")
    handler_source = ast.get_source_segment(source, handler)

    refine = handler_source.index("if entry_refine_only:")
    detection = handler_source.index("target = self._wait_for_cart_frame()")
    assert refine < detection
    assert "_perform_entry_refine_only" in handler_source[:detection]
    assert "attach_to_shelf=true confirmation" in handler_source


def test_entry_refine_runtime_is_one_bounded_drive_without_elevator():
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "entry_refine_required_completed_distance": 0.303,
        "entry_refine_confirmed_completed_distance": 0.303,
        "entry_refine_confirmation_tolerance": 0.01,
        "entry_refine_distance": 0.47,
        "entry_refine_max_distance": 0.50,
        "entry_refine_speed": 0.03,
        "entry_refine_timeout": 25.0,
    }
    drives = []
    stops = []
    elevators = []

    server.get_parameter = lambda name: SimpleNamespace(
        value=parameters[name]
    )
    server._wait_for_stable_odom_yaw = lambda deadline: 2.115
    server._drive_forward_measured = (
        lambda distance, deadline, speed=None: drives.append(
            (distance, speed)
        ) or True
    )
    server._accepted_odom_heading_ok = lambda yaw, deadline: True
    server._publish_stop = lambda: stops.append(True)
    server._publish_elevator_up = lambda: elevators.append(True)
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
        warning=lambda message: None,
    )

    assert server._perform_entry_refine_only() is True
    assert drives == [(0.47, 0.03)]
    assert len(stops) == 1
    assert elevators == []


def test_entry_refine_rejects_wrong_completed_distance_before_motion():
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "entry_refine_required_completed_distance": 0.303,
        "entry_refine_confirmed_completed_distance": 0.0,
        "entry_refine_confirmation_tolerance": 0.01,
    }
    drives = []
    stops = []

    server.get_parameter = lambda name: SimpleNamespace(
        value=parameters[name]
    )
    server._drive_forward_measured = lambda *args: drives.append(args)
    server._publish_stop = lambda: stops.append(True)
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
        warning=lambda message: None,
    )

    assert server._perform_entry_refine_only() is False
    assert drives == []
    assert len(stops) == 1


@pytest.mark.parametrize("distance", [0.0, 0.51])
def test_entry_refine_rejects_invalid_distance_before_motion(distance):
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "entry_refine_required_completed_distance": 0.303,
        "entry_refine_confirmed_completed_distance": 0.303,
        "entry_refine_confirmation_tolerance": 0.01,
        "entry_refine_distance": distance,
        "entry_refine_max_distance": 0.50,
        "entry_refine_speed": 0.03,
    }
    drives = []
    stops = []

    server.get_parameter = lambda name: SimpleNamespace(
        value=parameters[name]
    )
    server._drive_forward_measured = lambda *args: drives.append(args)
    server._publish_stop = lambda: stops.append(True)
    server.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        error=lambda message: None,
        warning=lambda message: None,
    )

    assert server._perform_entry_refine_only() is False
    assert drives == []
    assert len(stops) == 1


def test_refine_helper_contains_no_detection_final_push_or_elevator():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    refine = _function(tree, "_perform_entry_refine_only")
    refine_source = ast.get_source_segment(source, refine)

    assert "_drive_forward_measured" in refine_source
    assert "_accepted_odom_heading_ok" in refine_source
    assert "_wait_for_cart_frame" not in refine_source
    assert "_perform_stepwise_attach" not in refine_source
    assert "final_drive_distance" not in refine_source
    assert "_publish_elevator_up" not in refine_source


def test_real_entry_refine_launch_loads_only_refine_profile():
    source = (
        PACKAGE / "launch" / "real_entry_refine.launch.py"
    ).read_text(encoding="utf-8")

    assert '"real_entry_refine.yaml"' in source
    assert 'executable="shelf_detection_server"' in source
    assert "path_planner_server" not in source
