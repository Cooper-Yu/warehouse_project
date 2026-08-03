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


def test_real_final_center_profile_derives_square_shelf_center_distance():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "real_final_center.yaml").read_text()
    )["shelf_detection_server"]["ros__parameters"]

    assert profile["use_sim_time"] is False
    assert profile["staging_only"] is False
    assert profile["entry_only"] is False
    assert profile["entry_refine_only"] is False
    assert profile["final_center_only"] is True
    assert profile["odom_frame"] == "robot_odom"
    assert profile["final_center_confirmed_front_offset"] == 0.20
    assert profile["final_center_confirmed_shelf_depth"] == 0.6562
    derived_distance = (
        profile["final_center_confirmed_front_offset"]
        + profile["final_center_confirmed_shelf_depth"] / 2.0
    )
    assert derived_distance == pytest.approx(0.5281)
    assert derived_distance <= profile["final_center_max_distance"] <= 0.55
    assert profile["final_center_speed"] <= 0.02
    assert profile["final_center_timeout"] >= 40.0
    assert profile["entry_odom_yaw_tolerance"] == 0.04
    assert profile["final_drive_distance"] == 0.0


def test_final_center_handler_bypasses_cart_detection():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    handler = _function(tree, "_handle_request")
    handler_source = ast.get_source_segment(source, handler)

    center = handler_source.index("if final_center_only:")
    detection = handler_source.index("target = self._wait_for_cart_frame()")
    assert center < detection
    assert "_perform_final_center_only" in handler_source[:detection]


def test_default_final_center_distance_rejects_before_motion():
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "final_center_confirmed_front_offset": 0.0,
        "final_center_confirmed_shelf_depth": 0.0,
        "final_center_max_distance": 0.55,
        "final_center_speed": 0.02,
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

    assert server._perform_final_center_only() is False
    assert drives == []
    assert len(stops) == 1


@pytest.mark.parametrize(
    ("front_offset", "shelf_depth"),
    [(0.0, 0.6562), (0.20, 0.0), (0.20, 0.72)],
)
def test_final_center_invalid_geometry_never_moves(
    front_offset, shelf_depth
):
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "final_center_confirmed_front_offset": front_offset,
        "final_center_confirmed_shelf_depth": shelf_depth,
        "final_center_max_distance": 0.55,
        "final_center_speed": 0.02,
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

    assert server._perform_final_center_only() is False
    assert drives == []


def test_square_geometry_is_one_derived_drive_without_elevator():
    server = object.__new__(ShelfDetectionServer)
    parameters = {
        "final_center_confirmed_front_offset": 0.20,
        "final_center_confirmed_shelf_depth": 0.6562,
        "final_center_max_distance": 0.55,
        "final_center_speed": 0.02,
        "final_center_timeout": 40.0,
    }
    drives = []
    stops = []
    elevators = []

    server.get_parameter = lambda name: SimpleNamespace(
        value=parameters[name]
    )
    server._wait_for_stable_odom_yaw = lambda deadline: 2.126
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

    assert server._perform_final_center_only() is True
    assert drives == [(pytest.approx(0.5281), 0.02)]
    assert len(stops) == 1
    assert elevators == []


def test_final_center_helper_has_no_detection_attach_or_elevator():
    source = SERVER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    center = _function(tree, "_perform_final_center_only")
    center_source = ast.get_source_segment(source, center)

    assert "_drive_forward_measured" in center_source
    assert "_accepted_odom_heading_ok" in center_source
    assert "_wait_for_cart_frame" not in center_source
    assert "_perform_stepwise_attach" not in center_source
    assert "_publish_elevator_up" not in center_source


def test_real_final_center_launch_loads_only_locked_profile():
    source = (
        PACKAGE / "launch" / "real_final_center.launch.py"
    ).read_text(encoding="utf-8")

    assert '"real_final_center.yaml"' in source
    assert 'executable="shelf_detection_server"' in source
    assert "path_planner_server" not in source
