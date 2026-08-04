import ast
from pathlib import Path

from nav2_apps import move_shelf_to_ship


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "nav2_apps"
    / "move_shelf_to_ship.py"
)


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_no_mission_flags_selects_integrated_course_route():
    args = move_shelf_to_ship._parser().parse_args([])

    operation_modes = (
        args.detection_only,
        args.approach_and_elevator,
        args.loaded_footprint_only,
        args.shipping_only,
        args.shipping_alignment_only,
        args.lower_only,
        args.exit_restore_only,
        args.exit_clearance_refine_only,
        args.shipping_relift_only,
        args.shipping_forward_refine_only,
        args.return_only,
    )

    assert not any(operation_modes)
    assert args.shipping_refine_distance == 0.16
    assert args.loaded_egress_initial_reverse == 0.20
    assert args.loaded_egress_turn_yaw == 0.12
    assert args.loaded_egress_final_reverse == 0.25
    assert args.loaded_egress_linear_speed == 0.05
    assert args.loaded_egress_angular_speed == 0.05
    assert args.exit_distance == 0.75
    assert args.clearance_refine_distance == 0.02
    assert args.clearance_x == 0.36


def test_integrated_route_contains_complete_fail_closed_sequence():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission = _function(tree, "_run_integrated_mission")
    mission_source = ast.get_source_segment(source, mission)

    expected_calls = [
        "_request_stepwise_attach",
        "_apply_loaded_footprint_verified",
        "_loaded_egress_before_shipping",
        "_navigate_to_shipping",
        "_bounded_forward_by_odom",
        "_publish_elevator_down_and_wait",
        "_exit_restore_integrated",
        "_navigate_to_init",
    ]
    positions = [mission_source.index(call) for call in expected_calls]

    assert positions == sorted(positions)
    assert "integrated_mode = not any(operation_modes)" in source
    assert "integrated simulation mission will initialize AMCL" not in source
    assert "_wait_for_existing_localization" in source


def test_loaded_egress_is_bounded_and_ordered_before_shipping():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    egress = _function(tree, "_loaded_egress_before_shipping")
    egress_source = ast.get_source_segment(source, egress)

    first_reverse = egress_source.index("_bounded_reverse_by_odom")
    turn = egress_source.index("_bounded_rotate_by_odom")
    second_reverse = egress_source.rindex("_bounded_reverse_by_odom")

    assert first_reverse < turn < second_reverse
    assert egress_source.count("_bounded_reverse_by_odom") == 2
    assert egress_source.count("_settle_without_motion") == 3
    assert '"loaded egress initial reverse"' in egress_source
    assert '"loaded egress final reverse"' in egress_source
    assert "LOADED_EGRESS_COMPLETE" in egress_source


def test_loaded_egress_turn_uses_signed_odom_accumulation_and_stops():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    turn = _function(tree, "_bounded_rotate_by_odom")
    turn_source = ast.get_source_segment(source, turn)

    assert "direction = math.copysign(1.0, yaw)" in turn_source
    assert "delta = _normalize_angle(current_yaw - last_yaw)" in turn_source
    assert "command.angular.z = direction * speed" in turn_source
    assert "finally:" in turn_source
    assert "publisher.publish(stop)" in turn_source


def test_integrated_exit_allows_only_one_clearance_refinement():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    exit_flow = _function(tree, "_exit_restore_integrated")
    exit_source = ast.get_source_segment(source, exit_flow)

    assert exit_source.count("_bounded_reverse_by_odom") == 2
    assert exit_source.count("_request_shelf_transform") == 2
    assert "if transform is None" in exit_source
    assert "_apply_unloaded_footprint_verified" in exit_source
    assert "EXIT_ACCEPTANCE_PENDING" in exit_source


def test_course_script_delegates_to_installed_integrated_main():
    script = (
        Path(__file__).parents[1] / "scripts" / "move_shelf_to_ship.py"
    ).read_text(encoding="utf-8")

    assert "from nav2_apps.move_shelf_to_ship import main" in script
    assert "main()" in script


def test_pathplanner_launch_owns_simulation_shelf_server():
    repository = Path(__file__).parents[2]
    launch_source = (
        repository
        / "path_planner_server"
        / "launch"
        / "pathplanner.launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="shelf_detection_server"' in launch_source
    assert 'executable="shelf_detection_server"' in launch_source
    assert "condition=IfCondition(use_sim_time)" in launch_source


def test_localization_launch_initializes_simulation_before_mission():
    repository = Path(__file__).parents[2]
    launch_source = (
        repository
        / "localization_server"
        / "launch"
        / "localization.launch.py"
    ).read_text(encoding="utf-8")

    assert '"auto_initial_pose"' in launch_source
    assert "default_value=use_sim_time" in launch_source
    assert 'executable="auto_initial_pose.py"' in launch_source
    assert "condition=IfCondition(auto_initial_pose_enabled)" in launch_source
