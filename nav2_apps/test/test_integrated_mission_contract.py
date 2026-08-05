import ast
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped
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
    assert args.loaded_footprint == (
        "[[0.40, 0.40], [-0.40, 0.40], "
        "[-0.40, -0.40], [0.40, -0.40]]"
    )
    assert args.loaded_egress_initial_reverse == 0.25
    assert args.loaded_egress_first_turn_yaw == 0.12
    assert args.loaded_egress_final_reverse == 0.60
    assert args.loaded_egress_second_turn_yaw == 0.16
    assert args.loaded_egress_handoff_right_yaw == pytest.approx(math.pi / 2.0)
    assert args.loaded_egress_handoff_angular_speed == 0.08
    assert args.loaded_egress_handoff_turn_timeout == 45.0
    assert args.loaded_egress_linear_speed == 0.05
    assert args.loaded_egress_angular_speed == 0.05
    assert args.loaded_egress_arc_distance == 0.35
    assert args.loaded_egress_arc_yaw == 0.18
    assert args.loaded_egress_arc_angular_speed == 0.026
    assert args.loaded_egress_arc_distance_tolerance == 0.01
    assert args.loaded_handoff_costmap_timeout == 5.0
    assert args.loaded_prealign_max_segment_yaw == 0.15
    assert args.loaded_prealign_arc_max_distance == 0.12
    assert args.loaded_prealign_arc_linear_speed == 0.04
    assert args.loaded_prealign_max_total_yaw == 2.80
    assert args.loaded_prealign_bearing_tolerance == 0.20
    assert args.loaded_prealign_max_confirmable_position_jump == 0.23
    assert args.loaded_prealign_max_localization_confirmations == 1
    assert args.loaded_prealign_path_probe_timeout == 5.0
    assert args.loaded_prealign_path_end_tolerance == 0.55
    assert args.loaded_prealign_path_handoff_max_bearing == 0.60
    assert args.loaded_prealign_planner_id == "GridBased"
    assert args.loaded_localization_samples == 5
    assert args.loaded_localization_max_position_jump == 0.20
    assert args.loaded_localization_max_yaw_jump == 0.20
    assert args.loaded_shipping_max_linear_speed == 0.15
    assert args.loaded_shipping_max_angular_speed == 0.30
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
        "_wait_for_loaded_localization_stability",
        "_wait_for_loaded_handoff_clearance",
        "_prealign_loaded_shipping_bearing",
        "_controller_speed_snapshot",
        "_set_controller_speeds",
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
    assert "CONTROLLER_SPEEDS_RESTORED" in mission_source


def test_localization_step_rejects_translation_and_yaw_jumps():
    stable = (1.0, -2.0, 0.10)

    assert move_shelf_to_ship.localization_step_is_stable(
        stable, (1.05, -1.95, 0.15), 0.20, 0.20
    )
    assert not move_shelf_to_ship.localization_step_is_stable(
        stable, (1.30, -2.0, 0.10), 0.20, 0.20
    )
    assert not move_shelf_to_ship.localization_step_is_stable(
        stable, (1.0, -2.0, 0.40), 0.20, 0.20
    )


def test_loaded_localization_monitor_reads_direct_map_to_odom_transform():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    monitor = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef)
        and node.name == "_LoadedLocalizationMonitor"
    )
    monitor_source = ast.get_source_segment(source, monitor)

    assert '"map", self.odom_frame' in monitor_source
    assert '"map", self.base_frame' not in monitor_source
    assert "self.odom_frame, self.base_frame" not in monitor_source


def test_loaded_localization_preflight_enforces_monotonic_sample_spacing():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    gate = _function(tree, "_wait_for_loaded_localization_stability")
    gate_source = ast.get_source_segment(source, gate)

    assert "next_sample_at = time.monotonic()" in gate_source
    assert "if now < next_sample_at" in gate_source
    assert "next_sample_at - now" in gate_source
    assert "next_sample_at = time.monotonic() + sample_interval" in gate_source
    assert "monitor.last_position_jump" in gate_source
    assert "monitor.last_yaw_jump" in gate_source


def test_loaded_egress_is_one_bounded_reverse_s_curve():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    egress = _function(tree, "_loaded_egress_before_shipping")
    egress_source = ast.get_source_segment(source, egress)

    calls = [
        node.func.id
        for node in ast.walk(egress)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == "_bounded_reverse_arc_by_odom"
    ]

    assert calls == [
        "_bounded_reverse_arc_by_odom",
        "_bounded_reverse_arc_by_odom",
    ]
    assert egress_source.count("_bounded_reverse_arc_by_odom") == 2
    assert egress_source.count("_settle_without_motion") == 2
    assert "reverse-right 0.35/0.18" in egress_source
    assert "reverse-left 0.35/0.18" in egress_source
    assert "args.loaded_egress_arc_distance" in egress_source
    assert "args.loaded_egress_arc_yaw" in egress_source
    assert "-args.loaded_egress_arc_yaw" in egress_source
    assert egress_source.index("-args.loaded_egress_arc_yaw") < (
        egress_source.index("args.loaded_egress_arc_yaw", 1)
    )
    assert "LOADED_EGRESS_REVERSE_S_COMPLETE" in egress_source


def test_loaded_egress_namespace_contract_has_every_referenced_argument():
    args = move_shelf_to_ship._parser().parse_args([])
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    egress = _function(tree, "_loaded_egress_before_shipping")
    referenced = {
        node.attr
        for node in ast.walk(egress)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "args"
    }

    missing = sorted(name for name in referenced if not hasattr(args, name))

    assert missing == []


def test_loaded_reverse_arc_requires_both_odom_targets_and_stops():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    arc = _function(tree, "_bounded_reverse_arc_by_odom")
    arc_source = ast.get_source_segment(source, arc)

    assert "command.linear.x = -linear_speed" in arc_source
    assert "command.angular.z = yaw_direction * angular_speed" in arc_source
    assert "distance += math.hypot" in arc_source
    assert "yaw_traveled += yaw_direction * delta" in arc_source
    assert "if distance_done and yaw_done" in arc_source
    assert "wrong yaw direction" in arc_source
    assert "finally:" in arc_source
    assert "publisher.publish(stop)" in arc_source


def test_loaded_handoff_clearance_requires_no_lethal_or_outside_cells():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    gate = _function(tree, "_wait_for_loaded_handoff_clearance")
    gate_source = ast.get_source_segment(source, gate)

    assert '"/global_costmap/costmap_raw"' in gate_source
    assert '"/global_costmap/published_footprint"' in gate_source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in gate_source
    assert "analyze_costmap_start" in gate_source
    assert 'analysis["footprint_lethal"]' in gate_source
    assert 'analysis["footprint_outside"]' in gate_source
    assert "if lethal or outside" in gate_source
    assert "LOADED_HANDOFF_CLEARANCE_BLOCKED" in gate_source
    assert "LOADED_HANDOFF_CLEARANCE_READY" in gate_source


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


def test_loaded_prealign_arc_is_forward_right_dual_bounded_and_stops():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    arc = _function(tree, "_bounded_forward_right_arc_by_odom")
    arc_source = ast.get_source_segment(source, arc)

    assert "command.linear.x = linear_speed" in arc_source
    assert "command.angular.z = -angular_speed" in arc_source
    assert "math.hypot(current_x - start_x" in arc_source
    assert "turned_right += -delta" in arc_source
    assert "if distance_done or yaw_done" in arc_source
    assert "wrong yaw direction" in arc_source
    assert "finally:" in arc_source
    assert "publisher.publish(stop)" in arc_source


def test_loaded_shipping_handoff_requires_post_s_curve_path():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    prealign = _function(tree, "_prealign_loaded_shipping_bearing")
    prealign_source = ast.get_source_segment(source, prealign)

    assert "_bounded_loaded_shipping_path_probe" in prealign_source
    assert "probe_result is PathProbeResult.PATH_READY" in prealign_source
    assert "probe_result is PathProbeResult.NO_PATH" in prealign_source
    assert "LOADED_SHIPPING_DIRECT_NAV2_HANDOFF" in prealign_source
    assert "LOADED_SHIPPING_REVERSE_S_NO_PATH" in prealign_source
    assert "LOADED_SHIPPING_DIRECT_NAV2_UNCERTAIN" in prealign_source
    assert "_loaded_egress_before_shipping" not in prealign_source
    assert "_wait_for_loaded_localization_stability" not in prealign_source
    assert "_bounded_forward_right_arc_by_odom" not in prealign_source
    assert "_bounded_rotate_by_odom" not in prealign_source
    assert "_bounded_reverse_by_odom" not in prealign_source
    assert "create_publisher" not in prealign_source


def test_loaded_boundary_pair_diagnostic_stops_before_more_motion():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    prealign = _function(tree, "_legacy_prealign_loaded_shipping_bearing")
    prealign_source = ast.get_source_segment(source, prealign)
    helper = _function(tree, "_run_loaded_boundary_pair_diagnostic")
    helper_source = ast.get_source_segment(source, helper)

    trigger = prealign_source.index(
        "LOADED_BOUNDARY_PAIR_DIAGNOSTIC_TRIGGERED"
    )
    pair = prealign_source.index("_run_loaded_boundary_pair_diagnostic")
    next_motion = prealign_source.index(
        "_bounded_forward_right_arc_by_odom"
    )
    assert trigger < pair < next_motion
    diagnostic_tail = prealign_source[pair:next_motion]
    assert "return False" in diagnostic_tail
    assert "probe_result is PathProbeResult.NO_PATH" in prealign_source
    assert "args.loaded_boundary_pair_diagnostic" in prealign_source
    assert "ExplicitStartPairProbe" in helper_source
    assert "node.run()" in helper_source
    assert "node.destroy_node()" in helper_source
    assert "goToPose" not in helper_source
    assert "create_publisher" not in helper_source


def test_prealign_reconfirmation_accepts_only_narrow_translation_boundary():
    class Monitor:
        last_position_jump = 0.212
        last_yaw_jump = 0.006

    assert move_shelf_to_ship._prealign_localization_reconfirmation_allowed(
        Monitor(), 0.20, 0.20, 0.23
    )

    Monitor.last_position_jump = 0.315
    assert not (
        move_shelf_to_ship._prealign_localization_reconfirmation_allowed(
            Monitor(), 0.20, 0.20, 0.23
        )
    )

    Monitor.last_position_jump = 0.212
    Monitor.last_yaw_jump = 0.201
    assert not (
        move_shelf_to_ship._prealign_localization_reconfirmation_allowed(
            Monitor(), 0.20, 0.20, 0.23
        )
    )


def test_loaded_path_probe_is_bounded_and_never_starts_navigation():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    probe = _function(tree, "_bounded_loaded_shipping_path_probe")
    probe_source = ast.get_source_segment(source, probe)

    assert "ComputePathToPose.Goal()" in probe_source
    assert "request.use_start = False" in probe_source
    assert "request.planner_id = planner_id" in probe_source
    assert "time.monotonic() + timeout" in probe_source
    assert "goal_handle.cancel_goal_async()" in probe_source
    assert "GoalStatus.STATUS_SUCCEEDED" in probe_source
    assert "_path_reaches_goal" in probe_source
    assert "PathProbeResult.PATH_READY" in probe_source
    assert "PathProbeResult.NO_PATH" in probe_source
    assert "PathProbeResult.UNCERTAIN" in probe_source
    assert "LOADED_PATH_PROBE_READY" in probe_source
    assert "LOADED_PATH_PROBE_NO_PATH" in probe_source
    assert "LOADED_PATH_PROBE_UNCERTAIN" in probe_source
    assert "goToPose" not in probe_source
    assert "cmd_vel" not in probe_source


def test_path_probe_result_requires_map_frame_finite_endpoint_near_goal():
    from nav_msgs.msg import Path

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 2.0
    goal.pose.position.y = 1.0
    path = Path()
    path.header.frame_id = "map"
    path.poses = [PoseStamped(), PoseStamped()]
    path.poses[-1].pose.position.x = 2.1
    path.poses[-1].pose.position.y = 1.1

    assert move_shelf_to_ship._path_reaches_goal(
        path, "map", goal, 0.75
    )
    path.header.frame_id = "odom"
    assert not move_shelf_to_ship._path_reaches_goal(
        path, "map", goal, 0.75
    )


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
