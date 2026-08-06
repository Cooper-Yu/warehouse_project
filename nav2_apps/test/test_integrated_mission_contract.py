import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, Transform
from nav2_msgs.msg import Costmap
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
    assert not args.stop_at_shipping
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
    assert args.loaded_egress_turn_step == 0.10
    assert args.loaded_egress_reverse_step == 0.05
    assert args.loaded_egress_max_total_yaw == 0.90
    assert args.loaded_egress_max_total_reverse == 1.00
    assert args.loaded_egress_max_reverse_per_round == 0.25
    assert args.loaded_egress_max_rounds == 12
    assert args.loaded_egress_no_improvement_limit == 2
    assert not args.loaded_egress_extreme_left_90_experiment
    assert args.loaded_egress_extreme_round1_turn == 0.10
    assert args.loaded_egress_extreme_round1_reverse == 0.50
    assert args.loaded_egress_extreme_round2_turn == 0.20
    assert args.loaded_egress_extreme_round2_reverse == 0.70
    assert args.loaded_egress_arc_distance == 0.35
    assert args.loaded_egress_arc_yaw == 0.18
    assert args.loaded_egress_arc_angular_speed == 0.026
    assert args.loaded_egress_arc_distance_tolerance == 0.01
    assert args.loaded_handoff_costmap_timeout == 5.0
    assert args.loaded_handoff_sweep_step == 0.05
    assert args.loaded_handoff_path_lookahead == 0.30
    assert args.loaded_handoff_max_nav_yaw == 0.15
    assert args.loaded_handoff_max_turn_segment == 0.10
    assert args.loaded_handoff_max_total_turn == 2.80
    assert args.loaded_handoff_max_turn_rounds == 8
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
    assert args.loaded_localization_samples == 21
    assert args.loaded_localization_sample_interval == 0.20
    assert args.loaded_localization_max_position_jump == 0.10
    assert args.loaded_localization_max_yaw_jump == 0.10
    assert args.loaded_localization_recovery_samples == 11
    assert args.loaded_localization_recovery_timeout == 5.0
    assert not args.loaded_localization_odom_recovery
    assert args.loaded_localization_recovery_distance == 0.15
    assert args.loaded_localization_recovery_speed == 0.03
    assert args.loaded_map_odom_freeze_lifecycle_timeout == 5.0
    assert args.loaded_shipping_max_linear_speed == 0.15
    assert args.loaded_shipping_max_angular_speed == 0.30
    assert args.exit_distance == 0.75
    assert args.clearance_refine_distance == 0.02
    assert args.clearance_x == 0.36


def test_integrated_stop_at_shipping_is_bounded_before_unload_actions():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    integrated = _function(tree, "_run_integrated_mission")
    function_source = ast.get_source_segment(source, integrated)

    stop_index = function_source.index("if args.stop_at_shipping")
    refine_index = function_source.index("_bounded_forward_by_odom")
    lower_index = function_source.index("_publish_elevator_down_and_wait")
    exit_index = function_source.index("_exit_restore_integrated")

    assert "--stop-at-shipping" in source
    assert "INTEGRATED_STOP_AT_SHIPPING" in function_source
    assert stop_index < refine_index < lower_index < exit_index
    assert "return ExitCode.SUCCEEDED" in function_source[stop_index:refine_index]


def test_integrated_route_contains_complete_fail_closed_sequence():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission = _function(tree, "_run_integrated_mission")
    mission_source = ast.get_source_segment(source, mission)

    expected_calls = [
        "_request_stepwise_attach",
        "_apply_loaded_footprint_verified",
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
    assert "DIRECT_NAV2_HANDOFF_AFTER_LIFT" in mission_source


def test_default_integrated_route_uses_loaded_speed_limit_and_localization_gate():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission_source = ast.get_source_segment(
        source, _function(tree, "_run_integrated_mission")
    )
    default_branch = mission_source.split(
        "if not args.loaded_egress_extreme_left_90_experiment:", 1
    )[1].split("else:", 1)[0]

    assert "_navigate_to_shipping" in default_branch
    assert "_loaded_egress_before_shipping" not in default_branch
    assert "_LoadedLocalizationMonitor" in default_branch
    assert "_wait_for_loaded_localization_stability" in default_branch
    assert "localization_monitor" in default_branch
    assert "_hold_zero_velocity" in default_branch
    assert "_wait_for_loaded_handoff_clearance" not in default_branch
    assert "_prealign_loaded_shipping_bearing" not in default_branch
    assert "_bounded_loaded_prehandoff_rotation" not in default_branch
    assert "_controller_speed_snapshot" in default_branch
    assert default_branch.count("_set_controller_speeds") == 2
    assert default_branch.count("_controller_speeds_match") == 2
    assert '"FollowPath.max_vel_x"' in default_branch
    assert '"FollowPath.max_speed_xy"' in default_branch
    assert '"FollowPath.max_vel_theta"' in default_branch
    assert "LOADED_CONTROLLER_SPEED_LIMIT_APPLIED" in default_branch
    assert "LOADED_CONTROLLER_SPEED_RESTORED" in default_branch
    assert "finally:" in default_branch
    begin_index = default_branch.index(
        "localization_monitor.begin_motion_monitoring()"
    )
    snapshot_index = default_branch.index("_controller_speed_snapshot")
    apply_index = default_branch.index("_set_controller_speeds")
    navigation_index = default_branch.index("_navigate_to_shipping")
    restore_index = default_branch.rindex("_set_controller_speeds")
    assert begin_index < snapshot_index < apply_index < navigation_index
    assert navigation_index < restore_index


def test_controller_speed_match_requires_all_values_within_tolerance():
    expected = {
        "FollowPath.max_vel_x": 0.15,
        "FollowPath.max_speed_xy": 0.15,
        "FollowPath.max_vel_theta": 0.30,
    }

    assert move_shelf_to_ship._controller_speeds_match(
        dict(expected), expected
    )
    assert not move_shelf_to_ship._controller_speeds_match(None, expected)
    assert not move_shelf_to_ship._controller_speeds_match(
        {"FollowPath.max_vel_x": 0.15}, expected
    )
    changed = dict(expected)
    changed["FollowPath.max_vel_theta"] = 0.31
    assert not move_shelf_to_ship._controller_speeds_match(changed, expected)


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


def test_loaded_localization_monitor_disables_baseline_gate_during_motion():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    monitor = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef)
        and node.name == "_LoadedLocalizationMonitor"
    )
    monitor_source = ast.get_source_segment(source, monitor)

    assert "self.baseline" in monitor_source
    assert "self.last_position_drift" in monitor_source
    assert "self.last_yaw_drift" in monitor_source
    assert "self.enforce_baseline_limits" in monitor_source
    assert "def begin_motion_monitoring" in monitor_source
    assert "self.enforce_baseline_limits = False" in monitor_source


def test_shipping_jump_cancel_holds_zero_velocity():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    shipping_source = ast.get_source_segment(
        source, _function(tree, "_navigate_to_shipping")
    )

    assert "navigator.cancelTask()" in shipping_source
    assert "_hold_zero_velocity(navigator, cmd_vel_topic)" in shipping_source
    assert "baseline_translation" in shipping_source


def test_odom_recovery_is_explicit_and_baseline_frozen():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission_source = ast.get_source_segment(
        source, _function(tree, "_navigate_to_shipping")
    )
    recovery_source = ast.get_source_segment(
        source, _function(tree, "_run_odom_localization_recovery")
    )
    freeze_source = ast.get_source_segment(
        source, _function(tree, "_freeze_map_to_odom")
    )

    assert "loaded_localization_odom_recovery" in source
    assert "_run_odom_localization_recovery" not in mission_source
    assert "baseline_transform" in recovery_source
    assert "captured_transform=monitor.baseline_transform" in recovery_source
    assert "_bounded_reverse_by_odom" in recovery_source
    assert "_restore_amcl_after_freeze" in recovery_source
    assert "captured_transform=None" in freeze_source


def test_localization_recovery_is_stationary_not_odom_motion():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    shipping_source = ast.get_source_segment(
        source, _function(tree, "_navigate_to_shipping")
    )
    assert "_hold_zero_velocity(navigator, cmd_vel_topic)" in shipping_source
    assert "LOADED_LOCALIZATION_STOP_RECOVERY_REPLAN" in shipping_source
    assert "_bounded_reverse_by_odom" not in shipping_source


def test_default_handoff_switches_to_step_only_motion_monitoring():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission_source = ast.get_source_segment(
        source, _function(tree, "_run_integrated_mission")
    )
    assert "localization_monitor.begin_motion_monitoring()" in mission_source
    assert "LOADED_LOCALIZATION_STOPPED_STABLE" in source


def test_restore_amcl_accepts_already_active_lifecycle_state():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    restore_source = ast.get_source_segment(
        source, _function(tree, "_restore_amcl_after_freeze")
    )

    assert "state = _amcl_state" in restore_source
    assert "state == State.PRIMARY_STATE_ACTIVE" in restore_source
    assert "state == State.PRIMARY_STATE_INACTIVE" in restore_source


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


def test_extreme_experiment_disables_only_localization_jump_enforcement():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission = _function(tree, "_run_integrated_mission")
    mission_source = ast.get_source_segment(source, mission)
    monitor = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef)
        and node.name == "_LoadedLocalizationMonitor"
    )
    monitor_source = ast.get_source_segment(source, monitor)

    assert "enforce_jump_limits: bool = True" in monitor_source
    assert "if not stable and not self.enforce_jump_limits" in monitor_source
    assert (
        "LOADED_LOCALIZATION_JUMP_OBSERVED_NOT_ENFORCED"
        in monitor_source
    )
    assert "DIRECT_NAV2_HANDOFF_AFTER_LIFT" in mission_source


def test_extreme_experiment_freezes_map_odom_and_restores_amcl():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    mission_source = ast.get_source_segment(
        source, _function(tree, "_run_integrated_mission")
    )
    freeze_source = ast.get_source_segment(
        source, _function(tree, "_freeze_map_to_odom")
    )
    restore_source = ast.get_source_segment(
        source, _function(tree, "_restore_amcl_after_freeze")
    )

    assert (
        "if not args.loaded_egress_extreme_left_90_experiment"
        in mission_source
    )
    assert "EXTREME_LOADED_EGRESS_DIAGNOSTIC_SELECTED" in mission_source
    assert "_freeze_map_to_odom" in mission_source
    assert "try:" in mission_source
    assert "finally:" in mission_source
    assert "_restore_amcl_after_freeze" in mission_source
    assert "Transition.TRANSITION_DEACTIVATE" in freeze_source
    assert "State.PRIMARY_STATE_INACTIVE" in freeze_source
    assert "Transition.TRANSITION_ACTIVATE" in restore_source
    assert "State.PRIMARY_STATE_ACTIVE" in restore_source


def test_frozen_map_odom_republishes_captured_transform(monkeypatch):
    sent = []
    timers = []

    class Broadcaster:
        def __init__(self, _node):
            pass

        def sendTransform(self, message):
            sent.append(message)

    monkeypatch.setattr(
        move_shelf_to_ship, "TransformBroadcaster", Broadcaster
    )
    navigator = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
        create_timer=lambda period, callback: (
            timers.append((period, callback))
            or SimpleNamespace(cancel=lambda: None)
        ),
        destroy_timer=lambda _timer: None,
    )
    transform = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        child_frame_id="odom",
        transform=Transform(),
    )

    frozen = move_shelf_to_ship._FrozenMapOdom(navigator, transform)
    assert timers[0][0] == pytest.approx(0.05)
    assert sent[0].header.frame_id == "map"
    assert sent[0].child_frame_id == "odom"
    assert sent[0].transform is transform.transform
    frozen.stop()


def test_loaded_egress_uses_bounded_adaptive_turn_reverse_loop():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    egress = _function(tree, "_loaded_egress_before_shipping")
    egress_source = ast.get_source_segment(source, egress)

    rotate_calls = [
        node.func.id
        for node in ast.walk(egress)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == "_bounded_rotate_by_odom"
    ]
    reverse_calls = [
        node.func.id
        for node in ast.walk(egress)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == "_bounded_reverse_by_odom"
    ]

    assert move_shelf_to_ship.LOADED_EGRESS_TURN_STEP == 0.10
    assert move_shelf_to_ship.LOADED_EGRESS_REVERSE_STEP == 0.05
    assert "for round_index in range(" in egress_source
    assert "args.loaded_egress_max_rounds + 1" in egress_source
    assert rotate_calls == ["_bounded_rotate_by_odom"]
    assert reverse_calls == ["_bounded_reverse_by_odom"]
    assert egress_source.count("_settle_without_motion") == 2
    assert "_loaded_dynamic_handoff_ready" in egress_source
    assert "_loaded_turn_segment_safe" in egress_source
    assert "turn = args.loaded_egress_turn_step" in egress_source
    assert "math.copysign" not in egress_source
    assert "_read_loaded_current_risk" in egress_source
    assert "current <= baseline" in egress_source
    assert "current < previous" in egress_source
    assert "previous = current" in egress_source
    assert "current[1] == 0" in egress_source
    assert "args.loaded_egress_max_total_yaw" in egress_source
    assert "args.loaded_egress_max_total_reverse" in egress_source
    assert "args.loaded_egress_max_reverse_per_round" in egress_source
    assert "args.loaded_egress_no_improvement_limit" in egress_source
    assert "LOADED_EGRESS_CLEARANCE_READY" in egress_source
    assert "LOADED_EGRESS_NO_IMPROVEMENT" in egress_source
    assert "LOADED_EGRESS_ROUNDS_EXHAUSTED" in egress_source


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


def test_loaded_egress_turns_then_reverses_until_lethal_risk_recovers(
    monkeypatch,
):
    risks = iter(
        (
            (0, 0, 8, 100),
            (0, 2, 8, 254),
            (0, 1, 8, 254),
            (0, 0, 8, 100),
            (0, 0, 8, 100),
        )
    )
    readiness = iter((False, True))
    rotations = []
    reverses = []

    monkeypatch.setattr(move_shelf_to_ship, "_pose", lambda *_args: object())
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_read_loaded_current_risk",
        lambda *_args: next(risks),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_loaded_dynamic_handoff_ready",
        lambda _navigator, _args, _pose, output: (
            output.append({"yaw_delta": 1.0}) or next(readiness)
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_loaded_turn_segment_safe", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_rotate_by_odom",
        lambda _navigator, _topic, _odom, _base, yaw, *_rest: (
            rotations.append(yaw) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_reverse_by_odom",
        lambda _navigator, _topic, _odom, _base, distance, *_rest: (
            reverses.append(distance) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )

    class Logger:
        def info(self, _message):
            pass

        def error(self, _message):
            pass

    navigator = SimpleNamespace(get_logger=lambda: Logger())
    args = move_shelf_to_ship._parser().parse_args([])

    assert move_shelf_to_ship._loaded_egress_before_shipping(navigator, args)
    assert rotations == pytest.approx([0.10])
    assert reverses == pytest.approx([0.05, 0.05])


def test_loaded_egress_blocked_turn_reverses_without_rotating(monkeypatch):
    risks = iter(
        (
            (0, 0, 8, 100),
            (0, 0, 8, 100),
            (0, 0, 8, 100),
        )
    )
    readiness = iter((False, True))
    rotations = []
    reverses = []

    monkeypatch.setattr(move_shelf_to_ship, "_pose", lambda *_args: object())
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_read_loaded_current_risk",
        lambda *_args: next(risks),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_loaded_dynamic_handoff_ready",
        lambda _navigator, _args, _pose, output: (
            output.append({"yaw_delta": 1.0}) or next(readiness)
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_loaded_turn_segment_safe", lambda *_args: False
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_rotate_by_odom",
        lambda _navigator, _topic, _odom, _base, yaw, *_rest: (
            rotations.append(yaw) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_reverse_by_odom",
        lambda _navigator, _topic, _odom, _base, distance, *_rest: (
            reverses.append(distance) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )

    class Logger:
        def info(self, _message):
            pass

        def error(self, _message):
            pass

    navigator = SimpleNamespace(get_logger=lambda: Logger())
    args = move_shelf_to_ship._parser().parse_args([])

    assert move_shelf_to_ship._loaded_egress_before_shipping(navigator, args)
    assert rotations == []
    assert reverses == pytest.approx([0.05])


def test_extreme_left_experiment_runs_exactly_two_fixed_pairs(
    monkeypatch,
):
    rotations = []
    reverses = []
    risks = iter(((0, 5, 120, 168), (0, 0, 100, 168)))

    monkeypatch.setattr(
        move_shelf_to_ship,
        "_loaded_turn_segment_within_costmap",
        lambda *_args: True,
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_rotate_by_odom",
        lambda _navigator, _topic, _odom, _base, yaw, *_rest: (
            rotations.append(yaw) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_reverse_by_odom",
        lambda _navigator, _topic, _odom, _base, distance, *_rest: (
            reverses.append(distance) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_read_loaded_current_risk",
        lambda *_args: next(risks),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_wait_for_loaded_handoff_clearance",
        lambda *_args: True,
    )

    class Logger:
        def info(self, _message):
            pass

        def warning(self, _message):
            pass

        def error(self, _message):
            pass

    navigator = SimpleNamespace(get_logger=lambda: Logger())
    args = move_shelf_to_ship._parser().parse_args([])

    assert move_shelf_to_ship._loaded_egress_extreme_left_90_experiment(
        navigator, args
    )
    assert rotations == pytest.approx([0.10, 0.20])
    assert reverses == pytest.approx([0.50, 0.70])


def test_extreme_left_experiment_stops_when_a_turn_prefix_is_outside(
    monkeypatch,
):
    rotations = []
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_loaded_turn_segment_within_costmap",
        lambda *_args: False,
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_rotate_by_odom",
        lambda *_args: rotations.append(True) or True,
    )

    class Logger:
        def info(self, _message):
            pass

        def warning(self, _message):
            pass

        def error(self, _message):
            pass

    navigator = SimpleNamespace(get_logger=lambda: Logger())
    args = move_shelf_to_ship._parser().parse_args([])

    assert not move_shelf_to_ship._loaded_egress_extreme_left_90_experiment(
        navigator, args
    )
    assert rotations == []


def test_extreme_left_experiment_stops_if_pair_endpoint_is_outside(
    monkeypatch,
):
    rotations = []
    reverses = []

    monkeypatch.setattr(
        move_shelf_to_ship,
        "_loaded_turn_segment_within_costmap",
        lambda *_args: True,
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_rotate_by_odom",
        lambda _navigator, _topic, _odom, _base, yaw, *_rest: (
            rotations.append(yaw) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_reverse_by_odom",
        lambda _navigator, _topic, _odom, _base, distance, *_rest: (
            reverses.append(distance) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_read_loaded_current_risk",
        lambda *_args: (1, 0, 0, 0),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_wait_for_loaded_handoff_clearance",
        lambda *_args: True,
    )

    class Logger:
        def info(self, _message):
            pass

        def warning(self, _message):
            pass

        def error(self, _message):
            pass

    navigator = SimpleNamespace(get_logger=lambda: Logger())
    args = move_shelf_to_ship._parser().parse_args([])

    assert not move_shelf_to_ship._loaded_egress_extreme_left_90_experiment(
        navigator, args
    )
    assert rotations == pytest.approx([0.10])
    assert reverses == pytest.approx([0.50])


def test_extreme_turn_preview_ignores_lethal_but_rejects_outside():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    helper = _function(tree, "_loaded_turn_segment_within_costmap")
    helper_source = ast.get_source_segment(source, helper)

    assert 'analysis["footprint_outside"]' in helper_source
    assert 'lethal[prefix] = analysis["footprint_lethal"]' in helper_source
    assert 'if analysis["footprint_lethal"]' not in helper_source
    assert "lethal intentionally ignored" in helper_source


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
    reader = _function(tree, "_read_loaded_handoff_clearance")
    gate = _function(tree, "_wait_for_loaded_handoff_clearance")
    reader_source = ast.get_source_segment(source, reader)
    gate_source = ast.get_source_segment(source, gate)

    assert '"/global_costmap/costmap_raw"' in reader_source
    assert '"/global_costmap/published_footprint"' in reader_source
    assert '"/local_costmap/costmap_raw"' in reader_source
    assert '"/local_costmap/published_footprint"' in reader_source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in reader_source
    assert "for subscription in subscriptions" in reader_source
    assert "destroy_subscription(subscription)" in reader_source
    assert "_read_loaded_handoff_clearance" in gate_source
    assert "analyze_costmap_start" in gate_source
    assert 'result["footprint_lethal"]' in gate_source
    assert 'result["footprint_outside"]' in gate_source
    assert "LOADED_HANDOFF_CLEARANCE_BLOCKED" in gate_source
    assert "LOADED_HANDOFF_CLEARANCE_READY" in gate_source


def test_loaded_risk_prioritizes_outside_then_lethal_then_margin():
    assert (0, 0, 120, 168) < (0, 1, 10, 0)
    assert (0, 0, 80, 168) < (0, 0, 120, 102)
    assert (0, 0, 80, 102) < (1, 0, 0, 0)


def test_loaded_turn_segment_checks_only_requested_prefix_on_two_costmaps():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    helper = _function(tree, "_loaded_turn_segment_safe")
    helper_source = ast.get_source_segment(source, helper)

    assert "yaw_delta" in helper_source
    assert 'for prefix in ("global", "local")' in helper_source
    assert "_swept_clearance_analysis" in helper_source
    assert 'analysis["footprint_lethal"]' in helper_source
    assert 'analysis["footprint_outside"]' in helper_source


def test_loaded_dynamic_handoff_requires_path_bearing_and_two_sweeps():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    gate = _function(tree, "_loaded_dynamic_handoff_ready")
    gate_source = ast.get_source_segment(source, gate)

    assert "_read_loaded_handoff_clearance" in gate_source
    assert "_bounded_loaded_shipping_path_probe" in gate_source
    assert "result is PathProbeResult.NO_PATH" in gate_source
    assert "result is PathProbeResult.UNCERTAIN" in gate_source
    assert "_initial_path_bearing" in gate_source
    assert "_lookup_fresh_transform" in gate_source
    assert "_normalize_angle(path_bearing - current_yaw)" in gate_source
    assert 'for prefix in ("global", "local")' in gate_source
    assert "_swept_clearance_analysis" in gate_source
    assert "LOADED_DYNAMIC_HANDOFF_RESULT" in gate_source
    assert "assessment_output.append" in gate_source


def test_loaded_prehandoff_rotation_is_segmented_and_rechecked():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    rotation = _function(tree, "_bounded_loaded_prehandoff_rotation")
    rotation_source = ast.get_source_segment(source, rotation)

    assert "args.loaded_handoff_max_nav_yaw" in rotation_source
    assert "args.loaded_handoff_max_turn_segment" in rotation_source
    assert "args.loaded_handoff_max_total_turn" in rotation_source
    assert "args.loaded_handoff_max_turn_rounds" in rotation_source
    assert "_loaded_dynamic_handoff_ready" in rotation_source
    assert "_bounded_rotate_by_odom" in rotation_source
    assert "_settle_without_motion" in rotation_source
    assert "_wait_for_loaded_localization_stability" in rotation_source
    assert "LOADED_PREHANDOFF_ROTATION_READY" in rotation_source
    assert "LOADED_PREHANDOFF_ROTATION_LIMIT_REJECTED" in rotation_source
    assert "LOADED_PREHANDOFF_ROTATION_ROUNDS_EXHAUSTED" in rotation_source


def test_loaded_prehandoff_rotation_consumes_large_yaw_in_small_segments(
    monkeypatch,
):
    yaw_sequence = iter((-0.30, -0.20, -0.10))
    rotations = []

    def assess(_navigator, _args, _pose, output):
        output.append({"yaw_delta": next(yaw_sequence), "blocked": False})
        return True

    monkeypatch.setattr(
        move_shelf_to_ship, "_loaded_dynamic_handoff_ready", assess
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_bounded_rotate_by_odom",
        lambda _navigator, _topic, _odom, _base, yaw, *_rest: (
            rotations.append(yaw) or True
        ),
    )
    monkeypatch.setattr(
        move_shelf_to_ship, "_settle_without_motion", lambda *_args: True
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "_wait_for_loaded_localization_stability",
        lambda *_args: True,
    )

    class Logger:
        def info(self, _message):
            pass

        def error(self, _message):
            pass

    navigator = SimpleNamespace(get_logger=lambda: Logger())
    args = SimpleNamespace(
        loaded_handoff_max_nav_yaw=0.15,
        loaded_handoff_max_turn_segment=0.10,
        loaded_handoff_max_total_turn=2.80,
        loaded_handoff_max_turn_rounds=8,
        cmd_vel_topic="/cmd_vel",
        odom_frame="odom",
        base_frame="robot_base_footprint",
        loaded_egress_angular_speed=0.05,
        loaded_egress_motion_timeout=20.0,
        odom_lookup_timeout=1.0,
        loaded_egress_yaw_tolerance=0.01,
        exit_settle=1.0,
        loaded_localization_samples=5,
        loaded_localization_sample_interval=0.2,
        shipping_pose_lookup_timeout=5.0,
    )

    assert move_shelf_to_ship._bounded_loaded_prehandoff_rotation(
        navigator, args, PoseStamped(), object()
    )
    assert rotations == pytest.approx([-0.10, -0.10])


def test_swept_clearance_samples_every_intermediate_yaw_and_fails_closed():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    sweep = _function(tree, "_swept_clearance_analysis")
    sweep_source = ast.get_source_segment(source, sweep)

    assert "math.ceil(abs(yaw_delta) / sweep_step)" in sweep_source
    assert "for index in range(sample_count + 1)" in sweep_source
    assert "_rotated_footprint" in sweep_source
    assert "analyze_costmap_start" in sweep_source
    assert 'analysis["footprint_lethal"]' in sweep_source
    assert 'analysis["footprint_outside"]' in sweep_source
    assert 'worst["blocked_sample"] = index' in sweep_source


def test_swept_clearance_detects_lethal_cell_only_entered_mid_turn():
    costmap = Costmap()
    costmap.metadata.resolution = 0.1
    costmap.metadata.size_x = 50
    costmap.metadata.size_y = 50
    costmap.metadata.origin.position.x = 0.0
    costmap.metadata.origin.position.y = 0.0
    costmap.data = [0] * (50 * 50)
    costmap.data[30 * 50 + 27] = 254

    footprint = PolygonStamped()
    for x, y in (
        (3.1, 2.7),
        (1.9, 2.7),
        (1.9, 2.3),
        (3.1, 2.3),
    ):
        point = Point32()
        point.x = x
        point.y = y
        footprint.polygon.points.append(point)

    start = move_shelf_to_ship._swept_clearance_analysis(
        costmap, footprint, 0.0, 0.05
    )
    swept = move_shelf_to_ship._swept_clearance_analysis(
        costmap, footprint, math.pi / 2.0, 0.05
    )

    assert start["footprint_lethal"] == 0
    assert swept["footprint_lethal"] == 1
    assert swept["blocked_sample"] not in (None, 0, swept["sample_count"])


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
    assert "path_output.append(path)" in probe_source
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
