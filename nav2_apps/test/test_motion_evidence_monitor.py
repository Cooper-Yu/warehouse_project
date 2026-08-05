import math
from pathlib import Path
import sys

import pytest
from geometry_msgs.msg import Point32, Quaternion
from nav2_msgs.msg import Costmap


sys.path.insert(0, str(Path(__file__).parents[1]))

from nav2_apps.motion_evidence_monitor import (  # noqa: E402
    MotionEvidenceMonitor,
    analyze_costmap_start,
    phase_from_log,
    yaw_from_quaternion,
)


def test_yaw_from_quaternion():
    quaternion = Quaternion()
    quaternion.z = math.sin(-0.7 / 2.0)
    quaternion.w = math.cos(-0.7 / 2.0)

    assert yaw_from_quaternion(quaternion) == pytest.approx(-0.7)


def test_phase_transitions_cover_navigation_and_shelf_events():
    phase = phase_from_log("Navigating to goal: 5.52 0.041", "IDLE")
    assert phase == "NAV_TO_LOADING"
    phase = phase_from_log("loading_position goal succeeded", phase)
    assert phase == "AT_LOADING"
    phase = phase_from_log("stepwise attach sample: step=1", phase)
    assert phase == "SHELF_APPROACH"
    phase = phase_from_log("locking final center approach", phase)
    assert phase == "CENTER_LOCK"
    phase = phase_from_log("published elevator-up 5 times", phase)
    assert phase == "ELEVATOR_UP"
    phase = phase_from_log(
        "loaded egress: reverse -> left turn -> reverse before shipping",
        phase,
    )
    assert phase == "LOADED_EGRESS"
    phase = phase_from_log("LOADED_EGRESS_COMPLETE", phase)
    assert phase == "SHIPPING_NAV_READY"
    phase = phase_from_log("LOADED_LOCALIZATION_STABLE", phase)
    assert phase == "LOCALIZATION_READY"
    phase = phase_from_log("LOADED_PATH_PROBE_READY: poses=181", phase)
    assert phase == "PATH_PROBE_READY"
    phase = phase_from_log(
        "LOADED_PATH_PROBE_NO_PATH: planner returned no current path",
        phase,
    )
    assert phase == "PATH_PROBE_NO_PATH"
    phase = phase_from_log(
        "LOADED_PATH_PROBE_UNCERTAIN: result timeout", phase
    )
    assert phase == "PATH_PROBE_UNCERTAIN"
    phase = phase_from_log(
        "LOADED_SHIPPING_NO_PATH_AT_ALIGNED_HEADING", phase
    )
    assert phase == "ALIGNED_NO_PATH"
    phase = phase_from_log("LOADED_SHIPPING_SPEEDS_VERIFIED", phase)
    assert phase == "LOADED_SPEEDS_READY"
    phase = phase_from_log("Navigating to shipping_position: 1.98", phase)
    assert phase == "NAV_TO_SHIPPING"
    phase = phase_from_log("Navigating to goal: 1.985 0.924", phase)
    assert phase == "NAV_TO_SHIPPING"
    phase = phase_from_log("shipping_position goal succeeded", phase)
    assert phase == "AT_SHIPPING"
    phase = phase_from_log("published elevator-down 5 times", phase)
    assert phase == "ELEVATOR_DOWN"
    phase = phase_from_log("lower_acceptance_pending", phase)
    assert phase == "LOWER_ACCEPTANCE_PENDING"
    phase = phase_from_log("bounded shelf exit started", phase)
    assert phase == "SHELF_EXIT"
    phase = phase_from_log("CLEAR_OF_SHELF verified", phase)
    assert phase == "CLEAR_OF_SHELF"
    phase = phase_from_log("unloaded_footprint_verified", phase)
    assert phase == "UNLOADED_FOOTPRINT_VERIFIED"
    phase = phase_from_log("Navigating to init_position: 0.0 0.0", phase)
    assert phase == "NAV_TO_INIT"
    phase = phase_from_log("Navigating to goal: 0.0 0.0", phase)
    assert phase == "NAV_TO_INIT"
    phase = phase_from_log("init_position goal succeeded", phase)
    assert phase == "AT_INIT"


def test_unrelated_log_keeps_current_phase():
    assert phase_from_log("controller frequency 20.0", "AT_LOADING") == (
        "AT_LOADING"
    )


def test_probe_result_classification_covers_all_three_states():
    classify = MotionEvidenceMonitor._probe_result

    assert classify("LOADED_PATH_PROBE_READY: poses=181") == "PATH_READY"
    assert classify("LOADED_PATH_PROBE_NO_PATH: planner failed") == "NO_PATH"
    assert classify("LOADED_PATH_PROBE_UNCERTAIN: timeout") == "UNCERTAIN"
    assert classify("controller frequency 20.0") is None


def test_costmap_start_analysis_reports_start_and_footprint_costs():
    costmap = Costmap()
    costmap.metadata.resolution = 1.0
    costmap.metadata.size_x = 4
    costmap.metadata.size_y = 4
    costmap.metadata.origin.position.x = 0.0
    costmap.metadata.origin.position.y = 0.0
    costmap.data = [
        0, 0, 0, 0,
        0, 253, 100, 0,
        0, 254, 255, 0,
        0, 0, 0, 0,
    ]
    footprint = [
        Point32(x=1.0, y=1.0),
        Point32(x=3.0, y=1.0),
        Point32(x=3.0, y=3.0),
        Point32(x=1.0, y=3.0),
    ]

    result = analyze_costmap_start(costmap, footprint, (1.5, 2.5, 0.0))

    assert result["start_grid_x"] == 1
    assert result["start_grid_y"] == 2
    assert result["start_cost"] == 254
    assert result["footprint_cells"] == 4
    assert result["footprint_inflated"] == 1
    assert result["footprint_inscribed"] == 1
    assert result["footprint_lethal"] == 1
    assert result["footprint_unknown"] == 1


def test_costmap_start_analysis_reports_outside_footprint_cells():
    costmap = Costmap()
    costmap.metadata.resolution = 1.0
    costmap.metadata.size_x = 2
    costmap.metadata.size_y = 2
    costmap.data = [0, 0, 0, 0]
    footprint = [
        Point32(x=-1.0, y=0.0),
        Point32(x=1.0, y=0.0),
        Point32(x=1.0, y=1.0),
        Point32(x=-1.0, y=1.0),
    ]

    result = analyze_costmap_start(costmap, footprint, (0.5, 0.5, 0.0))

    assert result["start_cost"] == 0
    assert result["footprint_cells"] == 2
    assert result["footprint_free"] == 1
    assert result["footprint_outside"] == 1


def test_collector_script_is_bounded_and_cleans_up_monitor():
    script = (
        Path(__file__).parents[1] / "scripts" / "collect_motion_evidence"
    ).read_text(encoding="utf-8")

    assert "trap stop_monitor EXIT INT TERM" in script
    assert "kill -INT" in script
    assert "kill -TERM" in script
    assert "monitor_executable" in script
    assert "runtime_logs/motion_" in script
    assert "mission_status=${PIPESTATUS[0]}" in script


def test_monitor_source_keeps_costmap_capture_read_only():
    source = (
        Path(__file__).parents[1]
        / "nav2_apps"
        / "motion_evidence_monitor.py"
    ).read_text(encoding="utf-8")

    assert '"/global_costmap/costmap_raw"' in source
    assert '"/global_costmap/published_footprint"' in source
    assert "lookup_transform" in source
    assert "costmap_snapshots.csv" in source
    assert "create_publisher" not in source
    assert "create_client" not in source
    assert "ActionClient" not in source
