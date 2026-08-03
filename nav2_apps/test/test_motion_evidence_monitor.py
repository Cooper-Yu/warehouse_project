import math
from pathlib import Path
import sys

import pytest
from geometry_msgs.msg import Quaternion


sys.path.insert(0, str(Path(__file__).parents[1]))

from nav2_apps.motion_evidence_monitor import (  # noqa: E402
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
    phase = phase_from_log("Navigating to shipping_position: 1.98", phase)
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
    phase = phase_from_log("init_position goal succeeded", phase)
    assert phase == "AT_INIT"


def test_unrelated_log_keeps_current_phase():
    assert phase_from_log("controller frequency 20.0", "AT_LOADING") == (
        "AT_LOADING"
    )


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
