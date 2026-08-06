"""Checkpoint 12 loading and C9-style shelf-attach orchestration."""

import argparse
from enum import Enum, auto
import math
import sys
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import (
    Point32,
    PolygonStamped,
    PoseStamped,
    TransformStamped,
    Twist,
)
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.utilities import remove_ros_args
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from nav2_apps.pose_config import (
    SIM_INIT_POSE,
    SIM_LOADING_POSE,
    SIM_SHIPPING_POSE,
    optional_initial_pose,
)
from nav2_apps.result_gate import ExitCode, classify_task_result
from nav2_apps.motion_evidence_monitor import analyze_costmap_start


SIM_LOADED_FOOTPRINT = (
    "[[0.40, 0.40], [-0.40, 0.40], "
    "[-0.40, -0.40], [0.40, -0.40]]"
)
SIM_UNLOADED_FOOTPRINT = (
    "[[0.25, 0.25], [-0.25, 0.25], "
    "[-0.25, -0.25], [0.25, -0.25]]"
)
LOADED_EGRESS_TURN_STEP = 0.10
LOADED_EGRESS_REVERSE_STEP = 0.05


class PathProbeResult(Enum):
    """Authoritative result of the current stopped planning probe."""

    PATH_READY = auto()
    NO_PATH = auto()
    UNCERTAIN = auto()


def _load_shelf_service_type():
    """Load the shared service only when detection mode is selected."""
    try:
        from warehouse_interfaces.srv import GoToLoading
    except ImportError as error:
        raise RuntimeError(
            "detection-only requires warehouse_interfaces/srv/GoToLoading"
        ) from error
    return GoToLoading


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send the Checkpoint 12 simulation loading goal and optionally "
            "run the bounded shelf-attach slice. The calibrated simulation "
            "pose may be overridden explicitly."
        )
    )
    parser.add_argument("--loading-x", type=float, default=SIM_LOADING_POSE[0])
    parser.add_argument("--loading-y", type=float, default=SIM_LOADING_POSE[1])
    parser.add_argument(
        "--loading-yaw", type=float, default=SIM_LOADING_POSE[2]
    )
    parser.add_argument("--initial-x", type=float)
    parser.add_argument("--initial-y", type=float)
    parser.add_argument("--initial-yaw", type=float)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--localization-timeout", type=float, default=30.0)
    parser.add_argument(
        "--detection-only",
        action="store_true",
        help=(
            "After loading success, detect the shelf without moving "
            "or lifting."
        ),
    )
    parser.add_argument("--shelf-service", default="/approach_shelf")
    parser.add_argument("--cart-frame", default="cart_frame")
    parser.add_argument("--base-frame", default="robot_base_footprint")
    parser.add_argument("--detection-timeout", type=float, default=15.0)
    parser.add_argument(
        "--approach-and-elevator",
        action="store_true",
        help=(
            "After loading, request the bounded C9-style shelf attach service "
            "and wait for external lift acceptance."
        ),
    )
    parser.add_argument("--attach-timeout", type=float, default=60.0)
    parser.add_argument("--elevator-wait", type=float, default=8.0)
    parser.add_argument(
        "--loaded-footprint-only",
        action="store_true",
        help=(
            "Apply and verify both loaded costmap footprints, then stop."
        ),
    )
    parser.add_argument(
        "--confirm-lift-accepted",
        action="store_true",
        help="Confirm external shelf-lift acceptance for footprint-only mode.",
    )
    parser.add_argument(
        "--confirm-robot-stopped",
        action="store_true",
        help="Confirm that no robot motion or Nav2 goal is active.",
    )
    parser.add_argument(
        "--loaded-footprint",
        default=SIM_LOADED_FOOTPRINT,
    )
    parser.add_argument("--footprint-timeout", type=float, default=10.0)
    parser.add_argument("--footprint-edge-tolerance", type=float, default=0.03)
    parser.add_argument(
        "--shipping-only",
        action="store_true",
        help=(
            "After external lift/stopped confirmation, verify loaded "
            "footprints, navigate to shipping_position, and stop."
        ),
    )
    parser.add_argument(
        "--stop-at-shipping",
        action="store_true",
        help=(
            "In integrated mode, stop successfully after the validated "
            "shipping pose instead of refining placement, lowering, exiting, "
            "and returning to the initial pose."
        ),
    )
    parser.add_argument(
        "--shipping-alignment-only",
        action="store_true",
        help=(
            "From an already stopped shipping pose, verify loaded "
            "footprints and run only bounded final yaw alignment."
        ),
    )
    parser.add_argument(
        "--shipping-x", type=float, default=SIM_SHIPPING_POSE[0]
    )
    parser.add_argument(
        "--shipping-y", type=float, default=SIM_SHIPPING_POSE[1]
    )
    parser.add_argument(
        "--shipping-yaw",
        type=float,
        default=SIM_SHIPPING_POSE[2],
    )
    parser.add_argument("--shipping-timeout", type=float, default=180.0)
    parser.add_argument(
        "--shipping-position-tolerance", type=float, default=0.25
    )
    parser.add_argument(
        "--shipping-yaw-tolerance", type=float, default=0.10
    )
    parser.add_argument(
        "--shipping-max-yaw-correction", type=float, default=0.40
    )
    parser.add_argument(
        "--shipping-alignment-timeout", type=float, default=15.0
    )
    parser.add_argument(
        "--shipping-yaw-correction-ratio", type=float, default=0.5
    )
    parser.add_argument(
        "--shipping-yaw-correction-rounds", type=int, default=3
    )
    parser.add_argument(
        "--shipping-alignment-settle", type=float, default=1.0
    )
    parser.add_argument(
        "--shipping-pose-lookup-timeout", type=float, default=5.0
    )
    parser.add_argument(
        "--loaded-egress-initial-reverse", type=float, default=0.25
    )
    parser.add_argument(
        "--loaded-egress-first-turn-yaw", type=float, default=0.12
    )
    parser.add_argument(
        "--loaded-egress-final-reverse", type=float, default=0.60
    )
    parser.add_argument(
        "--loaded-egress-second-turn-yaw", type=float, default=0.16
    )
    parser.add_argument(
        "--loaded-egress-linear-speed", type=float, default=0.05
    )
    parser.add_argument(
        "--loaded-egress-angular-speed", type=float, default=0.05
    )
    parser.add_argument(
        "--loaded-egress-turn-step",
        type=float,
        default=LOADED_EGRESS_TURN_STEP,
    )
    parser.add_argument(
        "--loaded-egress-reverse-step",
        type=float,
        default=LOADED_EGRESS_REVERSE_STEP,
    )
    parser.add_argument(
        "--loaded-egress-max-total-yaw", type=float, default=0.90
    )
    parser.add_argument(
        "--loaded-egress-max-total-reverse", type=float, default=1.00
    )
    parser.add_argument(
        "--loaded-egress-max-reverse-per-round", type=float, default=0.25
    )
    parser.add_argument(
        "--loaded-egress-max-rounds", type=int, default=12
    )
    parser.add_argument(
        "--loaded-egress-no-improvement-limit", type=int, default=2
    )
    parser.add_argument(
        "--loaded-egress-extreme-left-90-experiment",
        action="store_true",
        help=(
            "Simulation-only diagnostic: run exactly two bounded left-turn "
            "and straight-reverse pairs, then request a direct zero-lethal "
            "Nav2 handoff."
        ),
    )
    parser.add_argument(
        "--loaded-egress-extreme-round1-turn", type=float, default=0.10
    )
    parser.add_argument(
        "--loaded-egress-extreme-round1-reverse", type=float, default=0.50
    )
    parser.add_argument(
        "--loaded-egress-extreme-round2-turn", type=float, default=0.20
    )
    parser.add_argument(
        "--loaded-egress-extreme-round2-reverse", type=float, default=0.70
    )
    parser.add_argument(
        "--loaded-egress-arc-distance", type=float, default=0.35
    )
    parser.add_argument(
        "--loaded-egress-arc-yaw", type=float, default=0.18
    )
    parser.add_argument(
        "--loaded-egress-arc-angular-speed", type=float, default=0.026
    )
    parser.add_argument(
        "--loaded-egress-arc-distance-tolerance", type=float, default=0.01
    )
    parser.add_argument(
        "--loaded-handoff-costmap-timeout", type=float, default=5.0
    )
    parser.add_argument(
        "--loaded-handoff-sweep-step", type=float, default=0.05
    )
    parser.add_argument(
        "--loaded-handoff-path-lookahead", type=float, default=0.30
    )
    parser.add_argument(
        "--loaded-handoff-max-nav-yaw", type=float, default=0.60
    )
    parser.add_argument(
        "--loaded-handoff-max-turn-segment", type=float, default=0.35
    )
    parser.add_argument(
        "--loaded-handoff-max-total-turn", type=float, default=2.80
    )
    parser.add_argument(
        "--loaded-handoff-max-turn-rounds", type=int, default=8
    )
    parser.add_argument(
        "--loaded-egress-motion-timeout", type=float, default=20.0
    )
    parser.add_argument(
        "--loaded-egress-yaw-tolerance", type=float, default=0.01
    )
    parser.add_argument(
        "--loaded-egress-handoff-right-yaw", type=float, default=math.pi / 2.0
    )
    parser.add_argument(
        "--loaded-egress-handoff-angular-speed", type=float, default=0.08
    )
    parser.add_argument(
        "--loaded-egress-handoff-turn-timeout", type=float, default=45.0
    )
    parser.add_argument(
        "--loaded-prealign-max-segment-yaw", type=float, default=0.15
    )
    parser.add_argument(
        "--loaded-prealign-arc-max-distance", type=float, default=0.12
    )
    parser.add_argument(
        "--loaded-prealign-arc-linear-speed", type=float, default=0.04
    )
    parser.add_argument(
        "--loaded-prealign-max-total-yaw", type=float, default=2.80
    )
    parser.add_argument(
        "--loaded-prealign-bearing-tolerance", type=float, default=0.20
    )
    parser.add_argument(
        "--loaded-prealign-max-confirmable-position-jump",
        type=float,
        default=0.23,
    )
    parser.add_argument(
        "--loaded-prealign-max-localization-confirmations",
        type=int,
        default=1,
    )
    parser.add_argument(
        "--loaded-prealign-path-probe-timeout", type=float, default=5.0
    )
    parser.add_argument(
        "--loaded-prealign-path-end-tolerance", type=float, default=0.55
    )
    parser.add_argument(
        "--loaded-prealign-path-handoff-max-bearing",
        type=float,
        default=0.60,
    )
    parser.add_argument(
        "--loaded-prealign-planner-id", default="GridBased"
    )
    parser.add_argument(
        "--loaded-boundary-pair-diagnostic",
        action="store_true",
        help=(
            "At the first stopped auto-start NO_PATH during loaded "
            "prealignment, run one explicit-start pair and exit before "
            "any further motion or navigation handoff."
        ),
    )
    parser.add_argument(
        "--loaded-localization-samples", type=int, default=5
    )
    parser.add_argument(
        "--loaded-localization-sample-interval", type=float, default=0.20
    )
    parser.add_argument(
        "--loaded-localization-max-position-jump", type=float, default=0.20
    )
    parser.add_argument(
        "--loaded-localization-max-yaw-jump", type=float, default=0.20
    )
    parser.add_argument(
        "--loaded-map-odom-freeze-lifecycle-timeout",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--loaded-shipping-max-linear-speed", type=float, default=0.15
    )
    parser.add_argument(
        "--loaded-shipping-max-angular-speed", type=float, default=0.30
    )
    parser.add_argument(
        "--controller-parameter-timeout", type=float, default=5.0
    )
    parser.add_argument(
        "--lower-only",
        action="store_true",
        help=(
            "Verify the stopped loaded state, publish bounded elevator-down "
            "commands, and stop at lower_acceptance_pending."
        ),
    )
    parser.add_argument(
        "--exit-restore-only",
        action="store_true",
        help=(
            "After explicit lowered/stopped confirmation, reverse out, "
            "verify shelf clearance, and restore unloaded footprints."
        ),
    )
    parser.add_argument(
        "--exit-clearance-refine-only",
        action="store_true",
        help=(
            "After the main shelf exit completed but fresh clearance was "
            "slightly short, reverse a small bounded distance, recheck, "
            "and conditionally restore unloaded footprints."
        ),
    )
    parser.add_argument("--confirm-at-shipping", action="store_true")
    parser.add_argument("--confirm-shelf-lowered", action="store_true")
    parser.add_argument("--elevator-down-topic", default="/elevator_down")
    parser.add_argument("--elevator-down-count", type=int, default=5)
    parser.add_argument("--elevator-down-interval", type=float, default=0.1)
    parser.add_argument("--elevator-down-wait", type=float, default=8.0)
    parser.add_argument(
        "--shipping-relift-only",
        action="store_true",
        help=(
            "From a stopped lowered shipping state, publish bounded "
            "elevator-up commands and stop for external acceptance."
        ),
    )
    parser.add_argument(
        "--shipping-forward-refine-only",
        action="store_true",
        help=(
            "After external re-lift acceptance, move the loaded assembly "
            "forward by a bounded odom distance and stop."
        ),
    )
    parser.add_argument("--elevator-up-topic", default="/elevator_up")
    parser.add_argument("--elevator-up-count", type=int, default=5)
    parser.add_argument("--elevator-up-interval", type=float, default=0.1)
    parser.add_argument("--elevator-up-wait", type=float, default=8.0)
    parser.add_argument("--shipping-refine-distance", type=float, default=0.16)
    parser.add_argument("--shipping-refine-speed", type=float, default=0.05)
    parser.add_argument("--shipping-refine-timeout", type=float, default=15.0)
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--exit-distance", type=float, default=0.75)
    parser.add_argument("--exit-speed", type=float, default=0.05)
    parser.add_argument("--exit-timeout", type=float, default=40.0)
    parser.add_argument(
        "--clearance-refine-distance", type=float, default=0.02
    )
    parser.add_argument("--clearance-refine-speed", type=float, default=0.03)
    parser.add_argument(
        "--clearance-refine-motion-timeout", type=float, default=10.0
    )
    parser.add_argument(
        "--confirm-exit-distance-complete", action="store_true"
    )
    parser.add_argument("--exit-heading-tolerance", type=float, default=0.03)
    parser.add_argument("--exit-lateral-tolerance", type=float, default=0.10)
    parser.add_argument("--odom-lookup-timeout", type=float, default=1.0)
    parser.add_argument("--exit-settle", type=float, default=1.0)
    parser.add_argument("--clearance-timeout", type=float, default=10.0)
    parser.add_argument("--clearance-x", type=float, default=0.36)
    parser.add_argument(
        "--unloaded-footprint",
        default=SIM_UNLOADED_FOOTPRINT,
    )
    parser.add_argument(
        "--return-only",
        action="store_true",
        help=(
            "After explicit clear/stopped/unloaded confirmation, verify "
            "unloaded footprints and navigate once to init_position."
        ),
    )
    parser.add_argument("--confirm-clear-of-shelf", action="store_true")
    parser.add_argument("--confirm-unloaded-footprint", action="store_true")
    parser.add_argument("--return-x", type=float, default=SIM_INIT_POSE[0])
    parser.add_argument("--return-y", type=float, default=SIM_INIT_POSE[1])
    parser.add_argument(
        "--return-yaw",
        type=float,
        default=SIM_INIT_POSE[2],
    )
    parser.add_argument("--return-timeout", type=float, default=180.0)
    return parser


def _parse_application_args(
    argv: Optional[List[str]] = None,
) -> argparse.Namespace:
    """Parse mission arguments strictly while preserving ROS arguments."""
    raw_args = sys.argv if argv is None else argv
    application_args = remove_ros_args(args=raw_args)
    if argv is None:
        # remove_ros_args preserves the executable at sys.argv[0].
        application_args = application_args[1:]
    return _parser().parse_args(application_args)


def _pose(
    navigator: BasicNavigator,
    frame_id: str,
    x: float,
    y: float,
    yaw: float,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _wait_for_existing_localization(
    navigator: BasicNavigator,
    timeout: float,
) -> bool:
    """Wait for AMCL without publishing the default origin pose."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not navigator.initial_pose_received:
        if time.monotonic() >= deadline:
            return False
        rclpy.spin_once(navigator, timeout_sec=0.1)
    return navigator.initial_pose_received


def _wait_for_detection(
    navigator: BasicNavigator,
    service_name: str,
    cart_frame: str,
    base_frame: str,
    timeout: float,
) -> bool:
    """Call detection-only and require completion plus a usable TF frame."""
    import tf2_ros

    service_type = _load_shelf_service_type()
    client = navigator.create_client(service_type, service_name)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    try:
        service_deadline = time.monotonic() + timeout
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= service_deadline:
                navigator.get_logger().error(
                    "Shelf detection service was not available"
                )
                return False

        request = service_type.Request()
        request.attach_to_shelf = False
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= service_deadline:
                navigator.get_logger().error(
                    "Shelf detection service timed out"
                )
                return False
            rclpy.spin_once(navigator, timeout_sec=0.1)

        if (
            not future.done()
            or not future.result()
            or not future.result().complete
        ):
            navigator.get_logger().error(
                "Shelf detection did not complete successfully"
            )
            return False

        tf_deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < tf_deadline:
            try:
                buffer.lookup_transform(
                    base_frame, cart_frame, rclpy.time.Time()
                )
                navigator.get_logger().info(
                    "detection-only passed: complete=true and "
                    f"{base_frame} -> {cart_frame} is available"
                )
                return True
            except tf2_ros.TransformException:
                rclpy.spin_once(navigator, timeout_sec=0.1)
    finally:
        del listener
    navigator.get_logger().error(
        "Shelf detection completed but "
        f"{base_frame} -> {cart_frame} TF was unavailable"
    )
    return False


def _request_stepwise_attach(
    navigator: BasicNavigator,
    service_name: str,
    timeout: float,
) -> bool:
    """Request the capability owning bounded motion and the lift command."""
    if timeout <= 0.0:
        navigator.get_logger().error("attach timeout must be positive")
        return False

    service_type = _load_shelf_service_type()
    client = navigator.create_client(service_type, service_name)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
        if time.monotonic() >= deadline:
            navigator.get_logger().error(
                "Shelf attach service was unavailable"
            )
            return False

    request = service_type.Request()
    request.attach_to_shelf = True
    future = client.call_async(request)
    while rclpy.ok() and not future.done():
        if time.monotonic() >= deadline:
            navigator.get_logger().error("Shelf attach service timed out")
            return False
        rclpy.spin_once(navigator, timeout_sec=0.1)

    if (
        not future.done()
        or not future.result()
        or not future.result().complete
    ):
        navigator.get_logger().error(
            "Shelf attach did not complete successfully"
        )
        return False
    navigator.get_logger().info(
        "stepwise attach service passed: motion stopped and elevator-up sent"
    )
    return True


def _wait_for_external_lift_acceptance(
    navigator: BasicNavigator,
    wait_seconds: float,
) -> bool:
    """Wait without claiming that the mechanical lift completed."""
    if wait_seconds <= 0.0:
        navigator.get_logger().error("elevator wait must be positive")
        return False
    deadline = time.monotonic() + wait_seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    navigator.get_logger().warning(
        "elevator-up bounded wait ended; external visual/state acceptance "
        "is required before footprint change"
    )
    return False


def _publish_elevator_down_and_wait(
    navigator: BasicNavigator,
    topic: str,
    count: int,
    interval: float,
    wait_seconds: float,
) -> bool:
    """Publish bounded lowering commands without claiming completion."""
    if count <= 0 or interval < 0.0 or wait_seconds <= 0.0:
        navigator.get_logger().error(
            "elevator-down count/wait must be positive and interval "
            "must be non-negative"
        )
        return False
    publisher = navigator.create_publisher(String, topic, 10)
    message = String()
    message.data = "down"
    try:
        for _ in range(count):
            publisher.publish(message)
            if interval > 0.0:
                time.sleep(interval)
        navigator.get_logger().warning(
            f"published elevator-down {count} times; no programmatic "
            "completion feedback is available"
        )
        deadline = time.monotonic() + wait_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
    finally:
        navigator.destroy_publisher(publisher)
    navigator.get_logger().warning(
        "elevator-down bounded wait ended; external side-view acceptance "
        "is required before shelf exit"
    )
    return True


def _publish_elevator_up_and_wait(
    navigator: BasicNavigator,
    topic: str,
    count: int,
    interval: float,
    wait_seconds: float,
) -> bool:
    """Publish bounded lift commands without claiming completion."""
    if count <= 0 or interval < 0.0 or wait_seconds <= 0.0:
        navigator.get_logger().error(
            "elevator-up count/wait must be positive and interval must be "
            "non-negative"
        )
        return False
    publisher = navigator.create_publisher(String, topic, 10)
    message = String()
    message.data = "up"
    try:
        for _ in range(count):
            publisher.publish(message)
            if interval > 0.0:
                time.sleep(interval)
        navigator.get_logger().warning(
            f"published elevator-up {count} times; no programmatic "
            "completion feedback is available"
        )
        deadline = time.monotonic() + wait_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
    finally:
        navigator.destroy_publisher(publisher)
    navigator.get_logger().warning(
        "elevator-up bounded wait ended; external side-view acceptance is "
        "required before loaded forward refinement"
    )
    return True


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_rotation(rotation) -> float:
    siny_cosp = 2.0 * (
        rotation.w * rotation.z + rotation.x * rotation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _shipping_pose_error(transform, target_x, target_y, target_yaw):
    """Return position and wrap-safe yaw error for shipping acceptance."""
    dx = target_x - transform.transform.translation.x
    dy = target_y - transform.transform.translation.y
    position_error = math.hypot(dx, dy)
    current_yaw = _yaw_from_rotation(transform.transform.rotation)
    yaw_error = _normalize_angle(target_yaw - current_yaw)
    return position_error, yaw_error


def _lookup_fresh_transform(
    navigator: BasicNavigator,
    target_frame: str,
    base_frame: str,
    timeout: float,
):
    """Read a bounded fresh transform after spinning the shared executor."""
    import tf2_ros

    if timeout <= 0.0:
        return None
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            try:
                return buffer.lookup_transform(
                    target_frame, base_frame, rclpy.time.Time()
                )
            except tf2_ros.TransformException:
                continue
    finally:
        del listener
    return None


def localization_step_is_stable(
    previous: tuple,
    current: tuple,
    max_position_jump: float,
    max_yaw_jump: float,
) -> bool:
    """Reject a discontinuous map-to-odom correction step."""
    if max_position_jump <= 0.0 or max_yaw_jump <= 0.0:
        return False
    position_jump = math.hypot(
        current[0] - previous[0], current[1] - previous[1]
    )
    yaw_jump = abs(_normalize_angle(current[2] - previous[2]))
    return position_jump <= max_position_jump and yaw_jump <= max_yaw_jump


class _LoadedLocalizationMonitor:
    """Track direct map-to-odom continuity with one shared TF buffer."""

    def __init__(
        self,
        navigator: BasicNavigator,
        odom_frame: str,
        base_frame: str,
        max_position_jump: float,
        max_yaw_jump: float,
        enforce_jump_limits: bool = True,
    ) -> None:
        import tf2_ros

        self.navigator = navigator
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.max_position_jump = max_position_jump
        self.max_yaw_jump = max_yaw_jump
        self.enforce_jump_limits = enforce_jump_limits
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(
            self.buffer, navigator, spin_thread=False
        )
        self.previous: Optional[tuple] = None
        self.last_position_jump = 0.0
        self.last_yaw_jump = 0.0

    def sample(self) -> Optional[bool]:
        import tf2_ros

        try:
            transform = self.buffer.lookup_transform(
                "map", self.odom_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        current = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            _yaw_from_rotation(transform.transform.rotation),
        )
        if self.previous is None:
            self.previous = current
            return True
        self.last_position_jump = math.hypot(
            current[0] - self.previous[0],
            current[1] - self.previous[1],
        )
        self.last_yaw_jump = abs(
            _normalize_angle(current[2] - self.previous[2])
        )
        stable = localization_step_is_stable(
            self.previous,
            current,
            self.max_position_jump,
            self.max_yaw_jump,
        )
        self.previous = current
        if not stable and not self.enforce_jump_limits:
            self.navigator.get_logger().warning(
                "LOADED_LOCALIZATION_JUMP_OBSERVED_NOT_ENFORCED: "
                f"translation={self.last_position_jump:.3f} "
                f"yaw={self.last_yaw_jump:.3f}"
            )
            return True
        return stable


def _amcl_state(navigator: BasicNavigator, timeout: float) -> Optional[int]:
    client = navigator.create_client(GetState, "/amcl/get_state")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            navigator, future, timeout_sec=timeout
        )
        if not future.done() or future.result() is None:
            return None
        return future.result().current_state.id
    finally:
        navigator.destroy_client(client)


def _change_amcl_state(
    navigator: BasicNavigator,
    transition_id: int,
    expected_state: int,
    timeout: float,
) -> bool:
    client = navigator.create_client(ChangeState, "/amcl/change_state")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            navigator.get_logger().error("AMCL lifecycle service unavailable")
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            navigator, future, timeout_sec=timeout
        )
        if (
            not future.done()
            or future.result() is None
            or not future.result().success
        ):
            navigator.get_logger().error(
                f"AMCL lifecycle transition {transition_id} failed"
            )
            return False
    finally:
        navigator.destroy_client(client)
    return _amcl_state(navigator, timeout) == expected_state


class _FrozenMapOdom:
    """Republish one captured map-to-odom transform while AMCL is inactive."""

    def __init__(
        self,
        navigator: BasicNavigator,
        transform,
        publish_period: float = 0.05,
    ) -> None:
        self.navigator = navigator
        self.transform = transform
        self.broadcaster = TransformBroadcaster(navigator)
        self.timer = navigator.create_timer(publish_period, self.publish)
        self.publish()

    def publish(self) -> None:
        message = TransformStamped()
        message.header.stamp = self.navigator.get_clock().now().to_msg()
        message.header.frame_id = self.transform.header.frame_id
        message.child_frame_id = self.transform.child_frame_id
        message.transform = self.transform.transform
        self.broadcaster.sendTransform(message)

    def stop(self) -> None:
        self.timer.cancel()
        self.navigator.destroy_timer(self.timer)


def _freeze_map_to_odom(
    navigator: BasicNavigator,
    map_frame: str,
    odom_frame: str,
    lookup_timeout: float,
    lifecycle_timeout: float,
) -> Optional[_FrozenMapOdom]:
    import tf2_ros

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + lookup_timeout
    captured = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            try:
                captured = buffer.lookup_transform(
                    map_frame, odom_frame, rclpy.time.Time()
                )
                break
            except tf2_ros.TransformException:
                continue
    finally:
        del listener
    if captured is None:
        navigator.get_logger().error(
            "MAP_ODOM_FREEZE_REJECTED: transform unavailable"
        )
        return None
    if not _change_amcl_state(
        navigator,
        Transition.TRANSITION_DEACTIVATE,
        State.PRIMARY_STATE_INACTIVE,
        lifecycle_timeout,
    ):
        navigator.get_logger().error(
            "MAP_ODOM_FREEZE_REJECTED: AMCL did not become inactive"
        )
        return None
    frozen = _FrozenMapOdom(navigator, captured)
    navigator.get_logger().warning(
        "MAP_ODOM_FREEZE_ACTIVE: explicit simulation experiment only"
    )
    return frozen


def _restore_amcl_after_freeze(
    navigator: BasicNavigator,
    frozen: _FrozenMapOdom,
    lifecycle_timeout: float,
) -> bool:
    frozen.stop()
    restored = _change_amcl_state(
        navigator,
        Transition.TRANSITION_ACTIVATE,
        State.PRIMARY_STATE_ACTIVE,
        lifecycle_timeout,
    )
    if restored:
        navigator.get_logger().warning("MAP_ODOM_FREEZE_RELEASED_AMCL_ACTIVE")
    else:
        navigator.get_logger().error("MAP_ODOM_FREEZE_RELEASE_FAILED")
    return restored


def _wait_for_loaded_localization_stability(
    navigator: BasicNavigator,
    monitor: _LoadedLocalizationMonitor,
    sample_count: int,
    sample_interval: float,
    timeout: float,
) -> bool:
    """Require consecutive available stable map-to-odom samples."""
    if sample_count < 2 or sample_interval <= 0.0 or timeout <= 0.0:
        navigator.get_logger().error(
            "invalid loaded localization gate parameters"
        )
        return False
    deadline = time.monotonic() + timeout
    accepted = 0
    next_sample_at = time.monotonic()
    while rclpy.ok() and time.monotonic() < deadline:
        now = time.monotonic()
        if now < next_sample_at:
            rclpy.spin_once(
                navigator,
                timeout_sec=min(0.1, next_sample_at - now),
            )
            continue
        stable = monitor.sample()
        if stable is None:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            continue
        if not stable:
            navigator.get_logger().error(
                "loaded localization gate rejected: map-to-odom jump "
                f"translation={monitor.last_position_jump:.3f} "
                f"yaw={monitor.last_yaw_jump:.3f}"
            )
            return False
        accepted += 1
        if accepted >= sample_count:
            navigator.get_logger().info(
                "LOADED_LOCALIZATION_STABLE: "
                f"samples={accepted}/{sample_count}"
            )
            return True
        next_sample_at = time.monotonic() + sample_interval
    navigator.get_logger().error(
        "loaded localization gate timed out before stable samples"
    )
    return False


def _prealign_localization_reconfirmation_allowed(
    monitor: _LoadedLocalizationMonitor,
    strict_position_jump: float,
    strict_yaw_jump: float,
    max_confirmable_position_jump: float,
) -> bool:
    """Allow only a narrow stopped translation correction to be rechecked."""
    return (
        strict_position_jump
        < monitor.last_position_jump
        <= max_confirmable_position_jump
        and monitor.last_yaw_jump <= strict_yaw_jump
    )


def _path_reaches_goal(path, frame_id: str, goal, tolerance: float) -> bool:
    """Validate a current finite planner result before motion handoff."""
    if tolerance < 0.0 or path.header.frame_id != frame_id:
        return False
    if len(path.poses) < 2:
        return False
    for pose in (path.poses[0].pose.position, path.poses[-1].pose.position):
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            return False
    endpoint = path.poses[-1].pose.position
    return (
        math.hypot(endpoint.x - goal.pose.position.x,
                   endpoint.y - goal.pose.position.y)
        <= tolerance
    )


def _bounded_loaded_shipping_path_probe(
    navigator: BasicNavigator,
    goal: PoseStamped,
    planner_id: str,
    timeout: float,
    endpoint_tolerance: float,
    path_output: Optional[list] = None,
) -> PathProbeResult:
    """Classify the stopped planning probe without commanding motion."""
    if timeout <= 0.0 or endpoint_tolerance < 0.0 or not planner_id:
        navigator.get_logger().error(
            "LOADED_PATH_PROBE_UNCERTAIN: invalid parameters"
        )
        return PathProbeResult.UNCERTAIN
    deadline = time.monotonic() + timeout
    client = navigator.compute_path_to_pose_client
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_server(timeout_sec=0.1):
            break
    else:
        navigator.get_logger().error(
            "LOADED_PATH_PROBE_UNCERTAIN: planner server timeout"
        )
        return PathProbeResult.UNCERTAIN

    request = ComputePathToPose.Goal()
    request.goal = goal
    request.planner_id = planner_id
    request.use_start = False
    send_future = client.send_goal_async(request)
    while (
        rclpy.ok()
        and not send_future.done()
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(navigator, timeout_sec=0.1)
    if not send_future.done() or send_future.result() is None:
        navigator.get_logger().error(
            "LOADED_PATH_PROBE_UNCERTAIN: goal acceptance timeout"
        )
        return PathProbeResult.UNCERTAIN
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        navigator.get_logger().warning(
            "LOADED_PATH_PROBE_UNCERTAIN: goal rejected"
        )
        return PathProbeResult.UNCERTAIN

    result_future = goal_handle.get_result_async()
    while (
        rclpy.ok()
        and not result_future.done()
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(navigator, timeout_sec=0.1)
    if not result_future.done():
        cancel_future = goal_handle.cancel_goal_async()
        cancel_deadline = time.monotonic() + 1.0
        while (
            rclpy.ok()
            and not cancel_future.done()
            and time.monotonic() < cancel_deadline
        ):
            rclpy.spin_once(navigator, timeout_sec=0.1)
        navigator.get_logger().error(
            "LOADED_PATH_PROBE_UNCERTAIN: result timeout; cancel requested"
        )
        return PathProbeResult.UNCERTAIN

    wrapped_result = result_future.result()
    if wrapped_result is None:
        navigator.get_logger().error(
            "LOADED_PATH_PROBE_UNCERTAIN: result missing"
        )
        return PathProbeResult.UNCERTAIN
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
        navigator.get_logger().info(
            "LOADED_PATH_PROBE_NO_PATH: planner returned no current path"
        )
        return PathProbeResult.NO_PATH
    path = wrapped_result.result.path
    if not _path_reaches_goal(
        path, goal.header.frame_id, goal, endpoint_tolerance
    ):
        navigator.get_logger().warning(
            "LOADED_PATH_PROBE_UNCERTAIN: invalid successful path"
        )
        return PathProbeResult.UNCERTAIN
    navigator.get_logger().info(
        f"LOADED_PATH_PROBE_READY: poses={len(path.poses)}"
    )
    if path_output is not None:
        path_output.append(path)
    return PathProbeResult.PATH_READY


def _wait_parameter_future(navigator, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done():
        if time.monotonic() >= deadline:
            return None
        rclpy.spin_once(navigator, timeout_sec=0.1)
    return future.result() if future.done() else None


def _controller_speed_snapshot(
    navigator: BasicNavigator, timeout: float
) -> Optional[dict]:
    client = navigator.create_client(
        GetParameters, "/controller_server/get_parameters"
    )
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    request = GetParameters.Request()
    request.names = [
        "FollowPath.max_vel_x",
        "FollowPath.max_speed_xy",
        "FollowPath.max_vel_theta",
    ]
    response = _wait_parameter_future(
        navigator, client.call_async(request), timeout
    )
    if response is None or len(response.values) != 3:
        return None
    values = [value.double_value for value in response.values]
    if any(
        value.type != ParameterType.PARAMETER_DOUBLE
        for value in response.values
    ):
        return None
    return dict(zip(request.names, values))


def _set_controller_speeds(
    navigator: BasicNavigator, values: dict, timeout: float
) -> bool:
    client = navigator.create_client(
        SetParameters, "/controller_server/set_parameters"
    )
    if not client.wait_for_service(timeout_sec=timeout):
        return False
    request = SetParameters.Request()
    for name, value in values.items():
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )
        request.parameters.append(parameter)
    response = _wait_parameter_future(
        navigator, client.call_async(request), timeout
    )
    return (
        response is not None
        and len(response.results) == len(request.parameters)
        and all(result.successful for result in response.results)
    )


def _accept_or_align_shipping_pose(
    navigator: BasicNavigator,
    shipping_pose: PoseStamped,
    base_frame: str,
    position_tolerance: float,
    yaw_tolerance: float,
    max_yaw_correction: float,
    alignment_timeout: float,
    alignment_settle: float,
    lookup_timeout: float,
    correction_ratio: float = 0.5,
    max_correction_rounds: int = 3,
) -> bool:
    """Accept shipping pose or run bounded partial Nav2 Spin corrections."""
    if (
        position_tolerance <= 0.0
        or yaw_tolerance < 0.0
        or max_yaw_correction <= 0.0
        or alignment_timeout <= 0.0
        or alignment_settle < 0.0
        or lookup_timeout <= 0.0
        or correction_ratio <= 0.0
        or correction_ratio >= 1.0
        or max_correction_rounds <= 0
    ):
        navigator.get_logger().error(
            "invalid shipping alignment parameters"
        )
        return False

    target_x = shipping_pose.pose.position.x
    target_y = shipping_pose.pose.position.y
    target_yaw = _yaw_from_rotation(shipping_pose.pose.orientation)
    deadline = time.monotonic() + alignment_timeout
    for round_index in range(max_correction_rounds + 1):
        if time.monotonic() >= deadline:
            navigator.get_logger().error(
                "shipping fine-yaw alignment deadline exhausted"
            )
            return False
        if not _settle_without_motion(navigator, alignment_settle):
            return False
        transform = _lookup_fresh_transform(
            navigator,
            shipping_pose.header.frame_id,
            base_frame,
            min(lookup_timeout, max(0.001, deadline - time.monotonic())),
        )
        if transform is None:
            navigator.get_logger().error(
                "shipping acceptance failed: fresh map pose unavailable"
            )
            return False
        position_error, yaw_error = _shipping_pose_error(
            transform, target_x, target_y, target_yaw
        )
        navigator.get_logger().info(
            "shipping pose observation: "
            f"round={round_index}/{max_correction_rounds} "
            f"position_error={position_error:.3f} "
            f"yaw_error={yaw_error:.3f}"
        )
        if position_error > position_tolerance:
            navigator.get_logger().error(
                "shipping alignment rejected: position is outside the "
                f"acceptance radius ({position_error:.3f}>"
                f"{position_tolerance:.3f})"
            )
            return False
        if abs(yaw_error) <= yaw_tolerance:
            return True
        if abs(yaw_error) > max_yaw_correction:
            navigator.get_logger().error(
                "shipping alignment rejected: required yaw correction "
                f"exceeds bound ({abs(yaw_error):.3f}>"
                f"{max_yaw_correction:.3f})"
            )
            return False
        if round_index >= max_correction_rounds:
            break

        spin_dist = correction_ratio * yaw_error
        remaining = deadline - time.monotonic()
        navigator.get_logger().warning(
            "shipping partial fine-yaw correction requested through "
            f"Nav2 Spin: round={round_index + 1}/"
            f"{max_correction_rounds} measured_yaw_error="
            f"{yaw_error:.3f} spin_dist={spin_dist:.3f}"
        )
        if not navigator.spin(
            spin_dist=spin_dist,
            time_allowance=max(1, int(math.ceil(remaining))),
        ):
            navigator.get_logger().error(
                "shipping fine-yaw Spin goal was rejected"
            )
            return False
        while not navigator.isTaskComplete():
            if time.monotonic() >= deadline:
                navigator.cancelTask()
                navigator.get_logger().error(
                    "shipping fine-yaw Spin timed out; cancel requested"
                )
                return False
            rclpy.spin_once(navigator, timeout_sec=0.1)
        spin_result = classify_task_result(navigator.getResult(), TaskResult)
        if spin_result != ExitCode.SUCCEEDED:
            navigator.get_logger().error(
                "shipping fine-yaw Spin did not succeed"
            )
            return False

    navigator.get_logger().error(
        "shipping final pose rejected after bounded partial Spin loop: "
        f"position_error={position_error:.3f}/{position_tolerance:.3f} "
        f"yaw_error={abs(yaw_error):.3f}/{yaw_tolerance:.3f} "
        f"rounds={max_correction_rounds}"
    )
    return False


def _exit_progress(
    start_x: float,
    start_y: float,
    start_yaw: float,
    current_x: float,
    current_y: float,
):
    """Project odom displacement onto the starting reverse/lateral axes."""
    dx = current_x - start_x
    dy = current_y - start_y
    reverse = -(dx * math.cos(start_yaw) + dy * math.sin(start_yaw))
    lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
    return reverse, lateral


def _forward_progress(
    start_x: float,
    start_y: float,
    start_yaw: float,
    current_x: float,
    current_y: float,
):
    """Project odom displacement onto the starting forward/lateral axes."""
    dx = current_x - start_x
    dy = current_y - start_y
    forward = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
    lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
    return forward, lateral


def _bounded_forward_by_odom(
    navigator: BasicNavigator,
    cmd_vel_topic: str,
    odom_frame: str,
    base_frame: str,
    distance: float,
    speed: float,
    timeout: float,
    lookup_timeout: float,
    heading_tolerance: float,
    lateral_tolerance: float,
) -> bool:
    """Move a measured local-x distance with heading/lateral guards."""
    import tf2_ros

    if (
        distance <= 0.0
        or speed <= 0.0
        or timeout <= 0.0
        or lookup_timeout <= 0.0
        or heading_tolerance < 0.0
        or lateral_tolerance < 0.0
    ):
        navigator.get_logger().error("invalid bounded-refinement parameters")
        return False

    publisher = navigator.create_publisher(Twist, cmd_vel_topic, 10)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout

    def lookup():
        try:
            return buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None

    start = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            start = lookup()
            if start is not None:
                break
        if start is None:
            navigator.get_logger().error(
                "shipping forward refinement rejected: odom TF unavailable"
            )
            return False

        start_x = start.transform.translation.x
        start_y = start.transform.translation.y
        start_yaw = _yaw_from_rotation(start.transform.rotation)
        command = Twist()
        command.linear.x = speed
        last_tf_time = time.monotonic()
        forward_progress = 0.0
        lateral = 0.0
        navigator.get_logger().info(
            "bounded shipping forward refinement started: "
            f"target_distance={distance:.3f} speed={speed:.3f} "
            f"accepted_odom_yaw={start_yaw:.3f}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            current = lookup()
            if current is None:
                if time.monotonic() - last_tf_time >= lookup_timeout:
                    navigator.get_logger().error(
                        "shipping forward refinement stopped: odom TF stale"
                    )
                    return False
                publisher.publish(command)
                continue

            last_tf_time = time.monotonic()
            forward_progress, lateral = _forward_progress(
                start_x,
                start_y,
                start_yaw,
                current.transform.translation.x,
                current.transform.translation.y,
            )
            current_yaw = _yaw_from_rotation(current.transform.rotation)
            heading_drift = _normalize_angle(current_yaw - start_yaw)
            if abs(heading_drift) > heading_tolerance:
                navigator.get_logger().error(
                    "shipping forward refinement stopped: heading drift "
                    f"{heading_drift:.3f} exceeds {heading_tolerance:.3f}"
                )
                return False
            if abs(lateral) > lateral_tolerance:
                navigator.get_logger().error(
                    "shipping forward refinement stopped: lateral drift "
                    f"{lateral:.3f} exceeds {lateral_tolerance:.3f}"
                )
                return False
            if forward_progress >= distance:
                navigator.get_logger().info(
                    "bounded shipping forward refinement complete: "
                    f"target={distance:.3f} progress={forward_progress:.3f} "
                    f"lateral={lateral:.3f} "
                    f"heading_drift={heading_drift:.3f}"
                )
                return True
            publisher.publish(command)

        navigator.get_logger().error(
            "shipping forward refinement timed out before odom target: "
            f"target={distance:.3f} progress={forward_progress:.3f}"
        )
        return False
    finally:
        stop = Twist()
        for _ in range(3):
            publisher.publish(stop)
            time.sleep(0.05)
        navigator.destroy_publisher(publisher)
        del listener


def _bounded_reverse_by_odom(
    navigator: BasicNavigator,
    cmd_vel_topic: str,
    odom_frame: str,
    base_frame: str,
    distance: float,
    speed: float,
    timeout: float,
    lookup_timeout: float,
    heading_tolerance: float,
    lateral_tolerance: float,
    motion_name: str = "shelf exit",
) -> bool:
    """Reverse a measured local-x distance with heading/lateral guards."""
    import tf2_ros

    if (
        distance <= 0.0
        or speed <= 0.0
        or timeout <= 0.0
        or lookup_timeout <= 0.0
        or heading_tolerance < 0.0
        or lateral_tolerance < 0.0
    ):
        navigator.get_logger().error("invalid bounded-exit parameters")
        return False

    publisher = navigator.create_publisher(Twist, cmd_vel_topic, 10)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout

    def lookup():
        try:
            return buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None

    start = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            start = lookup()
            if start is not None:
                break
        if start is None:
            navigator.get_logger().error(
                f"bounded {motion_name} rejected: odom TF unavailable"
            )
            return False

        start_x = start.transform.translation.x
        start_y = start.transform.translation.y
        start_yaw = _yaw_from_rotation(start.transform.rotation)
        command = Twist()
        command.linear.x = -speed
        last_tf_time = time.monotonic()
        reverse_progress = 0.0
        lateral = 0.0
        navigator.get_logger().info(
            f"bounded {motion_name} started: "
            f"target_distance={distance:.3f} speed={command.linear.x:.3f} "
            f"accepted_odom_yaw={start_yaw:.3f}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            current = lookup()
            if current is None:
                if time.monotonic() - last_tf_time >= lookup_timeout:
                    navigator.get_logger().error(
                        f"bounded {motion_name} stopped: odom TF stale"
                    )
                    return False
                publisher.publish(command)
                continue

            last_tf_time = time.monotonic()
            reverse_progress, lateral = _exit_progress(
                start_x,
                start_y,
                start_yaw,
                current.transform.translation.x,
                current.transform.translation.y,
            )
            current_yaw = _yaw_from_rotation(current.transform.rotation)
            heading_drift = _normalize_angle(current_yaw - start_yaw)
            if abs(heading_drift) > heading_tolerance:
                navigator.get_logger().error(
                    f"bounded {motion_name} stopped: heading drift "
                    f"{heading_drift:.3f} exceeds "
                    f"{heading_tolerance:.3f}"
                )
                return False
            if abs(lateral) > lateral_tolerance:
                navigator.get_logger().error(
                    f"bounded {motion_name} stopped: lateral drift "
                    f"{lateral:.3f} exceeds {lateral_tolerance:.3f}"
                )
                return False
            if reverse_progress >= distance:
                navigator.get_logger().info(
                    f"bounded {motion_name} odom target complete: "
                    f"target={distance:.3f} "
                    f"reverse_progress={reverse_progress:.3f} "
                    f"lateral={lateral:.3f} "
                    f"heading_drift={heading_drift:.3f}"
                )
                return True
            publisher.publish(command)

        navigator.get_logger().error(
            f"bounded {motion_name} timed out before odom target: "
            f"target={distance:.3f} progress={reverse_progress:.3f}"
        )
        return False
    finally:
        stop = Twist()
        for _ in range(3):
            publisher.publish(stop)
            time.sleep(0.05)
        navigator.destroy_publisher(publisher)
        del listener


def _bounded_rotate_by_odom(
    navigator: BasicNavigator,
    cmd_vel_topic: str,
    odom_frame: str,
    base_frame: str,
    yaw: float,
    speed: float,
    timeout: float,
    lookup_timeout: float,
    yaw_tolerance: float,
) -> bool:
    """Rotate through a measured signed yaw and stop fail-closed."""
    import tf2_ros

    target = abs(yaw)
    if (
        target <= 0.0
        or speed <= 0.0
        or timeout <= 0.0
        or lookup_timeout <= 0.0
        or yaw_tolerance < 0.0
    ):
        navigator.get_logger().error("invalid loaded-egress turn parameters")
        return False

    publisher = navigator.create_publisher(Twist, cmd_vel_topic, 10)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout

    def lookup():
        try:
            return buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None

    start = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            start = lookup()
            if start is not None:
                break
        if start is None:
            navigator.get_logger().error(
                "loaded-egress turn rejected: odom TF unavailable"
            )
            return False

        direction = math.copysign(1.0, yaw)
        last_yaw = _yaw_from_rotation(start.transform.rotation)
        traveled = 0.0
        last_tf_time = time.monotonic()
        command = Twist()
        command.angular.z = direction * speed
        navigator.get_logger().info(
            "bounded loaded-egress turn started: "
            f"target_yaw={yaw:.3f} speed={command.angular.z:.3f}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            current = lookup()
            if current is None:
                if time.monotonic() - last_tf_time >= lookup_timeout:
                    navigator.get_logger().error(
                        "loaded-egress turn stopped: odom TF stale"
                    )
                    return False
                publisher.publish(command)
                continue

            last_tf_time = time.monotonic()
            current_yaw = _yaw_from_rotation(current.transform.rotation)
            delta = _normalize_angle(current_yaw - last_yaw)
            last_yaw = current_yaw
            traveled += direction * delta
            if traveled >= max(0.0, target - yaw_tolerance):
                navigator.get_logger().info(
                    "bounded loaded-egress turn complete: "
                    f"target={yaw:.3f} traveled={direction * traveled:.3f}"
                )
                return True
            publisher.publish(command)

        navigator.get_logger().error(
            "loaded-egress turn timed out before odom target: "
            f"target={yaw:.3f} traveled={direction * traveled:.3f}"
        )
        return False
    finally:
        stop = Twist()
        for _ in range(3):
            publisher.publish(stop)
            time.sleep(0.05)
        navigator.destroy_publisher(publisher)
        del listener


def _bounded_forward_right_arc_by_odom(
    navigator: BasicNavigator,
    cmd_vel_topic: str,
    odom_frame: str,
    base_frame: str,
    max_distance: float,
    max_right_yaw: float,
    linear_speed: float,
    angular_speed: float,
    timeout: float,
    lookup_timeout: float,
    yaw_tolerance: float,
) -> bool:
    """Move on a bounded forward-right arc, stopping at either limit."""
    import tf2_ros

    if (
        max_distance <= 0.0
        or max_right_yaw <= 0.0
        or linear_speed <= 0.0
        or angular_speed <= 0.0
        or timeout <= 0.0
        or lookup_timeout <= 0.0
        or yaw_tolerance < 0.0
    ):
        navigator.get_logger().error(
            "invalid loaded prealignment arc parameters"
        )
        return False

    publisher = navigator.create_publisher(Twist, cmd_vel_topic, 10)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout

    def lookup():
        try:
            return buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None

    start = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            start = lookup()
            if start is not None:
                break
        if start is None:
            navigator.get_logger().error(
                "loaded prealignment arc rejected: odom TF unavailable"
            )
            return False

        start_x = start.transform.translation.x
        start_y = start.transform.translation.y
        last_yaw = _yaw_from_rotation(start.transform.rotation)
        turned_right = 0.0
        distance = 0.0
        last_tf_time = time.monotonic()
        command = Twist()
        command.linear.x = linear_speed
        command.angular.z = -angular_speed
        navigator.get_logger().info(
            "bounded loaded prealignment forward-right arc started: "
            f"max_distance={max_distance:.3f} "
            f"max_right_yaw={max_right_yaw:.3f} "
            f"linear={linear_speed:.3f} angular={-angular_speed:.3f}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            current = lookup()
            if current is None:
                if time.monotonic() - last_tf_time >= lookup_timeout:
                    navigator.get_logger().error(
                        "loaded prealignment arc stopped: odom TF stale"
                    )
                    return False
                publisher.publish(command)
                continue

            last_tf_time = time.monotonic()
            current_x = current.transform.translation.x
            current_y = current.transform.translation.y
            distance = math.hypot(current_x - start_x, current_y - start_y)
            current_yaw = _yaw_from_rotation(current.transform.rotation)
            delta = _normalize_angle(current_yaw - last_yaw)
            last_yaw = current_yaw
            turned_right += -delta
            distance_done = distance >= max_distance
            yaw_done = turned_right >= max(
                0.0, max_right_yaw - yaw_tolerance
            )
            if distance_done or yaw_done:
                navigator.get_logger().info(
                    "bounded loaded prealignment forward-right arc complete: "
                    f"distance={distance:.3f}/{max_distance:.3f} "
                    f"right_yaw={turned_right:.3f}/{max_right_yaw:.3f}"
                )
                return True
            if turned_right < -yaw_tolerance:
                navigator.get_logger().error(
                    "loaded prealignment arc stopped: wrong yaw direction"
                )
                return False
            publisher.publish(command)

        navigator.get_logger().error(
            "loaded prealignment arc timed out: "
            f"distance={distance:.3f} right_yaw={turned_right:.3f}"
        )
        return False
    finally:
        stop = Twist()
        for _ in range(3):
            publisher.publish(stop)
            time.sleep(0.05)
        navigator.destroy_publisher(publisher)
        del listener


def _bounded_reverse_arc_by_odom(
    navigator: BasicNavigator,
    cmd_vel_topic: str,
    odom_frame: str,
    base_frame: str,
    target_distance: float,
    target_yaw: float,
    linear_speed: float,
    angular_speed: float,
    timeout: float,
    lookup_timeout: float,
    distance_tolerance: float,
    yaw_tolerance: float,
    label: str,
) -> bool:
    """Reverse on a signed-yaw arc until both odom targets are reached."""
    import tf2_ros

    if (
        target_distance <= 0.0
        or target_yaw == 0.0
        or linear_speed <= 0.0
        or angular_speed <= 0.0
        or timeout <= 0.0
        or lookup_timeout <= 0.0
        or distance_tolerance < 0.0
        or yaw_tolerance < 0.0
    ):
        navigator.get_logger().error(
            f"invalid {label} reverse-arc parameters"
        )
        return False

    publisher = navigator.create_publisher(Twist, cmd_vel_topic, 10)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout

    def lookup():
        try:
            return buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None

    try:
        start = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            start = lookup()
            if start is not None:
                break
        if start is None:
            navigator.get_logger().error(
                f"{label} rejected: odom TF unavailable"
            )
            return False

        previous_x = start.transform.translation.x
        previous_y = start.transform.translation.y
        last_yaw = _yaw_from_rotation(start.transform.rotation)
        distance = 0.0
        yaw_direction = math.copysign(1.0, target_yaw)
        yaw_traveled = 0.0
        last_tf_time = time.monotonic()
        command = Twist()
        command.linear.x = -linear_speed
        command.angular.z = yaw_direction * angular_speed
        navigator.get_logger().info(
            f"bounded {label} started: distance={target_distance:.3f} "
            f"yaw={target_yaw:.3f} linear={command.linear.x:.3f} "
            f"angular={command.angular.z:.3f}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            current = lookup()
            if current is None:
                if time.monotonic() - last_tf_time >= lookup_timeout:
                    navigator.get_logger().error(
                        f"{label} stopped: odom TF stale"
                    )
                    return False
                publisher.publish(command)
                continue

            last_tf_time = time.monotonic()
            current_x = current.transform.translation.x
            current_y = current.transform.translation.y
            distance += math.hypot(
                current_x - previous_x, current_y - previous_y
            )
            previous_x = current_x
            previous_y = current_y
            current_yaw = _yaw_from_rotation(current.transform.rotation)
            delta = _normalize_angle(current_yaw - last_yaw)
            last_yaw = current_yaw
            yaw_traveled += yaw_direction * delta
            if yaw_traveled < -yaw_tolerance:
                navigator.get_logger().error(
                    f"{label} stopped: wrong yaw direction"
                )
                return False

            distance_done = distance >= max(
                0.0, target_distance - distance_tolerance
            )
            yaw_done = yaw_traveled >= max(
                0.0, abs(target_yaw) - yaw_tolerance
            )
            if distance_done and yaw_done:
                navigator.get_logger().info(
                    f"bounded {label} complete: "
                    f"distance={distance:.3f}/{target_distance:.3f} "
                    f"yaw={yaw_direction * yaw_traveled:.3f}/{target_yaw:.3f}"
                )
                return True
            publisher.publish(command)

        navigator.get_logger().error(
            f"{label} timed out: distance={distance:.3f}/"
            f"{target_distance:.3f} yaw={yaw_direction * yaw_traveled:.3f}/"
            f"{target_yaw:.3f}"
        )
        return False
    finally:
        stop = Twist()
        for _ in range(3):
            publisher.publish(stop)
            time.sleep(0.05)
        navigator.destroy_publisher(publisher)
        del listener


def _loaded_egress_before_shipping(
    navigator: BasicNavigator, args: argparse.Namespace
) -> bool:
    """Adapt left turns and reverse steps using live costmap risk."""
    navigator.get_logger().info(
        "loaded egress adaptive clearance: precheck left turn, then reverse "
        "in risk-reducing steps"
    )
    if (
        args.loaded_egress_turn_step <= 0.0
        or args.loaded_egress_reverse_step <= 0.0
        or args.loaded_egress_max_total_yaw <= 0.0
        or args.loaded_egress_max_total_reverse <= 0.0
        or args.loaded_egress_max_reverse_per_round <= 0.0
        or args.loaded_egress_max_rounds <= 0
        or args.loaded_egress_no_improvement_limit <= 0
    ):
        navigator.get_logger().error(
            "LOADED_EGRESS_ADAPTIVE_UNCERTAIN: invalid limits"
        )
        return False
    total_yaw = 0.0
    total_reverse = 0.0
    shipping_pose = _pose(
        navigator,
        args.frame_id,
        args.shipping_x,
        args.shipping_y,
        args.shipping_yaw,
    )
    for round_index in range(1, args.loaded_egress_max_rounds + 1):
        risk = _read_loaded_current_risk(
            navigator, args.loaded_handoff_costmap_timeout
        )
        if risk is None or risk[0] != 0:
            return False
        assessments = []
        readiness = _loaded_dynamic_handoff_ready(
            navigator, args, shipping_pose, assessments
        )
        if readiness is None or not assessments:
            return False
        if readiness:
            navigator.get_logger().info(
                "LOADED_EGRESS_CLEARANCE_READY: "
                f"round={round_index} yaw={total_yaw:.3f} "
                f"reverse={total_reverse:.3f}"
            )
            return True

        turn = args.loaded_egress_turn_step
        turn_safe = _loaded_turn_segment_safe(
            navigator, args, turn
        )
        turned = False
        if turn_safe is None:
            return False
        if turn_safe:
            if total_yaw + abs(turn) > (
                args.loaded_egress_max_total_yaw + 1e-9
            ):
                navigator.get_logger().error(
                    "LOADED_EGRESS_YAW_LIMIT_REJECTED"
                )
                return False
            if not _bounded_rotate_by_odom(
                navigator,
                args.cmd_vel_topic,
                args.odom_frame,
                args.base_frame,
                turn,
                args.loaded_egress_angular_speed,
                args.loaded_egress_motion_timeout,
                args.odom_lookup_timeout,
                args.loaded_egress_yaw_tolerance,
            ):
                return False
            total_yaw += abs(turn)
            turned = True
            if not _settle_without_motion(navigator, args.exit_settle):
                return False
        else:
            navigator.get_logger().info(
                "LOADED_EGRESS_TURN_DEFERRED: next segment blocked; "
                "reverse first"
            )

        baseline = risk
        previous = baseline
        if turned:
            previous = _read_loaded_current_risk(
                navigator, args.loaded_handoff_costmap_timeout
            )
            if previous is None or previous[0] != 0:
                return False
        reverse_this_round = 0.0
        no_improvement = 0
        while reverse_this_round + 1e-9 < (
            args.loaded_egress_max_reverse_per_round
        ):
            if total_reverse + args.loaded_egress_reverse_step > (
                args.loaded_egress_max_total_reverse + 1e-9
            ):
                navigator.get_logger().error(
                    "LOADED_EGRESS_REVERSE_LIMIT_REJECTED"
                )
                return False
            if not _bounded_reverse_by_odom(
                navigator,
                args.cmd_vel_topic,
                args.odom_frame,
                args.base_frame,
                args.loaded_egress_reverse_step,
                args.loaded_egress_linear_speed,
                args.loaded_egress_motion_timeout,
                args.odom_lookup_timeout,
                args.exit_heading_tolerance,
                args.exit_lateral_tolerance,
                f"loaded adaptive egress round {round_index}",
            ):
                return False
            total_reverse += args.loaded_egress_reverse_step
            reverse_this_round += args.loaded_egress_reverse_step
            if not _settle_without_motion(navigator, args.exit_settle):
                return False
            current = _read_loaded_current_risk(
                navigator, args.loaded_handoff_costmap_timeout
            )
            if current is None or current[0] != 0:
                return False
            if current < previous:
                no_improvement = 0
            else:
                no_improvement += 1
            navigator.get_logger().info(
                "LOADED_EGRESS_RISK_RESULT: "
                f"round={round_index} turned={turned} "
                f"baseline={baseline} current={current} "
                f"round_reverse={reverse_this_round:.3f}"
            )
            if current <= baseline and current[1] == 0:
                break
            if no_improvement >= args.loaded_egress_no_improvement_limit:
                navigator.get_logger().error(
                    "LOADED_EGRESS_NO_IMPROVEMENT"
                )
                return False
            previous = current
        else:
            navigator.get_logger().error(
                "LOADED_EGRESS_ROUND_REVERSE_EXHAUSTED"
            )
            return False
    navigator.get_logger().error("LOADED_EGRESS_ROUNDS_EXHAUSTED")
    return False


def _loaded_egress_extreme_left_90_experiment(
    navigator: BasicNavigator, args: argparse.Namespace
) -> bool:
    """Run the explicit simulation-only two-pair geometry experiment."""
    pairs = (
        (
            args.loaded_egress_extreme_round1_turn,
            args.loaded_egress_extreme_round1_reverse,
        ),
        (
            args.loaded_egress_extreme_round2_turn,
            args.loaded_egress_extreme_round2_reverse,
        ),
    )
    if any(turn <= 0.0 or reverse <= 0.0 for turn, reverse in pairs):
        navigator.get_logger().error(
            "LOADED_EGRESS_EXTREME_UNCERTAIN: invalid experiment limits"
        )
        return False

    navigator.get_logger().warning(
        "LOADED_EGRESS_EXTREME_BEGIN: simulation-only fixed pairs "
        f"round1=({pairs[0][0]:.3f},{pairs[0][1]:.3f}) "
        f"round2=({pairs[1][0]:.3f},{pairs[1][1]:.3f})"
    )
    total_yaw = 0.0
    total_reverse = 0.0
    for round_index, (segment, reverse_distance) in enumerate(pairs, 1):
        turn_within_costmap = _loaded_turn_segment_within_costmap(
            navigator, args, segment
        )
        if turn_within_costmap is None:
            return False
        if not turn_within_costmap:
            navigator.get_logger().error(
                "LOADED_EGRESS_EXTREME_TURN_OUTSIDE: "
                f"round={round_index} total_yaw={total_yaw:.3f} "
                f"next={segment:.3f}"
            )
            return False
        if not _bounded_rotate_by_odom(
            navigator,
            args.cmd_vel_topic,
            args.odom_frame,
            args.base_frame,
            segment,
            args.loaded_egress_angular_speed,
            args.loaded_egress_motion_timeout,
            args.odom_lookup_timeout,
            args.loaded_egress_yaw_tolerance,
        ):
            return False
        total_yaw += segment
        if not _settle_without_motion(navigator, args.exit_settle):
            return False
        if not _bounded_reverse_by_odom(
            navigator,
            args.cmd_vel_topic,
            args.odom_frame,
            args.base_frame,
            reverse_distance,
            args.loaded_egress_linear_speed,
            args.loaded_egress_motion_timeout,
            args.odom_lookup_timeout,
            args.exit_heading_tolerance,
            args.exit_lateral_tolerance,
            f"loaded extreme egress round {round_index}",
        ):
            return False
        total_reverse += reverse_distance
        if not _settle_without_motion(navigator, args.exit_settle):
            return False
        risk = _read_loaded_current_risk(
            navigator, args.loaded_handoff_costmap_timeout
        )
        if risk is None or risk[0] != 0:
            navigator.get_logger().error(
                "LOADED_EGRESS_EXTREME_PAIR_OUTSIDE_OR_UNCERTAIN: "
                f"round={round_index} risk={risk}"
            )
            return False
        navigator.get_logger().info(
            "LOADED_EGRESS_EXTREME_PAIR_RESULT: "
            f"round={round_index} total_yaw={total_yaw:.3f} "
            f"total_reverse={total_reverse:.3f} risk={risk}"
        )
    if not _wait_for_loaded_handoff_clearance(
        navigator, args.loaded_handoff_costmap_timeout
    ):
        return False
    navigator.get_logger().warning(
        "LOADED_EGRESS_EXTREME_COMPLETE: "
        f"yaw={total_yaw:.3f} reverse={total_reverse:.3f}; "
        "zero-lethal direct Nav2 handoff may proceed"
    )
    return True


def _read_loaded_handoff_clearance(
    navigator: BasicNavigator,
    timeout: float,
) -> Optional[dict]:
    """Return synchronized global/local costmaps and loaded footprints."""
    if timeout <= 0.0:
        navigator.get_logger().error(
            "LOADED_HANDOFF_CLEARANCE_UNCERTAIN: invalid timeout"
        )
        return None

    messages = {
        "global_costmap": None,
        "global_footprint": None,
        "local_costmap": None,
        "local_footprint": None,
    }

    def capture(name):
        def callback(message):
            messages[name] = message
        return callback

    costmap_qos = QoSProfile(depth=1)
    costmap_qos.reliability = ReliabilityPolicy.RELIABLE
    costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscriptions = (
        navigator.create_subscription(
            Costmap, "/global_costmap/costmap_raw",
            capture("global_costmap"), costmap_qos
        ),
        navigator.create_subscription(
            PolygonStamped, "/global_costmap/published_footprint",
            capture("global_footprint"), 10
        ),
        navigator.create_subscription(
            Costmap, "/local_costmap/costmap_raw",
            capture("local_costmap"), costmap_qos
        ),
        navigator.create_subscription(
            PolygonStamped, "/local_costmap/published_footprint",
            capture("local_footprint"), 10
        ),
    )
    try:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            if all(message is not None for message in messages.values()):
                break
        if any(message is None for message in messages.values()):
            navigator.get_logger().error(
                "LOADED_HANDOFF_CLEARANCE_UNCERTAIN: global/local costmap "
                "or footprint unavailable"
            )
            return None
        if any(
            len(messages[name].polygon.points) < 3
            for name in ("global_footprint", "local_footprint")
        ):
            navigator.get_logger().error(
                "LOADED_HANDOFF_CLEARANCE_UNCERTAIN: invalid footprint"
            )
            return None
        return messages
    finally:
        for subscription in subscriptions:
            navigator.destroy_subscription(subscription)


def _footprint_center_and_offsets(footprint) -> tuple:
    points = list(footprint.polygon.points)
    center_x = sum(float(point.x) for point in points) / len(points)
    center_y = sum(float(point.y) for point in points) / len(points)
    return (
        center_x,
        center_y,
        [(float(point.x) - center_x, float(point.y) - center_y)
         for point in points],
    )


def _rotated_footprint(center_x, center_y, offsets, yaw_delta):
    cosine = math.cos(yaw_delta)
    sine = math.sin(yaw_delta)
    points = []
    for offset_x, offset_y in offsets:
        point = Point32()
        point.x = center_x + cosine * offset_x - sine * offset_y
        point.y = center_y + sine * offset_x + cosine * offset_y
        points.append(point)
    return points


def _initial_path_bearing(path, minimum_distance: float) -> Optional[float]:
    if minimum_distance <= 0.0 or len(path.poses) < 2:
        return None
    start = path.poses[0].pose.position
    for stamped_pose in path.poses[1:]:
        point = stamped_pose.pose.position
        displacement = math.hypot(
            point.x - start.x, point.y - start.y
        )
        if displacement >= minimum_distance:
            return math.atan2(point.y - start.y, point.x - start.x)
    return None


def _swept_clearance_analysis(
    costmap, footprint, yaw_delta: float, sweep_step: float
) -> Optional[dict]:
    if sweep_step <= 0.0 or not math.isfinite(yaw_delta):
        return None
    center_x, center_y, offsets = _footprint_center_and_offsets(footprint)
    sample_count = max(1, math.ceil(abs(yaw_delta) / sweep_step))
    worst = {"footprint_lethal": 0, "footprint_outside": 0}
    for index in range(sample_count + 1):
        sample_yaw = yaw_delta * index / sample_count
        points = _rotated_footprint(
            center_x, center_y, offsets, sample_yaw
        )
        analysis = analyze_costmap_start(
            costmap, points, (center_x, center_y, 0.0)
        )
        worst["footprint_lethal"] = max(
            worst["footprint_lethal"], analysis["footprint_lethal"]
        )
        worst["footprint_outside"] = max(
            worst["footprint_outside"], analysis["footprint_outside"]
        )
        if analysis["footprint_lethal"] or analysis["footprint_outside"]:
            worst["blocked_sample"] = index
            worst["sample_count"] = sample_count
            return worst
    worst["blocked_sample"] = None
    worst["sample_count"] = sample_count
    return worst


def _loaded_costmap_risk(messages: dict) -> tuple:
    """Return lexicographic outside/lethal/inscribed/start-cost risk."""
    results = []
    for prefix in ("global", "local"):
        footprint = messages[f"{prefix}_footprint"]
        points = list(footprint.polygon.points)
        center_x = sum(float(point.x) for point in points) / len(points)
        center_y = sum(float(point.y) for point in points) / len(points)
        results.append(
            analyze_costmap_start(
                messages[f"{prefix}_costmap"],
                points,
                (center_x, center_y, 0.0),
            )
        )
    return (
        max(result["footprint_outside"] for result in results),
        max(result["footprint_lethal"] for result in results),
        max(result["footprint_inscribed"] for result in results),
        max(
            -1 if result["start_cost"] is None else result["start_cost"]
            for result in results
        ),
    )


def _read_loaded_current_risk(
    navigator: BasicNavigator, timeout: float
) -> Optional[tuple]:
    messages = _read_loaded_handoff_clearance(navigator, timeout)
    if messages is None:
        return None
    risk = _loaded_costmap_risk(messages)
    navigator.get_logger().info(f"LOADED_EGRESS_CURRENT_RISK: {risk}")
    return risk


def _loaded_turn_segment_safe(
    navigator: BasicNavigator,
    args: argparse.Namespace,
    yaw_delta: float,
) -> Optional[bool]:
    messages = _read_loaded_handoff_clearance(
        navigator, args.loaded_handoff_costmap_timeout
    )
    if messages is None:
        return None
    for prefix in ("global", "local"):
        analysis = _swept_clearance_analysis(
            messages[f"{prefix}_costmap"],
            messages[f"{prefix}_footprint"],
            yaw_delta,
            args.loaded_handoff_sweep_step,
        )
        if analysis is None:
            return None
        if analysis["footprint_lethal"] or analysis["footprint_outside"]:
            return False
    return True


def _loaded_turn_segment_within_costmap(
    navigator: BasicNavigator,
    args: argparse.Namespace,
    yaw_delta: float,
) -> Optional[bool]:
    """For the extreme simulation experiment, reject outside only."""
    messages = _read_loaded_handoff_clearance(
        navigator, args.loaded_handoff_costmap_timeout
    )
    if messages is None:
        return None
    lethal = {}
    for prefix in ("global", "local"):
        analysis = _swept_clearance_analysis(
            messages[f"{prefix}_costmap"],
            messages[f"{prefix}_footprint"],
            yaw_delta,
            args.loaded_handoff_sweep_step,
        )
        if analysis is None:
            return None
        if analysis["footprint_outside"]:
            return False
        lethal[prefix] = analysis["footprint_lethal"]
    navigator.get_logger().warning(
        "LOADED_EGRESS_EXTREME_TURN_PREVIEW: lethal intentionally ignored "
        f"during bounded rotation global={lethal['global']} "
        f"local={lethal['local']}"
    )
    return True


def _loaded_dynamic_handoff_ready(
    navigator: BasicNavigator,
    args: argparse.Namespace,
    shipping_pose: PoseStamped,
    assessment_output: Optional[list] = None,
) -> Optional[bool]:
    messages = _read_loaded_handoff_clearance(
        navigator, args.loaded_handoff_costmap_timeout
    )
    if messages is None:
        return None
    paths = []
    result = _bounded_loaded_shipping_path_probe(
        navigator,
        shipping_pose,
        args.loaded_prealign_planner_id,
        args.loaded_prealign_path_probe_timeout,
        args.loaded_prealign_path_end_tolerance,
        paths,
    )
    if result is PathProbeResult.UNCERTAIN:
        return None
    if result is PathProbeResult.NO_PATH:
        navigator.get_logger().info(
            "LOADED_DYNAMIC_HANDOFF_BLOCKED: no current shipping path"
        )
        return False
    path_bearing = _initial_path_bearing(
        paths[0], args.loaded_handoff_path_lookahead
    )
    if path_bearing is None:
        navigator.get_logger().error(
            "LOADED_DYNAMIC_HANDOFF_UNCERTAIN: path bearing unavailable"
        )
        return None
    current_transform = _lookup_fresh_transform(
        navigator,
        args.frame_id,
        args.base_frame,
        args.shipping_pose_lookup_timeout,
    )
    if current_transform is None:
        navigator.get_logger().error(
            "LOADED_DYNAMIC_HANDOFF_UNCERTAIN: current pose unavailable"
        )
        return None
    current_yaw = _yaw_from_rotation(current_transform.transform.rotation)
    yaw_delta = _normalize_angle(path_bearing - current_yaw)
    analyses = {}
    for prefix in ("global", "local"):
        analysis = _swept_clearance_analysis(
            messages[f"{prefix}_costmap"],
            messages[f"{prefix}_footprint"],
            yaw_delta,
            args.loaded_handoff_sweep_step,
        )
        if analysis is None:
            return None
        analyses[prefix] = analysis
    blocked = any(
        analysis["footprint_lethal"] or analysis["footprint_outside"]
        for analysis in analyses.values()
    )
    navigator.get_logger().info(
        "LOADED_DYNAMIC_HANDOFF_RESULT: "
        f"yaw_delta={yaw_delta:.3f} "
        f"global_lethal={analyses['global']['footprint_lethal']} "
        f"global_outside={analyses['global']['footprint_outside']} "
        f"local_lethal={analyses['local']['footprint_lethal']} "
        f"local_outside={analyses['local']['footprint_outside']}"
    )
    if assessment_output is not None:
        assessment_output.append(
            {
                "yaw_delta": yaw_delta,
                "blocked": blocked,
                "analyses": analyses,
            }
        )
    return not blocked


def _bounded_loaded_prehandoff_rotation(
    navigator: BasicNavigator,
    args: argparse.Namespace,
    shipping_pose: PoseStamped,
    localization_monitor: _LoadedLocalizationMonitor,
) -> bool:
    """Consume a safe large initial path turn in stopped checked segments."""
    if (
        args.loaded_handoff_max_nav_yaw < 0.0
        or args.loaded_handoff_max_turn_segment <= 0.0
        or args.loaded_handoff_max_total_turn <= 0.0
        or args.loaded_handoff_max_turn_rounds <= 0
    ):
        navigator.get_logger().error(
            "LOADED_PREHANDOFF_ROTATION_UNCERTAIN: invalid limits"
        )
        return False
    total_turn = 0.0
    for round_index in range(1, args.loaded_handoff_max_turn_rounds + 1):
        assessments = []
        readiness = _loaded_dynamic_handoff_ready(
            navigator, args, shipping_pose, assessments
        )
        if readiness is None or not assessments:
            navigator.get_logger().error(
                "LOADED_PREHANDOFF_ROTATION_UNCERTAIN: assessment failed"
            )
            return False
        if not readiness:
            navigator.get_logger().error(
                "LOADED_PREHANDOFF_ROTATION_BLOCKED: swept clearance lost"
            )
            return False
        yaw_delta = assessments[0]["yaw_delta"]
        if abs(yaw_delta) <= args.loaded_handoff_max_nav_yaw:
            navigator.get_logger().info(
                "LOADED_PREHANDOFF_ROTATION_READY: "
                f"remaining_yaw={yaw_delta:.3f} "
                f"total_turn={total_turn:.3f} rounds={round_index - 1}"
            )
            return True
        segment = math.copysign(
            min(abs(yaw_delta), args.loaded_handoff_max_turn_segment),
            yaw_delta,
        )
        if total_turn + abs(segment) > (
            args.loaded_handoff_max_total_turn + 1e-9
        ):
            navigator.get_logger().error(
                "LOADED_PREHANDOFF_ROTATION_LIMIT_REJECTED: "
                f"total={total_turn + abs(segment):.3f}"
            )
            return False
        navigator.get_logger().info(
            "LOADED_PREHANDOFF_ROTATION_SEGMENT: "
            f"round={round_index} target={segment:.3f} "
            f"remaining_before={yaw_delta:.3f}"
        )
        if not _bounded_rotate_by_odom(
            navigator,
            args.cmd_vel_topic,
            args.odom_frame,
            args.base_frame,
            segment,
            args.loaded_egress_angular_speed,
            args.loaded_egress_motion_timeout,
            args.odom_lookup_timeout,
            args.loaded_egress_yaw_tolerance,
        ):
            return False
        total_turn += abs(segment)
        if not _settle_without_motion(navigator, args.exit_settle):
            return False
        if not _wait_for_loaded_localization_stability(
            navigator,
            localization_monitor,
            args.loaded_localization_samples,
            args.loaded_localization_sample_interval,
            args.shipping_pose_lookup_timeout,
        ):
            navigator.get_logger().error(
                "LOADED_PREHANDOFF_ROTATION_LOCALIZATION_REJECTED"
            )
            return False
    navigator.get_logger().error(
        "LOADED_PREHANDOFF_ROTATION_ROUNDS_EXHAUSTED"
    )
    return False


def _wait_for_loaded_handoff_clearance(
    navigator: BasicNavigator,
    timeout: float,
) -> bool:
    """Require a received footprint with no lethal or outside costmap cells."""
    messages = _read_loaded_handoff_clearance(navigator, timeout)
    if messages is None:
        return False
    results = {}
    for prefix in ("global", "local"):
        footprint = messages[f"{prefix}_footprint"]
        points = list(footprint.polygon.points)
        center_x = sum(float(point.x) for point in points) / len(points)
        center_y = sum(float(point.y) for point in points) / len(points)
        results[prefix] = analyze_costmap_start(
            messages[f"{prefix}_costmap"],
            points,
            (center_x, center_y, 0.0),
        )
    if any(
        result["footprint_lethal"] or result["footprint_outside"]
        for result in results.values()
    ):
        navigator.get_logger().error("LOADED_HANDOFF_CLEARANCE_BLOCKED")
        return False
    navigator.get_logger().info(
        "LOADED_HANDOFF_CLEARANCE_READY: global/local lethal=0 outside=0"
    )
    return True


def _prealign_loaded_shipping_bearing(
    navigator: BasicNavigator,
    args: argparse.Namespace,
    localization_monitor: _LoadedLocalizationMonitor,
) -> bool:
    """Require one usable path after the bounded reverse S-curve."""
    shipping_pose = _pose(
        navigator,
        args.frame_id,
        args.shipping_x,
        args.shipping_y,
        args.shipping_yaw,
    )
    probe_result = _bounded_loaded_shipping_path_probe(
        navigator,
        shipping_pose,
        args.loaded_prealign_planner_id,
        args.loaded_prealign_path_probe_timeout,
        args.loaded_prealign_path_end_tolerance,
    )
    if probe_result is PathProbeResult.PATH_READY:
        navigator.get_logger().info(
            "LOADED_SHIPPING_DIRECT_NAV2_HANDOFF: usable path ready; "
            "no direct prealignment arc"
        )
        return True
    if probe_result is PathProbeResult.NO_PATH:
        navigator.get_logger().error(
            "LOADED_SHIPPING_REVERSE_S_NO_PATH: no further direct motion"
        )
        return False
    navigator.get_logger().error(
        "LOADED_SHIPPING_DIRECT_NAV2_UNCERTAIN: path probe unavailable"
    )
    return False


def _legacy_prealign_loaded_shipping_bearing(
    navigator: BasicNavigator,
    args: argparse.Namespace,
    localization_monitor: _LoadedLocalizationMonitor,
) -> bool:
    """Retained reference implementation; no longer called by the mission."""
    max_segment = args.loaded_prealign_max_segment_yaw
    max_total = args.loaded_prealign_max_total_yaw
    tolerance = args.loaded_prealign_bearing_tolerance
    max_confirmable_jump = (
        args.loaded_prealign_max_confirmable_position_jump
    )
    max_confirmations = (
        args.loaded_prealign_max_localization_confirmations
    )
    if (
        max_segment <= 0.0
        or max_total <= 0.0
        or tolerance < 0.0
        or max_confirmable_jump
        <= args.loaded_localization_max_position_jump
        or max_confirmations < 0
        or args.loaded_prealign_path_probe_timeout <= 0.0
        or args.loaded_prealign_path_end_tolerance < 0.0
        or args.loaded_prealign_path_handoff_max_bearing < tolerance
        or args.loaded_prealign_arc_max_distance <= 0.0
        or args.loaded_prealign_arc_linear_speed <= 0.0
        or not args.loaded_prealign_planner_id
    ):
        navigator.get_logger().error(
            "invalid loaded shipping prealignment parameters"
        )
        return False

    requested_total = 0.0
    localization_confirmations = 0
    while requested_total <= max_total:
        transform = _lookup_fresh_transform(
            navigator,
            args.frame_id,
            args.base_frame,
            args.shipping_pose_lookup_timeout,
        )
        if transform is None:
            navigator.get_logger().error(
                "loaded shipping prealignment rejected: map pose unavailable"
            )
            return False
        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y
        current_yaw = _yaw_from_rotation(transform.transform.rotation)
        bearing = math.atan2(
            args.shipping_y - current_y,
            args.shipping_x - current_x,
        )
        error = _normalize_angle(bearing - current_yaw)
        shipping_pose = _pose(
            navigator,
            args.frame_id,
            args.shipping_x,
            args.shipping_y,
            args.shipping_yaw,
        )
        probe_result = _bounded_loaded_shipping_path_probe(
            navigator,
            shipping_pose,
            args.loaded_prealign_planner_id,
            args.loaded_prealign_path_probe_timeout,
            args.loaded_prealign_path_end_tolerance,
        )
        if (
            probe_result is PathProbeResult.PATH_READY
            and abs(error)
            <= args.loaded_prealign_path_handoff_max_bearing
        ):
            navigator.get_logger().info(
                "LOADED_SHIPPING_PREALIGN_COMPLETE: planner handoff "
                f"remaining_bearing_error={error:.3f} "
                f"requested_total={requested_total:.3f}"
            )
            return True
        if probe_result is PathProbeResult.PATH_READY:
            navigator.get_logger().info(
                "loaded shipping path ready but bearing remains outside "
                "bounded handoff: "
                f"error={error:.3f} max="
                f"{args.loaded_prealign_path_handoff_max_bearing:.3f}"
            )
        if probe_result is PathProbeResult.UNCERTAIN:
            navigator.get_logger().error(
                "loaded shipping prealignment stopped: path probe uncertain"
            )
            return False
        if (
            probe_result is PathProbeResult.NO_PATH
            and args.loaded_boundary_pair_diagnostic
        ):
            navigator.get_logger().warning(
                "LOADED_BOUNDARY_PAIR_DIAGNOSTIC_TRIGGERED: "
                "auto-start NO_PATH; no further prealignment motion"
            )
            result = _run_loaded_boundary_pair_diagnostic(args)
            navigator.get_logger().warning(
                "LOADED_BOUNDARY_PAIR_DIAGNOSTIC_COMPLETE: "
                f"exit={result}; mission stops before handoff"
            )
            return False
        if (
            probe_result is PathProbeResult.NO_PATH
            and abs(error) <= tolerance
        ):
            navigator.get_logger().error(
                "LOADED_SHIPPING_NO_PATH_AT_ALIGNED_HEADING: "
                f"remaining_bearing_error={error:.3f} "
                f"requested_total={requested_total:.3f}"
            )
            return False

        if error >= 0.0:
            navigator.get_logger().error(
                "loaded shipping prealignment rejected: "
                "left arc is not authorized"
            )
            return False
        segment = min(abs(error), max_segment)
        if requested_total + segment > max_total:
            navigator.get_logger().error(
                "loaded shipping prealignment rejected: total yaw bound "
                f"would be exceeded requested={requested_total:.3f} "
                f"next={segment:.3f} max={max_total:.3f}"
            )
            return False
        navigator.get_logger().info(
            "loaded shipping prealignment segment: "
            f"bearing={bearing:.3f} current_yaw={current_yaw:.3f} "
            f"error={error:.3f} right_arc_yaw={segment:.3f}"
        )
        if not _bounded_forward_right_arc_by_odom(
            navigator,
            args.cmd_vel_topic,
            args.odom_frame,
            args.base_frame,
            args.loaded_prealign_arc_max_distance,
            segment,
            args.loaded_prealign_arc_linear_speed,
            args.loaded_egress_angular_speed,
            args.loaded_egress_motion_timeout,
            args.odom_lookup_timeout,
            args.loaded_egress_yaw_tolerance,
        ):
            return False
        requested_total += segment
        if not _settle_without_motion(navigator, args.exit_settle):
            return False
        if not _wait_for_loaded_localization_stability(
            navigator,
            localization_monitor,
            args.loaded_localization_samples,
            args.loaded_localization_sample_interval,
            args.shipping_pose_lookup_timeout,
        ):
            confirmable = _prealign_localization_reconfirmation_allowed(
                localization_monitor,
                args.loaded_localization_max_position_jump,
                args.loaded_localization_max_yaw_jump,
                max_confirmable_jump,
            )
            if (
                not confirmable
                or localization_confirmations >= max_confirmations
            ):
                navigator.get_logger().error(
                    "loaded shipping prealignment stopped: "
                    "localization unstable"
                )
                return False
            localization_confirmations += 1
            navigator.get_logger().warning(
                "LOADED_PREALIGN_LOCALIZATION_RECONFIRM: "
                f"translation={localization_monitor.last_position_jump:.3f} "
                f"yaw={localization_monitor.last_yaw_jump:.3f} "
                f"attempt={localization_confirmations}/{max_confirmations}"
            )
            if not _wait_for_loaded_localization_stability(
                navigator,
                localization_monitor,
                args.loaded_localization_samples,
                args.loaded_localization_sample_interval,
                args.shipping_pose_lookup_timeout,
            ):
                navigator.get_logger().error(
                    "loaded shipping prealignment stopped: borderline "
                    "localization correction did not settle"
                )
                return False
            navigator.get_logger().info(
                "LOADED_PREALIGN_LOCALIZATION_RECONFIRMED"
            )
    navigator.get_logger().error(
        "loaded shipping prealignment exhausted total yaw bound"
    )
    return False


def _run_loaded_boundary_pair_diagnostic(
    args: argparse.Namespace,
) -> int:
    """Run the verified planning-only pair at the current stopped boundary."""
    from nav2_apps.explicit_start_pair_probe import ExplicitStartPairProbe

    pair_args = argparse.Namespace(
        frame_id=args.frame_id,
        base_frame=args.base_frame,
        action_name="/compute_path_to_pose",
        costmap_topic="/global_costmap/costmap_raw",
        cmd_vel_topic=args.cmd_vel_topic,
        planner_id=args.loaded_prealign_planner_id,
        goal_x=args.shipping_x,
        goal_y=args.shipping_y,
        goal_yaw=args.shipping_yaw,
        tf_timeout=args.shipping_pose_lookup_timeout,
        costmap_timeout=args.loaded_prealign_path_probe_timeout,
        action_timeout=args.loaded_prealign_path_probe_timeout,
        between_wait=0.5,
        max_tf_position_delta=0.02,
        max_tf_yaw_delta=0.02,
        zero_twist_tolerance=1e-4,
    )
    node = ExplicitStartPairProbe(pair_args)
    try:
        return node.run()
    finally:
        node.destroy_node()


def _settle_without_motion(
    navigator: BasicNavigator, wait_seconds: float
) -> bool:
    if wait_seconds < 0.0:
        navigator.get_logger().error("exit settle must be non-negative")
        return False
    deadline = time.monotonic() + wait_seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    return True


def _request_shelf_transform(
    navigator: BasicNavigator,
    service_name: str,
    cart_frame: str,
    base_frame: str,
    timeout: float,
):
    """Request a fresh detection and return its base-frame transform."""
    import tf2_ros

    if timeout <= 0.0:
        navigator.get_logger().error("clearance timeout must be positive")
        return None
    service_type = _load_shelf_service_type()
    client = navigator.create_client(service_type, service_name)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, navigator, spin_thread=False)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                navigator.get_logger().error(
                    "shelf clearance detection service was unavailable"
                )
                return None
        request = service_type.Request()
        request.attach_to_shelf = False
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                navigator.get_logger().error(
                    "shelf clearance detection timed out"
                )
                return None
            rclpy.spin_once(navigator, timeout_sec=0.1)
        if (
            not future.done()
            or future.result() is None
            or not future.result().complete
        ):
            navigator.get_logger().error(
                "shelf clearance detection did not complete"
            )
            return None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = buffer.lookup_transform(
                    base_frame, cart_frame, rclpy.time.Time()
                )
                navigator.get_logger().info(
                    "fresh shelf clearance transform acquired: "
                    f"x={transform.transform.translation.x:.3f} "
                    f"y={transform.transform.translation.y:.3f}"
                )
                return transform
            except tf2_ros.TransformException:
                rclpy.spin_once(navigator, timeout_sec=0.1)
    finally:
        del listener
    navigator.get_logger().error(
        "shelf detection completed but fresh clearance TF was unavailable"
    )
    return None


def _clearance_passes(transform, minimum_x: float) -> bool:
    return (
        transform is not None
        and minimum_x > 0.0
        and transform.transform.translation.x >= minimum_x
    )


def _apply_loaded_footprint_verified(
    navigator: BasicNavigator,
    desired: str,
    timeout: float,
    tolerance: float,
) -> bool:
    """Apply both costmap footprints through the compensating transaction."""
    try:
        from nav2_apps.footprint_transaction import apply_loaded_footprint

        transaction = apply_loaded_footprint(
            navigator,
            desired,
            timeout,
            tolerance,
        )
    except (RuntimeError, TypeError, ValueError) as error:
        navigator.get_logger().error(
            f"loaded footprint transaction error: {error}"
        )
        return False
    if not transaction.success:
        navigator.get_logger().error(
            "loaded footprint transaction failed: "
            f"{transaction.reason}; "
            f"rollback_verified={transaction.rollback_verified}"
        )
        return False
    navigator.get_logger().info(
        "loaded_footprint_verified on global and local costmaps"
    )
    return True


def _apply_unloaded_footprint_verified(
    navigator: BasicNavigator,
    desired: str,
    timeout: float,
    tolerance: float,
) -> bool:
    """Restore both footprints, compensating to loaded state on failure."""
    try:
        from nav2_apps.footprint_transaction import apply_footprint

        transaction = apply_footprint(
            navigator,
            desired,
            timeout,
            tolerance,
        )
    except (RuntimeError, TypeError, ValueError) as error:
        navigator.get_logger().error(
            f"unloaded footprint transaction error: {error}"
        )
        return False
    if not transaction.success:
        navigator.get_logger().error(
            "unloaded footprint transaction failed: "
            f"{transaction.reason}; "
            f"loaded_rollback_verified={transaction.rollback_verified}"
        )
        return False
    navigator.get_logger().info(
        "unloaded_footprint_verified on global and local costmaps"
    )
    return True


def _navigate_to_shipping(
    navigator: BasicNavigator,
    shipping_pose: PoseStamped,
    timeout: float,
    base_frame: str,
    position_tolerance: float,
    yaw_tolerance: float,
    max_yaw_correction: float,
    alignment_timeout: float,
    alignment_settle: float,
    lookup_timeout: float,
    correction_ratio: float = 0.5,
    max_correction_rounds: int = 3,
    localization_monitor: Optional[_LoadedLocalizationMonitor] = None,
) -> ExitCode:
    """Navigate once to shipping and expose a bounded terminal result."""
    if timeout <= 0.0:
        navigator.get_logger().error("shipping timeout must be positive")
        return ExitCode.UNKNOWN
    navigator.get_logger().info(
        "Navigating to shipping_position: "
        f"{shipping_pose.pose.position.x} "
        f"{shipping_pose.pose.position.y}..."
    )
    if not navigator.goToPose(shipping_pose):
        navigator.get_logger().error("shipping_position goal was rejected")
        return ExitCode.GOAL_REJECTED

    deadline = time.monotonic() + timeout
    while not navigator.isTaskComplete():
        if time.monotonic() >= deadline:
            navigator.cancelTask()
            navigator.get_logger().error(
                "shipping_position navigation timed out; cancel requested"
            )
            return ExitCode.CANCELED
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if localization_monitor is not None:
            localization_stable = localization_monitor.sample()
            if localization_stable is False:
                navigator.cancelTask()
                navigator.get_logger().error(
                    "shipping canceled: loaded localization jump detected "
                    f"translation="
                    f"{localization_monitor.last_position_jump:.3f} "
                    f"yaw={localization_monitor.last_yaw_jump:.3f}"
                )
                return ExitCode.CANCELED

    result = navigator.getResult()
    exit_code = classify_task_result(result, TaskResult)
    if exit_code != ExitCode.SUCCEEDED:
        navigator.get_logger().error(
            f"shipping_position goal did not succeed: {result}"
        )
        return exit_code
    navigator.get_logger().info("shipping_position Nav2 goal succeeded")
    if not _accept_or_align_shipping_pose(
        navigator,
        shipping_pose,
        base_frame,
        position_tolerance,
        yaw_tolerance,
        max_yaw_correction,
        alignment_timeout,
        alignment_settle,
        lookup_timeout,
        correction_ratio,
        max_correction_rounds,
    ):
        navigator.get_logger().error(
            "Slice 3A stopped at SHIPPING_ALIGNMENT_PENDING"
        )
        return ExitCode.UNKNOWN
    navigator.get_logger().info(
        "shipping pose accepted; Slice 3A stopped at AT_SHIPPING"
    )
    return ExitCode.SUCCEEDED


def _align_at_shipping(
    navigator: BasicNavigator,
    shipping_pose: PoseStamped,
    base_frame: str,
    position_tolerance: float,
    yaw_tolerance: float,
    max_yaw_correction: float,
    alignment_timeout: float,
    alignment_settle: float,
    lookup_timeout: float,
    correction_ratio: float,
    max_correction_rounds: int,
) -> ExitCode:
    """Align an already stopped shipping pose without sending a Nav2 goal."""
    navigator.get_logger().info(
        "Shipping alignment-only mode: no goToPose goal will be sent"
    )
    if not _accept_or_align_shipping_pose(
        navigator,
        shipping_pose,
        base_frame,
        position_tolerance,
        yaw_tolerance,
        max_yaw_correction,
        alignment_timeout,
        alignment_settle,
        lookup_timeout,
        correction_ratio,
        max_correction_rounds,
    ):
        navigator.get_logger().error(
            "Alignment-only slice stopped at SHIPPING_ALIGNMENT_PENDING"
        )
        return ExitCode.UNKNOWN
    navigator.get_logger().info(
        "shipping pose accepted; alignment-only slice stopped at "
        "AT_SHIPPING"
    )
    return ExitCode.SUCCEEDED


def _navigate_to_init(
    navigator: BasicNavigator,
    init_pose: PoseStamped,
    timeout: float,
) -> ExitCode:
    """Navigate once to init_position and expose a bounded terminal result."""
    if timeout <= 0.0:
        navigator.get_logger().error("return timeout must be positive")
        return ExitCode.UNKNOWN
    navigator.get_logger().info(
        "Navigating to init_position: "
        f"{init_pose.pose.position.x} {init_pose.pose.position.y}..."
    )
    if not navigator.goToPose(init_pose):
        navigator.get_logger().error("init_position goal was rejected")
        return ExitCode.GOAL_REJECTED

    deadline = time.monotonic() + timeout
    while not navigator.isTaskComplete():
        if time.monotonic() >= deadline:
            navigator.cancelTask()
            navigator.get_logger().error(
                "init_position navigation timed out; cancel requested"
            )
            return ExitCode.CANCELED
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    exit_code = classify_task_result(result, TaskResult)
    if exit_code != ExitCode.SUCCEEDED:
        navigator.get_logger().error(
            f"init_position goal did not succeed: {result}"
        )
        return exit_code
    navigator.get_logger().info(
        "init_position goal succeeded; Slice 3C stopped at AT_INIT"
    )
    return ExitCode.SUCCEEDED


def _exit_restore_integrated(
    navigator: BasicNavigator,
    args: argparse.Namespace,
) -> bool:
    """Exit the shelf, refine clearance once if needed, then restore shape."""
    if not _bounded_reverse_by_odom(
        navigator,
        args.cmd_vel_topic,
        args.odom_frame,
        args.base_frame,
        args.exit_distance,
        args.exit_speed,
        args.exit_timeout,
        args.odom_lookup_timeout,
        args.exit_heading_tolerance,
        args.exit_lateral_tolerance,
    ):
        return False
    if not _settle_without_motion(navigator, args.exit_settle):
        return False

    transform = _request_shelf_transform(
        navigator,
        args.shelf_service,
        args.cart_frame,
        args.base_frame,
        args.clearance_timeout,
    )
    if transform is None:
        navigator.get_logger().error(
            "integrated mission stopped at EXIT_ACCEPTANCE_PENDING: fresh "
            "shelf clearance is unavailable; loaded footprint retained"
        )
        return False
    if not _clearance_passes(transform, args.clearance_x):
        navigator.get_logger().warning(
            "main exit clearance is below the acceptance threshold; "
            "running one bounded clearance refinement"
        )
        if not _bounded_reverse_by_odom(
            navigator,
            args.cmd_vel_topic,
            args.odom_frame,
            args.base_frame,
            args.clearance_refine_distance,
            args.clearance_refine_speed,
            args.clearance_refine_motion_timeout,
            args.odom_lookup_timeout,
            args.exit_heading_tolerance,
            args.exit_lateral_tolerance,
        ):
            return False
        if not _settle_without_motion(navigator, args.exit_settle):
            return False
        transform = _request_shelf_transform(
            navigator,
            args.shelf_service,
            args.cart_frame,
            args.base_frame,
            args.clearance_timeout,
        )
    if not _clearance_passes(transform, args.clearance_x):
        observed = (
            "unavailable"
            if transform is None
            else f"{transform.transform.translation.x:.3f}"
        )
        navigator.get_logger().error(
            "integrated mission stopped at EXIT_ACCEPTANCE_PENDING: fresh "
            f"cart_frame.x={observed}, required>={args.clearance_x:.3f}; "
            "loaded footprint retained"
        )
        return False

    navigator.get_logger().info("CLEAR_OF_SHELF")
    return _apply_unloaded_footprint_verified(
        navigator,
        args.unloaded_footprint,
        args.footprint_timeout,
        args.footprint_edge_tolerance,
    )


def _run_integrated_mission(
    navigator: BasicNavigator,
    args: argparse.Namespace,
) -> ExitCode:
    """Run the validated post-loading route as one fail-closed mission."""
    navigator.get_logger().info(
        "integrated mission: attach -> shipping -> lower -> exit -> return"
    )
    if not _request_stepwise_attach(
        navigator, args.shelf_service, args.attach_timeout
    ):
        return ExitCode.UNKNOWN
    if not _settle_without_motion(navigator, args.elevator_wait):
        return ExitCode.UNKNOWN
    if not _apply_loaded_footprint_verified(
        navigator,
        args.loaded_footprint,
        args.footprint_timeout,
        args.footprint_edge_tolerance,
    ):
        return ExitCode.UNKNOWN
    shipping_pose = _pose(
        navigator,
        args.frame_id,
        args.shipping_x,
        args.shipping_y,
        args.shipping_yaw,
    )
    if not args.loaded_egress_extreme_left_90_experiment:
        navigator.get_logger().info(
            "DIRECT_NAV2_HANDOFF_AFTER_LIFT: loaded footprint verified; "
            "custom egress, prealignment, and localization jump gate skipped"
        )
        shipping_result = _navigate_to_shipping(
            navigator,
            shipping_pose,
            args.shipping_timeout,
            args.base_frame,
            args.shipping_position_tolerance,
            args.shipping_yaw_tolerance,
            args.shipping_max_yaw_correction,
            args.shipping_alignment_timeout,
            args.shipping_alignment_settle,
            args.shipping_pose_lookup_timeout,
            args.shipping_yaw_correction_ratio,
            args.shipping_yaw_correction_rounds,
        )
    else:
        navigator.get_logger().warning(
            "EXTREME_LOADED_EGRESS_DIAGNOSTIC_SELECTED: default direct "
            "Nav2 handoff is bypassed"
        )
        frozen_map_odom = _freeze_map_to_odom(
            navigator,
            args.frame_id,
            args.odom_frame,
            args.shipping_pose_lookup_timeout,
            args.loaded_map_odom_freeze_lifecycle_timeout,
        )
        if frozen_map_odom is None:
            return ExitCode.UNKNOWN
        freeze_restore_ok = True
        try:
            egress_ready = _loaded_egress_extreme_left_90_experiment(
                navigator, args
            )
        finally:
            freeze_restore_ok = _restore_amcl_after_freeze(
                navigator,
                frozen_map_odom,
                args.loaded_map_odom_freeze_lifecycle_timeout,
            )
        if not freeze_restore_ok or not egress_ready:
            return ExitCode.UNKNOWN
        shipping_result = _navigate_to_shipping(
            navigator,
            shipping_pose,
            args.shipping_timeout,
            args.base_frame,
            args.shipping_position_tolerance,
            args.shipping_yaw_tolerance,
            args.shipping_max_yaw_correction,
            args.shipping_alignment_timeout,
            args.shipping_alignment_settle,
            args.shipping_pose_lookup_timeout,
            args.shipping_yaw_correction_ratio,
            args.shipping_yaw_correction_rounds,
        )
    if shipping_result != ExitCode.SUCCEEDED:
        return shipping_result

    if args.stop_at_shipping:
        navigator.get_logger().info(
            "INTEGRATED_STOP_AT_SHIPPING: shipping accepted; lower, exit, "
            "and return skipped"
        )
        return ExitCode.SUCCEEDED

    if not _bounded_forward_by_odom(
        navigator,
        args.cmd_vel_topic,
        args.odom_frame,
        args.base_frame,
        args.shipping_refine_distance,
        args.shipping_refine_speed,
        args.shipping_refine_timeout,
        args.odom_lookup_timeout,
        args.exit_heading_tolerance,
        args.exit_lateral_tolerance,
    ):
        return ExitCode.UNKNOWN
    if not _settle_without_motion(navigator, args.exit_settle):
        return ExitCode.UNKNOWN
    navigator.get_logger().info("SHIPPING_PLACEMENT_REFINED")

    if not _publish_elevator_down_and_wait(
        navigator,
        args.elevator_down_topic,
        args.elevator_down_count,
        args.elevator_down_interval,
        args.elevator_down_wait,
    ):
        return ExitCode.UNKNOWN
    if not _exit_restore_integrated(navigator, args):
        return ExitCode.UNKNOWN

    init_pose = _pose(
        navigator,
        args.frame_id,
        args.return_x,
        args.return_y,
        args.return_yaw,
    )
    return _navigate_to_init(navigator, init_pose, args.return_timeout)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_application_args(argv)
    try:
        initial_pose = optional_initial_pose(args)
    except ValueError as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return int(ExitCode.UNKNOWN)

    rclpy.init(args=argv)
    navigator = BasicNavigator()
    try:
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
        integrated_mode = not any(operation_modes)
        if sum(bool(mode) for mode in operation_modes) > 1:
            navigator.get_logger().error(
                "detection, attach, footprint, shipping, alignment, lower, "
                "exit, clearance-refine, re-lift, refine, and return "
                "modes are mutually exclusive"
            )
            return int(ExitCode.UNKNOWN)

        if args.shipping_relift_only:
            if not (
                args.confirm_at_shipping
                and args.confirm_shelf_lowered
                and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "shipping-relift-only requires explicit at-shipping, "
                    "shelf-lowered, and stopped-state confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "shipping-relift-only does not allow an initial pose "
                    "override"
                )
                return int(ExitCode.UNKNOWN)
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            if not _publish_elevator_up_and_wait(
                navigator,
                args.elevator_up_topic,
                args.elevator_up_count,
                args.elevator_up_interval,
                args.elevator_up_wait,
            ):
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().warning(
                "shipping re-lift stopped at lift_acceptance_pending; no "
                "forward command was published"
            )
            return int(ExitCode.UNKNOWN)

        if args.shipping_forward_refine_only:
            if not (
                args.confirm_at_shipping
                and args.confirm_lift_accepted
                and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "shipping-forward-refine-only requires explicit "
                    "at-shipping, re-lift-accepted, and stopped-state "
                    "confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "shipping-forward-refine-only does not allow an initial "
                    "pose override"
                )
                return int(ExitCode.UNKNOWN)
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            if not _bounded_forward_by_odom(
                navigator,
                args.cmd_vel_topic,
                args.odom_frame,
                args.base_frame,
                args.shipping_refine_distance,
                args.shipping_refine_speed,
                args.shipping_refine_timeout,
                args.odom_lookup_timeout,
                args.exit_heading_tolerance,
                args.exit_lateral_tolerance,
            ):
                navigator.get_logger().error(
                    "shipping forward refinement stopped at "
                    "PLACEMENT_ACCEPTANCE_PENDING; loaded footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            if not _settle_without_motion(navigator, args.exit_settle):
                navigator.get_logger().error(
                    "shipping forward refinement settle failed; loaded "
                    "footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().warning(
                "shipping forward refinement stopped at "
                "PLACEMENT_ACCEPTANCE_PENDING; external boundary visual "
                "acceptance is required before lowering"
            )
            return int(ExitCode.UNKNOWN)

        if args.lower_only:
            if not (
                args.confirm_at_shipping
                and args.confirm_lift_accepted
                and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "lower-only requires explicit at-shipping, lifted, "
                    "and stopped-state confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "lower-only does not allow an initial pose override"
                )
                return int(ExitCode.UNKNOWN)
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            if not _publish_elevator_down_and_wait(
                navigator,
                args.elevator_down_topic,
                args.elevator_down_count,
                args.elevator_down_interval,
                args.elevator_down_wait,
            ):
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().warning(
                "Slice 3B1 stopped at lower_acceptance_pending; no shelf "
                "exit command was published"
            )
            return int(ExitCode.UNKNOWN)

        if args.exit_restore_only:
            if not (
                args.confirm_at_shipping
                and args.confirm_shelf_lowered
                and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "exit-restore-only requires explicit at-shipping, "
                    "shelf-lowered, and stopped-state confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "exit-restore-only does not allow an initial pose "
                    "override"
                )
                return int(ExitCode.UNKNOWN)
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            if not _bounded_reverse_by_odom(
                navigator,
                args.cmd_vel_topic,
                args.odom_frame,
                args.base_frame,
                args.exit_distance,
                args.exit_speed,
                args.exit_timeout,
                args.odom_lookup_timeout,
                args.exit_heading_tolerance,
                args.exit_lateral_tolerance,
            ):
                navigator.get_logger().error(
                    "Slice 3B2 stopped at EXIT_ACCEPTANCE_PENDING; loaded "
                    "footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            if not _settle_without_motion(navigator, args.exit_settle):
                navigator.get_logger().error(
                    "Slice 3B2 exit settle failed; loaded footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            transform = _request_shelf_transform(
                navigator,
                args.shelf_service,
                args.cart_frame,
                args.base_frame,
                args.clearance_timeout,
            )
            if not _clearance_passes(transform, args.clearance_x):
                observed = (
                    "unavailable"
                    if transform is None
                    else f"{transform.transform.translation.x:.3f}"
                )
                navigator.get_logger().error(
                    "Slice 3B2 stopped at EXIT_ACCEPTANCE_PENDING: fresh "
                    f"cart_frame.x={observed}, required>="
                    f"{args.clearance_x:.3f}; loaded footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().info(
                "CLEAR_OF_SHELF verified after odom exit, stop/settle, "
                "and fresh shelf geometry"
            )
            if not _apply_unloaded_footprint_verified(
                navigator,
                args.unloaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().info(
                "Slice 3B2 stopped at UNLOADED_FOOTPRINT_VERIFIED before "
                "return navigation"
            )
            return int(ExitCode.SUCCEEDED)

        if args.exit_clearance_refine_only:
            if not (
                args.confirm_at_shipping
                and args.confirm_shelf_lowered
                and args.confirm_robot_stopped
                and args.confirm_exit_distance_complete
            ):
                navigator.get_logger().error(
                    "exit-clearance-refine-only requires explicit "
                    "at-shipping, shelf-lowered, stopped-state, and "
                    "main-exit-distance-complete confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "exit-clearance-refine-only does not allow an initial "
                    "pose override"
                )
                return int(ExitCode.UNKNOWN)
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            if not _bounded_reverse_by_odom(
                navigator,
                args.cmd_vel_topic,
                args.odom_frame,
                args.base_frame,
                args.clearance_refine_distance,
                args.clearance_refine_speed,
                args.clearance_refine_motion_timeout,
                args.odom_lookup_timeout,
                args.exit_heading_tolerance,
                args.exit_lateral_tolerance,
            ):
                navigator.get_logger().error(
                    "clearance refinement stopped at "
                    "EXIT_ACCEPTANCE_PENDING; loaded footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            if not _settle_without_motion(navigator, args.exit_settle):
                navigator.get_logger().error(
                    "clearance refinement settle failed; loaded footprint "
                    "retained"
                )
                return int(ExitCode.UNKNOWN)
            transform = _request_shelf_transform(
                navigator,
                args.shelf_service,
                args.cart_frame,
                args.base_frame,
                args.clearance_timeout,
            )
            if not _clearance_passes(transform, args.clearance_x):
                observed = (
                    "unavailable"
                    if transform is None
                    else f"{transform.transform.translation.x:.3f}"
                )
                navigator.get_logger().error(
                    "clearance refinement stopped at "
                    "EXIT_ACCEPTANCE_PENDING: fresh "
                    f"cart_frame.x={observed}, required>="
                    f"{args.clearance_x:.3f}; loaded footprint retained"
                )
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().info(
                "CLEAR_OF_SHELF verified after bounded clearance "
                "refinement, stop/settle, and fresh shelf geometry"
            )
            if not _apply_unloaded_footprint_verified(
                navigator,
                args.unloaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().info(
                "clearance refinement stopped at "
                "UNLOADED_FOOTPRINT_VERIFIED before return navigation"
            )
            return int(ExitCode.SUCCEEDED)

        if args.return_only:
            if not (
                args.confirm_clear_of_shelf
                and args.confirm_unloaded_footprint
                and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "return-only requires explicit clear-of-shelf, "
                    "unloaded-footprint, and stopped-state confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "return-only requires existing AMCL localization; "
                    "initial pose override is not allowed"
                )
                return int(ExitCode.UNKNOWN)
            if not _wait_for_existing_localization(
                navigator,
                args.localization_timeout,
            ):
                navigator.get_logger().error(
                    "No existing AMCL pose received before localization "
                    "timeout"
                )
                return int(ExitCode.UNKNOWN)

            navigator.waitUntilNav2Active()
            if not _apply_unloaded_footprint_verified(
                navigator,
                args.unloaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            init_pose = _pose(
                navigator,
                args.frame_id,
                args.return_x,
                args.return_y,
                args.return_yaw,
            )
            return int(
                _navigate_to_init(
                    navigator,
                    init_pose,
                    args.return_timeout,
                )
            )

        if args.loaded_footprint_only:
            if not (
                args.confirm_lift_accepted and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "loaded-footprint-only requires explicit lift and "
                    "stopped-state confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            navigator.get_logger().info(
                "Slice stopped before shipping navigation"
            )
            return int(ExitCode.SUCCEEDED)

        if args.shipping_only:
            if not (
                args.confirm_lift_accepted and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "shipping-only requires explicit lift and stopped-state "
                    "confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "shipping-only requires existing AMCL localization; "
                    "initial pose override is not allowed"
                )
                return int(ExitCode.UNKNOWN)
            if not _wait_for_existing_localization(
                navigator,
                args.localization_timeout,
            ):
                navigator.get_logger().error(
                    "No existing AMCL pose received before localization "
                    "timeout"
                )
                return int(ExitCode.UNKNOWN)

            navigator.waitUntilNav2Active()
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            shipping_pose = _pose(
                navigator,
                args.frame_id,
                args.shipping_x,
                args.shipping_y,
                args.shipping_yaw,
            )
            return int(
                _navigate_to_shipping(
                    navigator,
                    shipping_pose,
                    args.shipping_timeout,
                    args.base_frame,
                    args.shipping_position_tolerance,
                    args.shipping_yaw_tolerance,
                    args.shipping_max_yaw_correction,
                    args.shipping_alignment_timeout,
                    args.shipping_alignment_settle,
                    args.shipping_pose_lookup_timeout,
                    args.shipping_yaw_correction_ratio,
                    args.shipping_yaw_correction_rounds,
                )
            )

        if args.shipping_alignment_only:
            if not (
                args.confirm_lift_accepted and args.confirm_robot_stopped
            ):
                navigator.get_logger().error(
                    "shipping-alignment-only requires explicit lift and "
                    "stopped-state confirmations"
                )
                return int(ExitCode.UNKNOWN)
            if initial_pose is not None:
                navigator.get_logger().error(
                    "shipping-alignment-only requires existing AMCL "
                    "localization; initial pose override is not allowed"
                )
                return int(ExitCode.UNKNOWN)
            if not _wait_for_existing_localization(
                navigator,
                args.localization_timeout,
            ):
                navigator.get_logger().error(
                    "No existing AMCL pose received before localization "
                    "timeout"
                )
                return int(ExitCode.UNKNOWN)

            navigator.waitUntilNav2Active()
            if not _apply_loaded_footprint_verified(
                navigator,
                args.loaded_footprint,
                args.footprint_timeout,
                args.footprint_edge_tolerance,
            ):
                return int(ExitCode.UNKNOWN)
            shipping_pose = _pose(
                navigator,
                args.frame_id,
                args.shipping_x,
                args.shipping_y,
                args.shipping_yaw,
            )
            return int(
                _align_at_shipping(
                    navigator,
                    shipping_pose,
                    args.base_frame,
                    args.shipping_position_tolerance,
                    args.shipping_yaw_tolerance,
                    args.shipping_max_yaw_correction,
                    args.shipping_alignment_timeout,
                    args.shipping_alignment_settle,
                    args.shipping_pose_lookup_timeout,
                    args.shipping_yaw_correction_ratio,
                    args.shipping_yaw_correction_rounds,
                )
            )

        if initial_pose is not None:
            navigator.setInitialPose(
                _pose(navigator, args.frame_id, *initial_pose)
            )
        elif not _wait_for_existing_localization(
            navigator,
            args.localization_timeout,
        ):
            navigator.get_logger().error(
                "No existing AMCL pose received before localization timeout"
            )
            return int(ExitCode.UNKNOWN)

        navigator.waitUntilNav2Active()
        loading_pose = _pose(
            navigator,
            args.frame_id,
            args.loading_x,
            args.loading_y,
            args.loading_yaw,
        )

        if not navigator.goToPose(loading_pose):
            navigator.get_logger().error("loading_position goal was rejected")
            return int(ExitCode.GOAL_REJECTED)

        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)

        result = navigator.getResult()
        exit_code = classify_task_result(result, TaskResult)
        if exit_code != ExitCode.SUCCEEDED:
            navigator.get_logger().error(
                f"loading_position goal did not succeed: {result}"
            )
            return int(exit_code)

        navigator.get_logger().info("loading_position goal succeeded")
        if integrated_mode:
            return int(_run_integrated_mission(navigator, args))
        if not args.detection_only and not args.approach_and_elevator:
            return int(exit_code)
        if args.detection_only:
            detection_ok = _wait_for_detection(
                navigator,
                args.shelf_service,
                args.cart_frame,
                args.base_frame,
                args.detection_timeout,
            )
            return int(
                ExitCode.SUCCEEDED if detection_ok else ExitCode.UNKNOWN
            )
        if not _request_stepwise_attach(
            navigator, args.shelf_service, args.attach_timeout
        ):
            return int(ExitCode.UNKNOWN)
        if not _wait_for_external_lift_acceptance(
            navigator, args.elevator_wait
        ):
            navigator.get_logger().warning(
                "Slice 2B stopped at lift_acceptance_pending; external "
                "acceptance is required"
            )
            return int(ExitCode.UNKNOWN)
        return int(ExitCode.UNKNOWN)
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
