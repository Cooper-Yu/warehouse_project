"""Checkpoint 12 loading and C9-style shelf-attach orchestration."""

import argparse
import math
import sys
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from nav2_apps.pose_config import (
    SIM_INIT_POSE,
    SIM_LOADING_POSE,
    SIM_SHIPPING_POSE,
    optional_initial_pose,
)
from nav2_apps.result_gate import ExitCode, classify_task_result


SIM_LOADED_FOOTPRINT = (
    "[[0.40, 0.45], [-0.40, 0.45], "
    "[-0.40, -0.45], [0.40, -0.45]]"
)
SIM_UNLOADED_FOOTPRINT = (
    "[[0.25, 0.25], [-0.25, 0.25], "
    "[-0.25, -0.25], [0.25, -0.25]]"
)


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
    parser.add_argument("--clearance-refine-distance", type=float, default=0.02)
    parser.add_argument("--clearance-refine-speed", type=float, default=0.03)
    parser.add_argument("--clearance-refine-motion-timeout", type=float, default=10.0)
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
                "bounded shelf exit rejected: odom TF unavailable"
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
            "bounded shelf exit started: "
            f"target_distance={distance:.3f} speed={command.linear.x:.3f} "
            f"accepted_odom_yaw={start_yaw:.3f}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            current = lookup()
            if current is None:
                if time.monotonic() - last_tf_time >= lookup_timeout:
                    navigator.get_logger().error(
                        "bounded shelf exit stopped: odom TF stale"
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
                    "bounded shelf exit stopped: heading drift "
                    f"{heading_drift:.3f} exceeds "
                    f"{heading_tolerance:.3f}"
                )
                return False
            if abs(lateral) > lateral_tolerance:
                navigator.get_logger().error(
                    "bounded shelf exit stopped: lateral drift "
                    f"{lateral:.3f} exceeds {lateral_tolerance:.3f}"
                )
                return False
            if reverse_progress >= distance:
                navigator.get_logger().info(
                    "bounded shelf exit odom target complete: "
                    f"target={distance:.3f} "
                    f"reverse_progress={reverse_progress:.3f} "
                    f"lateral={lateral:.3f} "
                    f"heading_drift={heading_drift:.3f}"
                )
                return True
            publisher.publish(command)

        navigator.get_logger().error(
            "bounded shelf exit timed out before odom target: "
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
