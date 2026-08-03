"""Checkpoint 12 loading and C9-style shelf-attach orchestration."""

import argparse
import math
import sys
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav2_apps.pose_config import (
    SIM_LOADING_POSE,
    SIM_SHIPPING_POSE,
    optional_initial_pose,
)
from nav2_apps.result_gate import ExitCode, classify_task_result


SIM_LOADED_FOOTPRINT = (
    "[[0.40, 0.45], [-0.40, 0.45], "
    "[-0.40, -0.45], [0.40, -0.45]]"
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
    return parser


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


def _navigate_to_shipping(
    navigator: BasicNavigator,
    shipping_pose: PoseStamped,
    timeout: float,
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
    navigator.get_logger().info(
        "shipping_position goal succeeded; "
        "Slice 3A stopped at AT_SHIPPING"
    )
    return ExitCode.SUCCEEDED


def main(argv: Optional[List[str]] = None) -> int:
    args, ros_args = _parser().parse_known_args(argv)
    try:
        initial_pose = optional_initial_pose(args)
    except ValueError as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return int(ExitCode.UNKNOWN)

    rclpy.init(args=ros_args)
    navigator = BasicNavigator()
    try:
        operation_modes = (
            args.detection_only,
            args.approach_and_elevator,
            args.loaded_footprint_only,
            args.shipping_only,
        )
        if sum(bool(mode) for mode in operation_modes) > 1:
            navigator.get_logger().error(
                "detection, attach, footprint-only, and shipping-only "
                "modes are mutually exclusive"
            )
            return int(ExitCode.UNKNOWN)

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
