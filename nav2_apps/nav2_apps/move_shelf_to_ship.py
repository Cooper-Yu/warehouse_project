"""Slice 1: send one simulation-profile Nav2 goal to loading_position."""

import argparse
import math
import sys
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav2_apps.result_gate import ExitCode, classify_task_result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send the Checkpoint 12 Slice 1 simulation goal. The course pose "
            "coordinates must be supplied explicitly; this program does not "
            "guess them."
        )
    )
    parser.add_argument("--loading-x", type=float, required=True)
    parser.add_argument("--loading-y", type=float, required=True)
    parser.add_argument("--loading-yaw", type=float, required=True)
    parser.add_argument("--initial-x", type=float)
    parser.add_argument("--initial-y", type=float)
    parser.add_argument("--initial-yaw", type=float)
    parser.add_argument("--frame-id", default="map")
    return parser


def _optional_initial_pose(
    args: argparse.Namespace,
) -> Optional[Tuple[float, float, float]]:
    values = (args.initial_x, args.initial_y, args.initial_yaw)
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError(
            "--initial-x, --initial-y, and --initial-yaw must be supplied "
            "together"
        )
    return values


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


def main(argv: Optional[List[str]] = None) -> int:
    args, ros_args = _parser().parse_known_args(argv)
    try:
        initial_pose = _optional_initial_pose(args)
    except ValueError as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return int(ExitCode.UNKNOWN)

    rclpy.init(args=ros_args)
    navigator = BasicNavigator()
    try:
        if initial_pose is not None:
            navigator.setInitialPose(
                _pose(navigator, args.frame_id, *initial_pose)
            )

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
        if exit_code == ExitCode.SUCCEEDED:
            navigator.get_logger().info("loading_position goal succeeded")
        else:
            navigator.get_logger().error(
                f"loading_position goal did not succeed: {result}"
            )
        return int(exit_code)
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
