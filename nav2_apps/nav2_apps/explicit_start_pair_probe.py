"""Diagnostic-only paired ComputePathToPose probe with one explicit start."""

import argparse
from copy import deepcopy
import hashlib
import json
import math
import sys
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import tf2_ros

from nav2_apps.pose_config import SIM_SHIPPING_POSE


PATH_READY = "PATH_READY"
NO_PATH = "NO_PATH"
UNCERTAIN = "UNCERTAIN"


def _yaw_from_quaternion(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _pose_delta(first: PoseStamped, second: PoseStamped) -> tuple:
    position = math.hypot(
        second.pose.position.x - first.pose.position.x,
        second.pose.position.y - first.pose.position.y,
    )
    first_yaw = _yaw_from_quaternion(first.pose.orientation)
    second_yaw = _yaw_from_quaternion(second.pose.orientation)
    return position, abs(_normalize_angle(second_yaw - first_yaw))


def _costmap_digest(message: OccupancyGrid) -> str:
    digest = hashlib.sha256()
    digest.update(message.header.frame_id.encode("utf-8"))
    digest.update(str(message.info.resolution).encode("ascii"))
    digest.update(str(message.info.width).encode("ascii"))
    digest.update(str(message.info.height).encode("ascii"))
    origin = message.info.origin
    digest.update(repr((
        origin.position.x,
        origin.position.y,
        origin.position.z,
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    )).encode("ascii"))
    digest.update(bytes(value & 0xFF for value in message.data))
    return digest.hexdigest()


def _path_is_usable(result, frame_id: str) -> bool:
    path = result.path
    if path.header.frame_id != frame_id or len(path.poses) < 2:
        return False
    for stamped_pose in (path.poses[0], path.poses[-1]):
        position = stamped_pose.pose.position
        if not all(math.isfinite(value) for value in (
            position.x, position.y, position.z
        )):
            return False
    return True


class ExplicitStartPairProbe(Node):
    """Observe stopped state and issue two identical planning-only requests."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("explicit_start_pair_probe")
        self.args = args
        self.motion_seen = False
        self.costmap: Optional[OccupancyGrid] = None
        self.costmap_sequence = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.client = ActionClient(
            self, ComputePathToPose, args.action_name
        )
        self.create_subscription(
            Twist, args.cmd_vel_topic, self._on_twist, 20
        )
        self.create_subscription(
            OccupancyGrid, args.costmap_topic, self._on_costmap, 10
        )

    def _on_twist(self, message: Twist) -> None:
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        if any(
            abs(value) > self.args.zero_twist_tolerance
            for value in values
        ):
            self.motion_seen = True

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap = message
        self.costmap_sequence += 1

    def _spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(predicate())

    def _fresh_pose(self) -> Optional[PoseStamped]:
        deadline = time.monotonic() + self.args.tf_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.frame_id,
                    self.args.base_frame,
                    rclpy.time.Time(),
                )
            except tf2_ros.TransformException:
                continue
            pose = PoseStamped()
            pose.header.frame_id = self.args.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        return None

    def _goal_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.args.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.args.goal_x
        pose.pose.position.y = self.args.goal_y
        pose.pose.orientation.z = math.sin(self.args.goal_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.args.goal_yaw / 2.0)
        return pose

    def _request(self, start: PoseStamped, goal: PoseStamped) -> dict:
        request = ComputePathToPose.Goal()
        request.start = deepcopy(start)
        request.goal = deepcopy(goal)
        request.planner_id = self.args.planner_id
        request.use_start = True
        if not self.client.wait_for_server(
            timeout_sec=self.args.action_timeout
        ):
            return {"classification": UNCERTAIN, "reason": "server_timeout"}
        send_future = self.client.send_goal_async(request)
        if not self._spin_until(send_future.done, self.args.action_timeout):
            return {"classification": UNCERTAIN, "reason": "accept_timeout"}
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {"classification": UNCERTAIN, "reason": "rejected"}
        result_future = handle.get_result_async()
        if not self._spin_until(result_future.done, self.args.action_timeout):
            handle.cancel_goal_async()
            return {"classification": UNCERTAIN, "reason": "result_timeout"}
        wrapped = result_future.result()
        if wrapped is None:
            return {"classification": UNCERTAIN, "reason": "missing_result"}
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return {
                "classification": NO_PATH,
                "reason": "planner_non_success",
                "status": int(wrapped.status),
            }
        if not _path_is_usable(wrapped.result, self.args.frame_id):
            return {"classification": UNCERTAIN, "reason": "invalid_path"}
        return {
            "classification": PATH_READY,
            "reason": "usable_path",
            "status": int(wrapped.status),
            "poses": len(wrapped.result.path.poses),
        }

    def run(self) -> int:
        if not self._spin_until(
            lambda: self.costmap is not None, self.args.costmap_timeout
        ):
            self.get_logger().error("PAIR_PROBE_UNCERTAIN: no costmap")
            return 2
        start = self._fresh_pose()
        if start is None:
            self.get_logger().error("PAIR_PROBE_UNCERTAIN: no map TF")
            return 2
        if self.motion_seen:
            self.get_logger().error("PAIR_PROBE_REJECTED: motion before probe")
            return 3
        goal = self._goal_pose()
        initial_hash = _costmap_digest(self.costmap)
        initial_sequence = self.costmap_sequence
        first = self._request(start, goal)
        settle_deadline = time.monotonic() + self.args.between_wait
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        middle_pose = self._fresh_pose()
        if middle_pose is None or self.costmap is None or self.motion_seen:
            self.get_logger().error("PAIR_PROBE_REJECTED: unstable midpoint")
            return 3
        middle_hash = _costmap_digest(self.costmap)
        middle_sequence = self.costmap_sequence
        middle_position_delta, middle_yaw_delta = _pose_delta(
            start, middle_pose
        )
        if (
            middle_position_delta > self.args.max_tf_position_delta
            or middle_yaw_delta > self.args.max_tf_yaw_delta
            or middle_hash != initial_hash
        ):
            self.get_logger().error("PAIR_PROBE_REJECTED: state changed")
            return 3
        second = self._request(start, goal)
        final_pose = self._fresh_pose()
        if final_pose is None or self.costmap is None or self.motion_seen:
            self.get_logger().error(
                "PAIR_PROBE_REJECTED: unstable final state"
            )
            return 3
        final_hash = _costmap_digest(self.costmap)
        final_sequence = self.costmap_sequence
        final_position_delta, final_yaw_delta = _pose_delta(start, final_pose)
        stable = (
            final_position_delta <= self.args.max_tf_position_delta
            and final_yaw_delta <= self.args.max_tf_yaw_delta
            and final_hash == initial_hash
        )
        report = {
            "start": {
                "x": start.pose.position.x,
                "y": start.pose.position.y,
                "yaw": _yaw_from_quaternion(start.pose.orientation),
            },
            "goal": {
                "x": goal.pose.position.x,
                "y": goal.pose.position.y,
                "yaw": self.args.goal_yaw,
            },
            "first": first,
            "second": second,
            "motion_seen": self.motion_seen,
            "stable": stable,
            "tf_delta": {
                "mid_position": middle_position_delta,
                "mid_yaw": middle_yaw_delta,
                "final_position": final_position_delta,
                "final_yaw": final_yaw_delta,
            },
            "costmap": {
                "initial_sequence": initial_sequence,
                "middle_sequence": middle_sequence,
                "final_sequence": final_sequence,
                "initial_hash": initial_hash,
                "middle_hash": middle_hash,
                "final_hash": final_hash,
            },
        }
        self.get_logger().info("EXPLICIT_START_PAIR_RESULT " + json.dumps(
            report, sort_keys=True
        ))
        if not stable:
            return 3
        if first["classification"] == second["classification"]:
            return 0
        return 4


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run two stopped explicit-start planning-only probes."
    )
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--base-frame", default="robot_base_footprint")
    parser.add_argument("--action-name", default="/compute_path_to_pose")
    parser.add_argument(
        "--costmap-topic", default="/global_costmap/costmap_raw"
    )
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--goal-x", type=float, default=SIM_SHIPPING_POSE[0])
    parser.add_argument("--goal-y", type=float, default=SIM_SHIPPING_POSE[1])
    parser.add_argument("--goal-yaw", type=float, default=SIM_SHIPPING_POSE[2])
    parser.add_argument("--tf-timeout", type=float, default=5.0)
    parser.add_argument("--costmap-timeout", type=float, default=5.0)
    parser.add_argument("--action-timeout", type=float, default=5.0)
    parser.add_argument("--between-wait", type=float, default=0.5)
    parser.add_argument("--max-tf-position-delta", type=float, default=0.02)
    parser.add_argument("--max-tf-yaw-delta", type=float, default=0.02)
    parser.add_argument("--zero-twist-tolerance", type=float, default=1e-4)
    return parser


def main(argv=None) -> int:
    raw = sys.argv if argv is None else argv
    application_args = remove_ros_args(args=raw)
    if argv is None:
        application_args = application_args[1:]
    args = _parser().parse_args(application_args)
    rclpy.init(args=argv)
    node = ExplicitStartPairProbe(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
