"""Bounded read-only motion evidence monitor for Checkpoint 12."""

import csv
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw from a geometry quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def phase_from_log(message: str, current: str) -> str:
    """Map selected mission log messages to a compact evidence phase."""
    text = message.lower()
    transitions = (
        ("navigating to shipping_position", "NAV_TO_SHIPPING"),
        ("shipping_position goal succeeded", "AT_SHIPPING"),
        ("navigating to goal", "NAV_TO_LOADING"),
        ("loading_position goal succeeded", "AT_LOADING"),
        ("mode=detection-only", "DETECTION_ONLY"),
        ("stepwise attach sample", "SHELF_APPROACH"),
        ("locking final center approach", "CENTER_LOCK"),
        ("locking from estimated remaining distance", "CENTER_LOCK"),
        ("published elevator-up", "ELEVATOR_UP"),
        ("lift_acceptance_pending", "LIFT_ACCEPTANCE_PENDING"),
    )
    for marker, phase in transitions:
        if marker in text:
            return phase
    return current


def _pose_values(pose) -> tuple:
    return (
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(pose.orientation),
    )


class MotionEvidenceMonitor(Node):
    """Sample motion topics and selected ROS logs without publishing."""

    def __init__(self) -> None:
        super().__init__("motion_evidence_monitor")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("sample_hz", 2.0)

        output_value = str(self.get_parameter("output_dir").value)
        if not output_value:
            raise RuntimeError("output_dir parameter is required")
        self.output_dir = Path(output_value)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.phase = "IDLE"
        self.map_pose: Optional[tuple] = None
        self.odom_pose: Optional[tuple] = None
        self.feedback_pose: Optional[tuple] = None
        self.cmd_vel = (0.0, 0.0)
        self.distance_remaining: Optional[float] = None
        self.recoveries: Optional[int] = None
        self.global_path_poses = 0
        self.local_path_poses = 0

        self.sample_count = 0
        self.feedback_count = 0
        self.event_count = 0
        self.max_abs_linear = 0.0
        self.max_abs_angular = 0.0
        self.max_recoveries = 0
        self.phases = {self.phase}
        self.first_map_pose: Optional[tuple] = None
        self.last_map_pose: Optional[tuple] = None
        self.first_odom_pose: Optional[tuple] = None
        self.last_odom_pose: Optional[tuple] = None

        self._motion_file = (self.output_dir / "motion.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._event_file = (self.output_dir / "events.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._motion_writer = csv.writer(self._motion_file)
        self._event_writer = csv.writer(self._event_file)
        self._motion_writer.writerow(
            [
                "utc_time",
                "ros_time_sec",
                "phase",
                "map_x",
                "map_y",
                "map_yaw",
                "odom_x",
                "odom_y",
                "odom_yaw",
                "feedback_x",
                "feedback_y",
                "feedback_yaw",
                "cmd_linear_x",
                "cmd_angular_z",
                "distance_remaining",
                "recoveries",
                "global_path_poses",
                "local_path_poses",
            ]
        )
        self._event_writer.writerow(
            ["utc_time", "ros_stamp_sec", "level", "node", "message"]
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._cmd_vel_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._feedback_callback,
            10,
        )
        self.create_subscription(
            NavPath, "/plan", self._global_path_callback, 10
        )
        self.create_subscription(
            NavPath, "/local_plan", self._local_path_callback, 10
        )
        self.create_subscription(Log, "/rosout", self._log_callback, 100)

        sample_hz = max(float(self.get_parameter("sample_hz").value), 0.2)
        self.create_timer(1.0 / sample_hz, self._sample)
        self.get_logger().info(
            "motion evidence monitor ready; read-only subscriptions only"
        )

    def _amcl_callback(self, message: PoseWithCovarianceStamped) -> None:
        self.map_pose = _pose_values(message.pose.pose)
        if self.first_map_pose is None:
            self.first_map_pose = self.map_pose
        self.last_map_pose = self.map_pose

    def _odom_callback(self, message: Odometry) -> None:
        self.odom_pose = _pose_values(message.pose.pose)
        if self.first_odom_pose is None:
            self.first_odom_pose = self.odom_pose
        self.last_odom_pose = self.odom_pose

    def _cmd_vel_callback(self, message: Twist) -> None:
        self.cmd_vel = (message.linear.x, message.angular.z)
        self.max_abs_linear = max(self.max_abs_linear, abs(message.linear.x))
        self.max_abs_angular = max(
            self.max_abs_angular, abs(message.angular.z)
        )

    def _feedback_callback(self, message) -> None:
        feedback = message.feedback
        self.feedback_pose = _pose_values(feedback.current_pose.pose)
        self.distance_remaining = feedback.distance_remaining
        self.recoveries = int(feedback.number_of_recoveries)
        self.feedback_count += 1
        self.max_recoveries = max(self.max_recoveries, self.recoveries)

    def _global_path_callback(self, message: NavPath) -> None:
        self.global_path_poses = len(message.poses)

    def _local_path_callback(self, message: NavPath) -> None:
        self.local_path_poses = len(message.poses)

    def _log_callback(self, message: Log) -> None:
        self.phase = phase_from_log(message.msg, self.phase)
        self.phases.add(self.phase)
        selected_names = (
            "basic_navigator",
            "shelf_detection_server",
            "controller_server",
            "behavior_server",
            "bt_navigator",
            "lifecycle_manager_navigation",
        )
        selected = any(name in message.name for name in selected_names)
        if not selected and int(message.level) < 30:
            return
        stamp = float(message.stamp.sec) + float(message.stamp.nanosec) / 1e9
        self._event_writer.writerow(
            [
                datetime.now(timezone.utc).isoformat(),
                f"{stamp:.9f}",
                int(message.level),
                message.name,
                message.msg,
            ]
        )
        self._event_file.flush()
        self.event_count += 1

    @staticmethod
    def _values_or_blank(values: Optional[tuple]) -> tuple:
        return values if values is not None else ("", "", "")

    def _sample(self) -> None:
        ros_time = self.get_clock().now().nanoseconds / 1e9
        self._motion_writer.writerow(
            [
                datetime.now(timezone.utc).isoformat(),
                f"{ros_time:.9f}",
                self.phase,
                *self._values_or_blank(self.map_pose),
                *self._values_or_blank(self.odom_pose),
                *self._values_or_blank(self.feedback_pose),
                self.cmd_vel[0],
                self.cmd_vel[1],
                (
                    ""
                    if self.distance_remaining is None
                    else self.distance_remaining
                ),
                "" if self.recoveries is None else self.recoveries,
                self.global_path_poses,
                self.local_path_poses,
            ]
        )
        self._motion_file.flush()
        self.sample_count += 1

    @staticmethod
    def _format_pose(pose: Optional[tuple]) -> str:
        if pose is None:
            return "not observed"
        return f"x={pose[0]:.4f}, y={pose[1]:.4f}, yaw={pose[2]:.4f}"

    def close(self) -> None:
        self._motion_file.flush()
        self._event_file.flush()
        self._motion_file.close()
        self._event_file.close()
        summary = self.output_dir / "summary.md"
        summary.write_text(
            "\n".join(
                [
                    "# Bounded Motion Evidence Summary",
                    "",
                    f"- Samples: {self.sample_count}",
                    f"- Feedback samples: {self.feedback_count}",
                    f"- Selected events: {self.event_count}",
                    f"- Phases: {', '.join(sorted(self.phases))}",
                    "- First map pose: "
                    f"{self._format_pose(self.first_map_pose)}",
                    "- Last map pose: "
                    f"{self._format_pose(self.last_map_pose)}",
                    "- First odom pose: "
                    f"{self._format_pose(self.first_odom_pose)}",
                    "- Last odom pose: "
                    f"{self._format_pose(self.last_odom_pose)}",
                    f"- Max |linear.x|: {self.max_abs_linear:.4f}",
                    f"- Max |angular.z|: {self.max_abs_angular:.4f}",
                    f"- Max recoveries: {self.max_recoveries}",
                    "",
                    "Raw `motion.csv`, `events.csv`, and process logs "
                    "remain local.",
                ]
            )
            + "\n",
            encoding="utf-8",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionEvidenceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
