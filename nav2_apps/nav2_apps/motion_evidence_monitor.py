"""Bounded read-only motion evidence monitor for Checkpoint 12."""

import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformException, TransformListener


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
    if "navigating to goal" in text:
        if current in ("NAV_TO_SHIPPING", "NAV_TO_INIT"):
            return current
        return "NAV_TO_LOADING"
    transitions = (
        ("navigating to init_position", "NAV_TO_INIT"),
        ("init_position goal succeeded", "AT_INIT"),
        ("navigating to shipping_position", "NAV_TO_SHIPPING"),
        ("shipping_position goal succeeded", "AT_SHIPPING"),
        ("loaded egress:", "LOADED_EGRESS"),
        ("loaded_egress_complete", "SHIPPING_NAV_READY"),
        ("loaded_localization_stable", "LOCALIZATION_READY"),
        ("loaded_path_probe_ready", "PATH_PROBE_READY"),
        ("loaded_path_probe_no_path", "PATH_PROBE_NO_PATH"),
        ("loaded_path_probe_uncertain", "PATH_PROBE_UNCERTAIN"),
        (
            "loaded_shipping_no_path_at_aligned_heading",
            "ALIGNED_NO_PATH",
        ),
        ("loaded_shipping_speeds_verified", "LOADED_SPEEDS_READY"),
        ("shipping canceled: loaded localization jump", "LOCALIZATION_ABORT"),
        ("published elevator-down", "ELEVATOR_DOWN"),
        ("lower_acceptance_pending", "LOWER_ACCEPTANCE_PENDING"),
        ("bounded shelf exit started", "SHELF_EXIT"),
        ("clear_of_shelf verified", "CLEAR_OF_SHELF"),
        ("unloaded_footprint_verified", "UNLOADED_FOOTPRINT_VERIFIED"),
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


def _point_in_polygon(x: float, y: float, points: list) -> bool:
    """Return whether a point lies inside a non-self-intersecting polygon."""
    inside = False
    previous = len(points) - 1
    for current in range(len(points)):
        current_x, current_y = points[current]
        previous_x, previous_y = points[previous]
        crosses = (current_y > y) != (previous_y > y)
        if crosses:
            boundary_x = (
                (previous_x - current_x)
                * (y - current_y)
                / (previous_y - current_y)
                + current_x
            )
            if x < boundary_x:
                inside = not inside
        previous = current
    return inside


def analyze_costmap_start(costmap, footprint, pose: tuple) -> dict:
    """Summarize the start cell and costs covered by a map-frame footprint."""
    metadata = costmap.metadata
    resolution = float(metadata.resolution)
    size_x = int(metadata.size_x)
    size_y = int(metadata.size_y)
    origin_x = float(metadata.origin.position.x)
    origin_y = float(metadata.origin.position.y)
    result = {
        "start_grid_x": None,
        "start_grid_y": None,
        "start_cost": None,
        "footprint_cells": 0,
        "footprint_free": 0,
        "footprint_inflated": 0,
        "footprint_inscribed": 0,
        "footprint_lethal": 0,
        "footprint_unknown": 0,
        "footprint_outside": 0,
    }
    if resolution <= 0.0 or size_x <= 0 or size_y <= 0:
        return result

    start_x = math.floor((pose[0] - origin_x) / resolution)
    start_y = math.floor((pose[1] - origin_y) / resolution)
    result["start_grid_x"] = start_x
    result["start_grid_y"] = start_y
    if 0 <= start_x < size_x and 0 <= start_y < size_y:
        result["start_cost"] = int(costmap.data[start_y * size_x + start_x])

    points = [(float(point.x), float(point.y)) for point in footprint]
    if len(points) < 3:
        return result
    min_x = math.floor((min(x for x, _ in points) - origin_x) / resolution)
    max_x = math.floor((max(x for x, _ in points) - origin_x) / resolution)
    min_y = math.floor((min(y for _, y in points) - origin_y) / resolution)
    max_y = math.floor((max(y for _, y in points) - origin_y) / resolution)
    for grid_y in range(min_y, max_y + 1):
        for grid_x in range(min_x, max_x + 1):
            world_x = origin_x + (grid_x + 0.5) * resolution
            world_y = origin_y + (grid_y + 0.5) * resolution
            if not _point_in_polygon(world_x, world_y, points):
                continue
            result["footprint_cells"] += 1
            if not (0 <= grid_x < size_x and 0 <= grid_y < size_y):
                result["footprint_outside"] += 1
                continue
            cost = int(costmap.data[grid_y * size_x + grid_x])
            if cost == 255:
                result["footprint_unknown"] += 1
            elif cost == 254:
                result["footprint_lethal"] += 1
            elif cost == 253:
                result["footprint_inscribed"] += 1
            elif cost == 0:
                result["footprint_free"] += 1
            else:
                result["footprint_inflated"] += 1
    return result


class MotionEvidenceMonitor(Node):
    """Sample motion topics and selected ROS logs without publishing."""

    def __init__(self) -> None:
        super().__init__("motion_evidence_monitor")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("sample_hz", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "robot_base_footprint")

        output_value = str(self.get_parameter("output_dir").value)
        if not output_value:
            raise RuntimeError("output_dir parameter is required")
        self.output_dir = Path(output_value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.phase = "IDLE"
        self.map_pose: Optional[tuple] = None
        self.odom_pose: Optional[tuple] = None
        self.feedback_pose: Optional[tuple] = None
        self.cmd_vel = (0.0, 0.0)
        self.distance_remaining: Optional[float] = None
        self.recoveries: Optional[int] = None
        self.global_path_poses = 0
        self.local_path_poses = 0
        self.global_costmap: Optional[Costmap] = None
        self.global_footprint: Optional[PolygonStamped] = None
        self.global_costmap_receipt_ros_time: Optional[float] = None
        self.global_footprint_receipt_ros_time: Optional[float] = None
        self.global_costmap_sequence = 0
        self.global_footprint_sequence = 0
        self.latest_planner_diagnostic: Optional[tuple] = None
        self.costmap_snapshot_count = 0
        self._probe_sequence = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self._costmap_file = (self.output_dir / "costmap_snapshots.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._costmap_writer = csv.writer(self._costmap_file)
        self._costmap_dir = self.output_dir / "costmap_snapshots"
        self._costmap_dir.mkdir(parents=True, exist_ok=True)
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
        self._costmap_writer.writerow(
            [
                "sequence",
                "utc_time",
                "capture_ros_time_sec",
                "capture_clock_domain",
                "probe_result",
                "probe_log_stamp_sec",
                "probe_log_clock_domain",
                "probe_receipt_ros_time_sec",
                "planner_log_stamp_sec",
                "planner_receipt_ros_time_sec",
                "planner_receipt_delta_sec",
                "planner_log_level",
                "planner_log_message",
                "tf_x",
                "tf_y",
                "tf_yaw",
                "costmap_stamp_sec",
                "costmap_receipt_ros_time_sec",
                "costmap_sequence",
                "costmap_data_sha256",
                "costmap_age_sec",
                "costmap_frame",
                "resolution",
                "size_x",
                "size_y",
                "origin_x",
                "origin_y",
                "footprint_stamp_sec",
                "footprint_receipt_ros_time_sec",
                "footprint_sequence",
                "footprint_age_sec",
                "footprint_frame",
                "footprint_points",
                "start_grid_x",
                "start_grid_y",
                "start_cost",
                "footprint_cells",
                "footprint_free",
                "footprint_inflated",
                "footprint_inscribed",
                "footprint_lethal",
                "footprint_unknown",
                "footprint_outside",
                "raw_snapshot",
                "status",
            ]
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
        self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._global_costmap_callback,
            10,
        )
        self.create_subscription(
            PolygonStamped,
            "/global_costmap/published_footprint",
            self._global_footprint_callback,
            10,
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

    def _global_costmap_callback(self, message: Costmap) -> None:
        self.global_costmap = message
        self.global_costmap_receipt_ros_time = (
            self.get_clock().now().nanoseconds / 1e9
        )
        self.global_costmap_sequence += 1

    def _global_footprint_callback(self, message: PolygonStamped) -> None:
        self.global_footprint = message
        self.global_footprint_receipt_ros_time = (
            self.get_clock().now().nanoseconds / 1e9
        )
        self.global_footprint_sequence += 1

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    @staticmethod
    def _probe_result(message: str) -> Optional[str]:
        text = message.lower()
        for marker, result in (
            ("loaded_path_probe_ready", "PATH_READY"),
            ("loaded_path_probe_no_path", "NO_PATH"),
            ("loaded_path_probe_uncertain", "UNCERTAIN"),
        ):
            if marker in text:
                return result
        return None

    def _capture_probe_costmap(
        self,
        result: str,
        log_stamp: float,
        probe_receipt_ros_time: float,
    ) -> None:
        self._probe_sequence += 1
        sequence = self._probe_sequence
        capture_time = self.get_clock().now()
        capture_seconds = capture_time.nanoseconds / 1e9
        planner_stamp = None
        planner_receipt_ros_time = None
        planner_level = None
        planner_message = ""
        planner_delta = None
        if self.latest_planner_diagnostic is not None:
            (
                candidate_stamp,
                candidate_receipt_ros_time,
                candidate_level,
                candidate_message,
            ) = self.latest_planner_diagnostic
            candidate_delta = (
                probe_receipt_ros_time - candidate_receipt_ros_time
            )
            if 0.0 <= candidate_delta <= 2.0:
                planner_stamp = candidate_stamp
                planner_receipt_ros_time = candidate_receipt_ros_time
                planner_level = candidate_level
                planner_message = candidate_message
                planner_delta = candidate_delta
        status = []
        pose = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.10),
            )
            pose = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(transform.transform.rotation),
            )
        except TransformException as error:
            status.append(f"tf_unavailable:{error}")

        costmap = self.global_costmap
        footprint_message = self.global_footprint
        footprint = []
        analysis = {}
        raw_name = ""
        if costmap is None:
            status.append("costmap_unavailable")
        if footprint_message is None:
            status.append("footprint_unavailable")
        else:
            footprint = list(footprint_message.polygon.points)
        if costmap is not None and pose is not None and footprint:
            analysis = analyze_costmap_start(costmap, footprint, pose)
            raw_name = f"probe_{sequence:03d}_{result.lower()}.json.gz"
            metadata = costmap.metadata
            raw_payload = {
                "sequence": sequence,
                "probe_result": result,
                "probe_log_stamp_sec": log_stamp,
                "probe_receipt_ros_time_sec": probe_receipt_ros_time,
                "capture_ros_time_sec": capture_seconds,
                "tf_pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
                "costmap": {
                    "stamp_sec": self._stamp_seconds(costmap.header.stamp),
                    "receipt_ros_time_sec": (
                        self.global_costmap_receipt_ros_time
                    ),
                    "sequence": self.global_costmap_sequence,
                    "data_sha256": hashlib.sha256(
                        bytes(costmap.data)
                    ).hexdigest(),
                    "frame_id": costmap.header.frame_id,
                    "resolution": float(metadata.resolution),
                    "size_x": int(metadata.size_x),
                    "size_y": int(metadata.size_y),
                    "origin": {
                        "x": metadata.origin.position.x,
                        "y": metadata.origin.position.y,
                    },
                    "data": [int(value) for value in costmap.data],
                },
                "footprint": [
                    {"x": point.x, "y": point.y, "z": point.z}
                    for point in footprint
                ],
                "footprint_receipt_ros_time_sec": (
                    self.global_footprint_receipt_ros_time
                ),
                "footprint_sequence": self.global_footprint_sequence,
                "planner_diagnostic": {
                    "stamp_sec": planner_stamp,
                    "receipt_ros_time_sec": planner_receipt_ros_time,
                    "delta_sec": planner_delta,
                    "level": planner_level,
                    "message": planner_message,
                },
                "analysis": analysis,
            }
            with gzip.open(
                self._costmap_dir / raw_name, "wt", encoding="utf-8"
            ) as raw_file:
                json.dump(raw_payload, raw_file, separators=(",", ":"))
            self.costmap_snapshot_count += 1

        costmap_stamp = (
            None
            if costmap is None
            else self._stamp_seconds(costmap.header.stamp)
        )
        footprint_stamp = (
            None
            if footprint_message is None
            else self._stamp_seconds(footprint_message.header.stamp)
        )
        metadata = None if costmap is None else costmap.metadata
        row = [
            sequence,
            datetime.now(timezone.utc).isoformat(),
            f"{capture_seconds:.9f}",
            "ros_clock",
            result,
            f"{log_stamp:.9f}",
            "system_time_rosout_stamp",
            f"{probe_receipt_ros_time:.9f}",
            "" if planner_stamp is None else f"{planner_stamp:.9f}",
            (
                ""
                if planner_receipt_ros_time is None
                else f"{planner_receipt_ros_time:.9f}"
            ),
            "" if planner_delta is None else f"{planner_delta:.9f}",
            "" if planner_level is None else planner_level,
            planner_message,
            "" if pose is None else pose[0],
            "" if pose is None else pose[1],
            "" if pose is None else pose[2],
            "" if costmap_stamp is None else f"{costmap_stamp:.9f}",
            (
                ""
                if self.global_costmap_receipt_ros_time is None
                else f"{self.global_costmap_receipt_ros_time:.9f}"
            ),
            self.global_costmap_sequence,
            (
                ""
                if costmap is None
                else hashlib.sha256(bytes(costmap.data)).hexdigest()
            ),
            "" if costmap_stamp is None else capture_seconds - costmap_stamp,
            "" if costmap is None else costmap.header.frame_id,
            "" if metadata is None else metadata.resolution,
            "" if metadata is None else metadata.size_x,
            "" if metadata is None else metadata.size_y,
            "" if metadata is None else metadata.origin.position.x,
            "" if metadata is None else metadata.origin.position.y,
            "" if footprint_stamp is None else f"{footprint_stamp:.9f}",
            (
                ""
                if self.global_footprint_receipt_ros_time is None
                else f"{self.global_footprint_receipt_ros_time:.9f}"
            ),
            self.global_footprint_sequence,
            (
                ""
                if footprint_stamp is None
                else capture_seconds - footprint_stamp
            ),
            (
                ""
                if footprint_message is None
                else footprint_message.header.frame_id
            ),
            json.dumps(
                [[point.x, point.y] for point in footprint],
                separators=(",", ":"),
            ),
        ]
        for field in (
            "start_grid_x",
            "start_grid_y",
            "start_cost",
            "footprint_cells",
            "footprint_free",
            "footprint_inflated",
            "footprint_inscribed",
            "footprint_lethal",
            "footprint_unknown",
            "footprint_outside",
        ):
            row.append(analysis.get(field, ""))
        row.extend([raw_name, "ok" if not status else "|".join(status)])
        self._costmap_writer.writerow(row)
        self._costmap_file.flush()

    def _log_callback(self, message: Log) -> None:
        self.phase = phase_from_log(message.msg, self.phase)
        self.phases.add(self.phase)
        probe_result = self._probe_result(message.msg)
        stamp = float(message.stamp.sec) + float(message.stamp.nanosec) / 1e9
        receipt_ros_time = self.get_clock().now().nanoseconds / 1e9
        if "planner_server" in message.name and int(message.level) >= 30:
            self.latest_planner_diagnostic = (
                stamp,
                receipt_ros_time,
                int(message.level),
                message.msg,
            )
        if probe_result is not None:
            self._capture_probe_costmap(
                probe_result,
                stamp,
                receipt_ros_time,
            )
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
        self._costmap_file.flush()
        self._motion_file.close()
        self._event_file.close()
        self._costmap_file.close()
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
                    "- Full probe costmap snapshots: "
                    f"{self.costmap_snapshot_count}",
                    "",
                    "Raw `motion.csv`, `events.csv`, "
                    "`costmap_snapshots.csv`, compressed probe costmaps, "
                    "and process logs remain local.",
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
