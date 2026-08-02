"""Shelf detection and bounded C9-style stepwise attach service."""

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)
from warehouse_interfaces.srv import GoToLoading


@dataclass
class LegCandidate:
    x: float
    y: float
    low_index: int
    high_index: int
    size: int


class ShelfDetectionServer(Node):
    def __init__(self) -> None:
        super().__init__("shelf_detection_server")
        self.declare_parameter("intensity_threshold", 8000.0)
        self.declare_parameter("min_cluster_size", 2)
        self.declare_parameter("max_x_difference", 0.75)
        self.declare_parameter("min_leg_separation", 0.25)
        self.declare_parameter("detection_timeout", 3.0)
        self.declare_parameter("target_base_frame", "robot_base_footprint")
        self.declare_parameter("forward_speed", 0.10)
        self.declare_parameter("rotate_speed", 0.20)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("max_detected_yaw", 0.60)
        self.declare_parameter("lateral_yaw_gain", 0.40)
        self.declare_parameter("forward_step_distance", 0.20)
        self.declare_parameter("center_distance_tolerance", 0.20)
        self.declare_parameter("center_lateral_tolerance", 0.08)
        self.declare_parameter("final_drive_distance", 0.30)
        self.declare_parameter("movement_timeout", 45.0)
        self.declare_parameter("elevator_publish_count", 5)

        self._latest_scan: Optional[LaserScan] = None
        self._scan_sequence = 0
        self._scan_lock = threading.Lock()
        self._scan_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=self._scan_group,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._elevator_up_pub = self.create_publisher(
            String, "/elevator_up", 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._active_cart_frame = None
        self._cart_frame_expiry = 0.0
        self._cart_frame_timer = self.create_timer(
            0.1, self._republish_cart_frame
        )
        self._service = self.create_service(
            GoToLoading,
            "/approach_shelf",
            self._handle_request,
            callback_group=self._service_group,
        )
        self.get_logger().info(
            "/approach_shelf ready: detection-only false, C9-style attach true"
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = scan
            self._scan_sequence += 1

    def _handle_request(
        self, request: GoToLoading.Request, response: GoToLoading.Response
    ) -> GoToLoading.Response:
        target = self._wait_for_cart_frame()
        if target is None:
            self._publish_stop()
            response.complete = False
            self.get_logger().error(
                "complete=false: shelf detection timed out"
            )
            return response

        self._publish_cart_frame(*target)
        if not request.attach_to_shelf:
            response.complete = True
            self.get_logger().info(
                "complete=true: detection-only cart_frame published"
            )
            return response

        response.complete = self._perform_stepwise_attach(target)
        if response.complete:
            self.get_logger().info(
                "complete=true: stepwise attach finished and elevator-up sent"
            )
        else:
            self._publish_stop()
            self.get_logger().error("complete=false: stepwise attach failed")
        return response

    def _wait_for_cart_frame(
        self, after_sequence: Optional[int] = None
    ) -> Optional[tuple]:
        deadline = time.monotonic() + float(
            self.get_parameter("detection_timeout").value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                after_sequence is not None
                and self._current_scan_sequence() <= after_sequence
            ):
                time.sleep(0.05)
                continue
            target = self._detect_cart_frame()
            if target is not None:
                return target
            time.sleep(0.05)
        return None

    def _current_scan_sequence(self) -> int:
        with self._scan_lock:
            return self._scan_sequence

    def _detect_cart_frame(self) -> Optional[tuple]:
        with self._scan_lock:
            scan = self._latest_scan
        if scan is None or not scan.ranges or not scan.intensities:
            return None

        threshold = float(self.get_parameter("intensity_threshold").value)
        min_size = int(self.get_parameter("min_cluster_size").value)
        count = min(len(scan.ranges), len(scan.intensities))
        clusters: List[List[int]] = []
        current: List[int] = []
        for index in range(count):
            intensity = scan.intensities[index]
            distance = scan.ranges[index]
            valid = (
                math.isfinite(intensity)
                and intensity >= threshold
                and math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
            )
            if valid:
                current.append(index)
            else:
                if len(current) >= min_size:
                    clusters.append(current)
                current = []
        if len(current) >= min_size:
            clusters.append(current)

        candidates = []
        for cluster in clusters:
            candidate = self._candidate(scan, cluster)
            if candidate.x > 0.0:
                candidates.append(candidate)

        pair = self._best_pair(candidates)
        if pair is None:
            return None
        left, right = pair
        left_index = left.high_index if left.y < right.y else left.low_index
        right_index = right.low_index if left.y < right.y else right.high_index
        left = self._candidate_at(scan, left_index, left)
        right = self._candidate_at(scan, right_index, right)
        laser_x = (left.x + right.x) / 2.0
        laser_y = (left.y + right.y) / 2.0
        return self._transform_to_base(laser_x, laser_y, scan.header.frame_id)

    def _candidate(self, scan: LaserScan, cluster: List[int]) -> LegCandidate:
        return self._candidate_at(
            scan,
            cluster[len(cluster) // 2],
            LegCandidate(0.0, 0.0, cluster[0], cluster[-1], len(cluster)),
        )

    @staticmethod
    def _candidate_at(
        scan: LaserScan, index: int, source: LegCandidate
    ) -> LegCandidate:
        angle = scan.angle_min + index * scan.angle_increment
        distance = scan.ranges[index]
        return LegCandidate(
            distance * math.cos(angle),
            distance * math.sin(angle),
            source.low_index,
            source.high_index,
            source.size,
        )

    def _best_pair(self, candidates: List[LegCandidate]):
        min_separation = float(self.get_parameter("min_leg_separation").value)
        max_x_difference = float(self.get_parameter("max_x_difference").value)
        best = None
        best_score = math.inf
        for index, first in enumerate(candidates):
            for second in candidates[index + 1:]:
                separation = abs(first.y - second.y)
                x_difference = abs(first.x - second.x)
                if (
                    separation < min_separation
                    or x_difference > max_x_difference
                ):
                    continue
                score = abs((first.y + second.y) / 2.0) + x_difference
                if score < best_score:
                    best = (first, second)
                    best_score = score
        return best

    def _perform_stepwise_attach(self, initial_target: tuple) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter("movement_timeout").value
        )
        target = initial_target
        step = 0
        while rclpy.ok() and time.monotonic() < deadline:
            frame_id, x, y = target
            self._publish_cart_frame(frame_id, x, y)
            yaw = math.atan2(y, x)
            max_yaw = float(self.get_parameter("max_detected_yaw").value)
            if abs(yaw) > max_yaw:
                self.get_logger().error(
                    "attach rejected: detected yaw "
                    f"{yaw:.3f} exceeds {max_yaw:.3f}"
                )
                return False

            center_x = float(
                self.get_parameter("center_distance_tolerance").value
            )
            center_y = float(
                self.get_parameter("center_lateral_tolerance").value
            )
            self.get_logger().info(
                "stepwise attach sample: "
                f"step={step} x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
            )
            if x <= center_x and abs(y) <= center_y:
                break

            yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
            if abs(yaw) >= yaw_tolerance:
                scan_before_motion = self._current_scan_sequence()
                correction = yaw * float(
                    self.get_parameter("lateral_yaw_gain").value
                )
                if not self._rotate_open_loop(correction, deadline):
                    return False
                target = self._wait_for_cart_frame(scan_before_motion)
                if target is None:
                    self.get_logger().error(
                        "attach stopped: cart_frame unavailable after "
                        "bounded yaw correction"
                    )
                    return False
                step += 1
                continue

            drive_distance = min(
                float(self.get_parameter("forward_step_distance").value),
                max(x - center_x, 0.0),
            )
            if drive_distance <= 0.0:
                self.get_logger().error(
                    "attach stopped: lateral error remained after "
                    "center distance"
                )
                return False
            scan_before_motion = self._current_scan_sequence()
            if not self._drive_forward_open_loop(drive_distance, deadline):
                return False

            target = self._wait_for_cart_frame(scan_before_motion)
            if target is None:
                self.get_logger().error(
                    "attach stopped: cart_frame unavailable after bounded step"
                )
                return False
            step += 1

        if time.monotonic() >= deadline:
            self.get_logger().error("attach stopped: movement timeout")
            return False

        final_distance = float(
            self.get_parameter("final_drive_distance").value
        )
        if not self._drive_forward_open_loop(final_distance, deadline):
            return False
        self._publish_stop()
        self._publish_elevator_up()
        return True

    def _rotate_open_loop(self, yaw: float, deadline: float) -> bool:
        speed = float(self.get_parameter("rotate_speed").value)
        if speed <= 0.0 or time.monotonic() >= deadline:
            return False
        duration = abs(yaw) / speed
        if time.monotonic() + duration > deadline:
            return False
        command = Twist()
        command.angular.z = math.copysign(speed, yaw)
        self.get_logger().info(
            f"bounded yaw correction: yaw={yaw:.3f} duration={duration:.3f}"
        )
        try:
            end = time.monotonic() + duration
            while rclpy.ok() and time.monotonic() < end:
                self._cmd_vel_pub.publish(command)
                time.sleep(0.05)
        finally:
            self._publish_stop()
        return True

    def _drive_forward_open_loop(
        self, distance: float, deadline: float
    ) -> bool:
        speed = float(self.get_parameter("forward_speed").value)
        if distance <= 0.0 or speed <= 0.0 or time.monotonic() >= deadline:
            return False
        duration = distance / speed
        if time.monotonic() + duration > deadline:
            return False
        command = Twist()
        command.linear.x = speed
        self.get_logger().info(
            "bounded forward step: "
            f"distance={distance:.3f} duration={duration:.3f}"
        )
        try:
            end = time.monotonic() + duration
            while rclpy.ok() and time.monotonic() < end:
                self._cmd_vel_pub.publish(command)
                time.sleep(0.05)
        finally:
            self._publish_stop()
        return True

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_elevator_up(self) -> None:
        message = String()
        message.data = "up"
        count = max(
            1, int(self.get_parameter("elevator_publish_count").value)
        )
        for _ in range(count):
            self._elevator_up_pub.publish(message)
            time.sleep(0.1)
        self.get_logger().warning(
            f"published elevator-up {count} times after successful final push"
        )

    def _transform_to_base(self, x: float, y: float, source_frame: str):
        target_frame = str(self.get_parameter("target_base_frame").value)
        if not source_frame:
            return None
        if source_frame == target_frame:
            return target_frame, x, y
        point = PointStamped()
        point.header.frame_id = source_frame
        point.point.x = x
        point.point.y = y
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            transformed = do_transform_point(point, transform)
            return target_frame, transformed.point.x, transformed.point.y
        except TransformException as error:
            self.get_logger().warning(
                "cart_frame transform unavailable: %s" % error
            )
            return None

    def _publish_cart_frame(self, frame_id: str, x: float, y: float) -> None:
        self._active_cart_frame = (frame_id, x, y)
        self._cart_frame_expiry = time.monotonic() + 10.0
        self._broadcast_cart_frame(frame_id, x, y)

    def _republish_cart_frame(self) -> None:
        if self._active_cart_frame is None:
            return
        if time.monotonic() >= self._cart_frame_expiry:
            self._active_cart_frame = None
            return
        self._broadcast_cart_frame(*self._active_cart_frame)

    def _broadcast_cart_frame(self, frame_id: str, x: float, y: float) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = frame_id
        transform.child_frame_id = "cart_frame"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShelfDetectionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
