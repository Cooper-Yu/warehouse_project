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


def c9_yaw_correction_enabled(
    step: int,
    x: float,
    correction_steps: int,
    min_distance: float,
) -> bool:
    return step < correction_steps and x > min_distance


def c9_center_lock_enabled(
    step: int,
    x: float,
    min_steps: int,
    lock_distance: float,
) -> bool:
    return step >= min_steps and x <= lock_distance


def c9_locked_drive_distance(x: float, scale: float) -> float:
    return max(x, 0.0) * max(scale, 0.0)


def c9_pre_lock_yaw_correction(
    yaw: float, gain: float, max_abs_correction: float
) -> float:
    limit = max(max_abs_correction, 0.0)
    return max(-limit, min(yaw * gain, limit))


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
        self.declare_parameter("center_lock_distance", 0.35)
        self.declare_parameter("center_lock_min_steps", 2)
        self.declare_parameter("center_drive_scale", 1.0)
        self.declare_parameter("pre_lock_max_yaw_correction", 0.08)
        self.declare_parameter("yaw_correction_steps", 3)
        self.declare_parameter("min_yaw_correction_distance", 0.55)
        self.declare_parameter("cart_frame_retry_count", 6)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("odom_lookup_timeout", 1.0)
        self.declare_parameter("measured_drive_timeout_scale", 3.0)
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
        self,
        after_sequence: Optional[int] = None,
        timeout_seconds: Optional[float] = None,
    ) -> Optional[tuple]:
        timeout = (
            float(self.get_parameter("detection_timeout").value)
            if timeout_seconds is None
            else timeout_seconds
        )
        deadline = time.monotonic() + timeout
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
        center_approach_complete = False
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
                center_approach_complete = True
                break

            center_lock_distance = float(
                self.get_parameter("center_lock_distance").value
            )
            center_lock_min_steps = int(
                self.get_parameter("center_lock_min_steps").value
            )
            if c9_center_lock_enabled(
                step,
                x,
                center_lock_min_steps,
                center_lock_distance,
            ):
                locked_distance = c9_locked_drive_distance(
                    x,
                    float(self.get_parameter("center_drive_scale").value),
                )
                self.get_logger().warning(
                    "locking final center approach: "
                    f"step={step} x={x:.3f} y={y:.3f} "
                    f"locked_distance={locked_distance:.3f}; "
                    "close-range cart_frame re-detection is skipped"
                )
                if not self._apply_pre_lock_yaw(yaw, deadline):
                    return False
                if not self._drive_forward_measured(
                    locked_distance, deadline
                ):
                    return False
                center_approach_complete = True
                break

            yaw_correction_steps = int(
                self.get_parameter("yaw_correction_steps").value
            )
            min_yaw_distance = float(
                self.get_parameter("min_yaw_correction_distance").value
            )
            yaw_correction_enabled = c9_yaw_correction_enabled(
                step,
                x,
                yaw_correction_steps,
                min_yaw_distance,
            )
            yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
            correction = (
                yaw * float(self.get_parameter("lateral_yaw_gain").value)
                if yaw_correction_enabled
                else 0.0
            )
            scan_before_motion = self._current_scan_sequence()
            if abs(correction) >= yaw_tolerance:
                if not self._rotate_open_loop(correction, deadline):
                    return False
            elif abs(yaw) >= yaw_tolerance:
                self.get_logger().info(
                    "late yaw correction disabled by C9 gate: "
                    f"step={step} x={x:.3f} yaw={yaw:.3f}"
                )

            drive_distance = min(
                float(self.get_parameter("forward_step_distance").value),
                max(x, 0.0),
            )
            if drive_distance <= 0.0:
                self.get_logger().error(
                    "attach stopped: lateral error remained after "
                    "center distance"
                )
                return False
            if not self._drive_forward_measured(drive_distance, deadline):
                return False

            target = self._recover_cart_frame_after_motion(
                scan_before_motion, deadline
            )
            if target is None:
                estimated_remaining = max(x - drive_distance, 0.0)
                if c9_center_lock_enabled(
                    step,
                    estimated_remaining,
                    center_lock_min_steps,
                    center_lock_distance,
                ):
                    locked_distance = c9_locked_drive_distance(
                        estimated_remaining,
                        float(
                            self.get_parameter("center_drive_scale").value
                        ),
                    )
                    self.get_logger().warning(
                        "close-range detection recovery exhausted; "
                        "locking from estimated remaining distance: "
                        f"previous_x={x:.3f} "
                        f"last_drive={drive_distance:.3f} "
                        f"estimated_remaining={estimated_remaining:.3f} "
                        f"locked_distance={locked_distance:.3f}"
                    )
                    if not self._apply_pre_lock_yaw(yaw, deadline):
                        return False
                    if locked_distance > 0.0 and not (
                        self._drive_forward_measured(
                            locked_distance, deadline
                        )
                    ):
                        return False
                    center_approach_complete = True
                    break
                self.get_logger().error(
                    "attach stopped: cart_frame recovery exhausted after "
                    "bounded step"
                )
                return False
            step += 1

        if time.monotonic() >= deadline:
            self.get_logger().error("attach stopped: movement timeout")
            return False
        if not center_approach_complete:
            self.get_logger().error(
                "attach stopped before reaching or locking cart center"
            )
            return False

        final_distance = float(
            self.get_parameter("final_drive_distance").value
        )
        if not self._drive_forward_measured(final_distance, deadline):
            return False
        self._publish_stop()
        self._publish_elevator_up()
        return True

    def _recover_cart_frame_after_motion(
        self, after_sequence: int, deadline: float
    ) -> Optional[tuple]:
        target = self._wait_for_cart_frame(
            after_sequence, timeout_seconds=0.5
        )
        if target is not None:
            return target

        retry_count = max(
            0, int(self.get_parameter("cart_frame_retry_count").value)
        )
        for retry in range(1, retry_count + 1):
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.3)
            target = self._wait_for_cart_frame(
                after_sequence, timeout_seconds=0.5
            )
            if target is not None:
                self.get_logger().info(
                    "cart_frame recovery accepted: "
                    f"retry={retry}/{retry_count}"
                )
                return target
            self.get_logger().warning(
                "cart_frame recovery failed: "
                f"retry={retry}/{retry_count}"
            )
        return None

    def _apply_pre_lock_yaw(self, yaw: float, deadline: float) -> bool:
        correction = c9_pre_lock_yaw_correction(
            yaw,
            float(self.get_parameter("lateral_yaw_gain").value),
            float(
                self.get_parameter("pre_lock_max_yaw_correction").value
            ),
        )
        tolerance = float(self.get_parameter("yaw_tolerance").value)
        if abs(correction) < tolerance:
            self.get_logger().info(
                "pre-lock yaw correction skipped: "
                f"raw_yaw={yaw:.3f} correction={correction:.3f}"
            )
            return True
        self.get_logger().warning(
            "applying one pre-lock yaw correction from last trusted cart: "
            f"raw_yaw={yaw:.3f} correction={correction:.3f}"
        )
        return self._rotate_open_loop(correction, deadline)

    def _lookup_odom_xy(self) -> Optional[tuple]:
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("target_base_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except TransformException:
            return None

    def _wait_for_odom_xy(self, deadline: float) -> Optional[tuple]:
        lookup_timeout = max(
            0.0, float(self.get_parameter("odom_lookup_timeout").value)
        )
        lookup_deadline = min(deadline, time.monotonic() + lookup_timeout)
        while rclpy.ok() and time.monotonic() < lookup_deadline:
            position = self._lookup_odom_xy()
            if position is not None:
                return position
            time.sleep(0.05)
        return None

    def _drive_forward_measured(
        self, distance: float, deadline: float
    ) -> bool:
        speed = float(self.get_parameter("forward_speed").value)
        if distance <= 0.0:
            self._publish_stop()
            return True
        if speed <= 0.0 or time.monotonic() >= deadline:
            return False

        start = self._wait_for_odom_xy(deadline)
        if start is None:
            self.get_logger().error(
                "measured forward drive rejected: odom TF unavailable"
            )
            return False

        timeout_scale = max(
            1.0,
            float(
                self.get_parameter("measured_drive_timeout_scale").value
            ),
        )
        motion_deadline = min(
            deadline,
            time.monotonic() + max(2.0, distance / speed * timeout_scale),
        )
        lookup_timeout = max(
            0.0, float(self.get_parameter("odom_lookup_timeout").value)
        )
        last_odom_time = time.monotonic()
        traveled = 0.0
        command = Twist()
        command.linear.x = speed
        self.get_logger().info(
            "measured forward drive started: "
            f"target_distance={distance:.3f} speed={speed:.3f}"
        )
        try:
            while rclpy.ok() and time.monotonic() < motion_deadline:
                position = self._lookup_odom_xy()
                if position is not None:
                    last_odom_time = time.monotonic()
                    traveled = math.hypot(
                        position[0] - start[0], position[1] - start[1]
                    )
                    if traveled >= distance:
                        self.get_logger().info(
                            "measured forward drive complete: "
                            f"target={distance:.3f} "
                            f"traveled={traveled:.3f}"
                        )
                        return True
                elif time.monotonic() - last_odom_time >= lookup_timeout:
                    self.get_logger().error(
                        "measured forward drive stopped: odom TF stale"
                    )
                    return False
                self._cmd_vel_pub.publish(command)
                time.sleep(0.05)
        finally:
            self._publish_stop()

        self.get_logger().error(
            "measured forward drive stopped before target: "
            f"target={distance:.3f} traveled={traveled:.3f}"
        )
        return False

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
