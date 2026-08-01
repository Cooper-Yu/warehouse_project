"""Detection-only shelf service with no motion or elevator publishers."""

import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
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

        self._latest_scan: Optional[LaserScan] = None
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._service = self.create_service(
            GoToLoading, "/approach_shelf", self._handle_request
        )
        self.get_logger().info(
            "detection-only /approach_shelf ready; no cmd_vel/elevator publishers"
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        self._latest_scan = scan

    def _handle_request(
        self, request: GoToLoading.Request, response: GoToLoading.Response
    ) -> GoToLoading.Response:
        if request.attach_to_shelf:
            response.complete = False
            self.get_logger().error(
                "Rejected attach_to_shelf=true in detection-only mode"
            )
            return response

        deadline = time.monotonic() + float(
            self.get_parameter("detection_timeout").value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            target = self._detect_cart_frame()
            if target is not None:
                self._publish_cart_frame(*target)
                response.complete = True
                self.get_logger().info(
                    "complete=true: detection-only cart_frame published"
                )
                return response
            time.sleep(0.05)

        response.complete = False
        self.get_logger().error("complete=false: shelf detection timed out")
        return response

    def _detect_cart_frame(self) -> Optional[tuple]:
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
            for second in candidates[index + 1 :]:
                separation = abs(first.y - second.y)
                x_difference = abs(first.x - second.x)
                if separation < min_separation or x_difference > max_x_difference:
                    continue
                score = abs((first.y + second.y) / 2.0) + x_difference
                if score < best_score:
                    best = (first, second)
                    best_score = score
        return best

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
            self.get_logger().warning("cart_frame transform unavailable: %s" % error)
            return None

    def _publish_cart_frame(self, frame_id: str, x: float, y: float) -> None:
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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
