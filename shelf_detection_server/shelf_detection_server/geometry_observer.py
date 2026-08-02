"""Bounded read-only observer for reflective shelf-leg geometry."""

import statistics
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from shelf_detection_server.leg_geometry import (
    LegPairMeasurement,
    detect_leg_pair,
)


class ShelfGeometryObserver(Node):
    """Collect repeated samples without creating publishers."""

    def __init__(self) -> None:
        super().__init__("shelf_geometry_observer")
        self.declare_parameter("intensity_threshold", 8000.0)
        self.declare_parameter("min_cluster_size", 2)
        self.declare_parameter("max_x_difference", 0.75)
        self.declare_parameter("min_leg_separation", 0.25)
        self.declare_parameter("sample_count", 20)
        self.declare_parameter("timeout", 10.0)

        self.measurements: List[LegPairMeasurement] = []
        self.done = False
        self.succeeded = False
        self._deadline = time.monotonic() + max(
            float(self.get_parameter("timeout").value), 0.1
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._timeout_timer = self.create_timer(0.1, self._check_timeout)
        self.get_logger().info(
            "read-only shelf geometry observer ready; no publishers created"
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        if self.done:
            return
        measurement = detect_leg_pair(
            scan,
            intensity_threshold=float(
                self.get_parameter("intensity_threshold").value
            ),
            min_cluster_size=int(
                self.get_parameter("min_cluster_size").value
            ),
            max_x_difference=float(
                self.get_parameter("max_x_difference").value
            ),
            min_leg_separation=float(
                self.get_parameter("min_leg_separation").value
            ),
        )
        if measurement is None:
            return
        self.measurements.append(measurement)
        self.get_logger().info(
            "geometry sample: "
            f"n={len(self.measurements)} "
            f"frame={measurement.frame_id} "
            f"left=({measurement.left_x:.3f},{measurement.left_y:.3f}) "
            f"right=({measurement.right_x:.3f},{measurement.right_y:.3f}) "
            f"inner_span={measurement.euclidean_separation:.3f} "
            f"center_span={measurement.center_separation:.3f} "
            f"outer_span={measurement.outer_separation:.3f}"
            f" midpoint_bearing={measurement.midpoint_bearing:.3f}"
            f" shelf_normal_yaw={measurement.shelf_normal_yaw:.3f}"
        )
        target = max(int(self.get_parameter("sample_count").value), 1)
        if len(self.measurements) >= target:
            self._report_success()

    def _report_success(self) -> None:
        separations = [
            item.euclidean_separation for item in self.measurements
        ]
        center_separations = [
            item.center_separation for item in self.measurements
        ]
        outer_separations = [
            item.outer_separation for item in self.measurements
        ]
        lateral = [item.lateral_separation for item in self.measurements]
        midpoint_x = [item.midpoint_x for item in self.measurements]
        midpoint_y = [item.midpoint_y for item in self.measurements]
        midpoint_bearings = [
            item.midpoint_bearing for item in self.measurements
        ]
        shelf_normal_yaws = [
            item.shelf_normal_yaw for item in self.measurements
        ]
        self.get_logger().info(
            "GEOMETRY_RESULT "
            f"samples={len(separations)} "
            f"frame={self.measurements[-1].frame_id} "
            f"inner_span_median={statistics.median(separations):.4f} "
            f"center_span_median="
            f"{statistics.median(center_separations):.4f} "
            f"outer_span_median="
            f"{statistics.median(outer_separations):.4f} "
            f"outer_span_min={min(outer_separations):.4f} "
            f"outer_span_max={max(outer_separations):.4f} "
            f"lateral_median={statistics.median(lateral):.4f} "
            f"midpoint_x_median={statistics.median(midpoint_x):.4f} "
            f"midpoint_y_median={statistics.median(midpoint_y):.4f} "
            f"midpoint_bearing_median="
            f"{statistics.median(midpoint_bearings):.4f} "
            f"shelf_normal_yaw_median="
            f"{statistics.median(shelf_normal_yaws):.4f}"
        )
        self.succeeded = True
        self.done = True

    def _check_timeout(self) -> None:
        if self.done or time.monotonic() < self._deadline:
            return
        self.get_logger().error(
            "geometry observation timed out: "
            f"samples={len(self.measurements)}"
        )
        self.done = True


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ShelfGeometryObserver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if node.succeeded else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
