#!/usr/bin/env python3
"""Publish a localization-only scan with loaded shelf self-returns removed."""

import math
from typing import Iterable, Sequence, Tuple

from geometry_msgs.msg import PolygonStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


AngleSector = Tuple[float, float]


def maximum_polygon_edge(points: Sequence) -> float:
    """Return the longest closed-polygon edge, or infinity if incomplete."""
    if len(points) < 3:
        return math.inf
    return max(
        math.hypot(
            points[(index + 1) % len(points)].x - point.x,
            points[(index + 1) % len(points)].y - point.y,
        )
        for index, point in enumerate(points)
    )


def filter_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    loaded: bool,
    sectors: Iterable[AngleSector],
    maximum_self_range: float,
) -> list:
    """Return a copy; only loaded, near-range samples in measured sectors go inf."""
    output = list(ranges)
    if not loaded:
        return output
    for index, value in enumerate(output):
        if not math.isfinite(value) or value > maximum_self_range:
            continue
        angle = angle_min + index * angle_increment
        if any(lower <= angle <= upper for lower, upper in sectors):
            output[index] = math.inf
    return output


class LoadedScanFilter(Node):
    def __init__(self) -> None:
        super().__init__("loaded_scan_filter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_localization")
        self.declare_parameter("left_sector_min", 0.843)
        self.declare_parameter("left_sector_max", 2.3562)
        self.declare_parameter("right_sector_min", -2.3562)
        self.declare_parameter("right_sector_max", -1.111)
        self.declare_parameter("maximum_self_range", 0.60)
        self.declare_parameter(
            "unloaded_footprint_max_edge", 0.55
        )

        self._loaded = False
        self._lowering_seen = False
        self._sectors = (
            (
                float(self.get_parameter("right_sector_min").value),
                float(self.get_parameter("right_sector_max").value),
            ),
            (
                float(self.get_parameter("left_sector_min").value),
                float(self.get_parameter("left_sector_max").value),
            ),
        )
        self._maximum_self_range = float(
            self.get_parameter("maximum_self_range").value
        )
        self._unloaded_footprint_max_edge = float(
            self.get_parameter("unloaded_footprint_max_edge").value
        )
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(String, "/elevator_up", self._up_callback, 10)
        self.create_subscription(
            String, "/elevator_down", self._down_callback, 10
        )
        self.create_subscription(
            PolygonStamped,
            "/local_costmap/published_footprint",
            self._footprint_callback,
            10,
        )
        self.get_logger().info(
            "localization scan filter ready: unloaded pass-through; loaded "
            f"near self-returns <= {self._maximum_self_range:.2f} m removed"
        )

    def _up_callback(self, message: String) -> None:
        if message.data.strip().lower() == "up" and not self._loaded:
            self._loaded = True
            self._lowering_seen = False
            self.get_logger().warning(
                "LOADED_SCAN_FILTER_ACTIVE: AMCL self-return filtering enabled"
            )

    def _down_callback(self, message: String) -> None:
        if message.data.strip().lower() == "down" and self._loaded:
            self._lowering_seen = True
            self.get_logger().info(
                "LOADED_SCAN_FILTER_LOWERING: filtering retained until "
                "the unloaded footprint is restored"
            )

    def _footprint_callback(self, message: PolygonStamped) -> None:
        if not (self._loaded and self._lowering_seen):
            return
        maximum_edge = maximum_polygon_edge(message.polygon.points)
        if maximum_edge <= self._unloaded_footprint_max_edge:
            self._loaded = False
            self._lowering_seen = False
            self.get_logger().info(
                "LOADED_SCAN_FILTER_INACTIVE: unloaded footprint verified; "
                "AMCL raw scan pass-through restored"
            )

    def _scan_callback(self, message: LaserScan) -> None:
        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        output.ranges = filter_ranges(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            self._loaded,
            self._sectors,
            self._maximum_self_range,
        )
        output.intensities = list(message.intensities)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoadedScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
