#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class AutoInitialPose(Node):
    """Publish one RViz-like initial pose after AMCL and scan are ready."""

    def __init__(self):
        super().__init__("auto_initial_pose")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("covariance_x", 0.25)
        self.declare_parameter("covariance_y", 0.25)
        self.declare_parameter("covariance_yaw", math.radians(15.0) ** 2)
        self.declare_parameter("timeout", 30.0)

        self._scan_received = False
        self._scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self._amcl_state = self.create_client(GetState, "/amcl/get_state")

    def _scan_callback(self, _message):
        self._scan_received = True

    def wait_until_ready(self):
        deadline = time.monotonic() + float(
            self.get_parameter("timeout").value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self._scan_received:
                continue
            if not self._amcl_state.wait_for_service(timeout_sec=0.0):
                continue
            request = GetState.Request()
            future = self._amcl_state.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if not future.done() or future.result() is None:
                continue
            if future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
                continue
            if self._publisher.get_subscription_count() == 0:
                continue
            return True
        return False

    def publish_initial_pose(self):
        x = float(self.get_parameter("initial_x").value)
        y = float(self.get_parameter("initial_y").value)
        yaw = float(self.get_parameter("initial_yaw").value)
        covariance_x = float(self.get_parameter("covariance_x").value)
        covariance_y = float(self.get_parameter("covariance_y").value)
        covariance_yaw = float(
            self.get_parameter("covariance_yaw").value
        )

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = covariance_x
        message.pose.covariance[7] = covariance_y
        message.pose.covariance[35] = covariance_yaw
        self._publisher.publish(message)
        self.get_logger().info(
            "published simulation initial pose: "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f} "
            f"covariance=({covariance_x:.3f}, {covariance_y:.3f}, "
            f"{covariance_yaw:.6f})"
        )
        # Give DDS time to deliver before this one-shot process exits.
        end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = AutoInitialPose()
    exit_code = 0
    try:
        if node.wait_until_ready():
            node.publish_initial_pose()
        else:
            node.get_logger().error(
                "auto initial pose timed out before AMCL, scan, and "
                "/initialpose subscription were ready"
            )
            exit_code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
