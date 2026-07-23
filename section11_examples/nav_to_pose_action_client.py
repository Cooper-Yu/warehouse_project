import rclpy
# Learner slice S11-C027: import GoalStatus here.
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavToPoseActionClient(Node):
    def __init__(self):
        super().__init__("nav_to_pose_action_client")

        # Learner slice S11-C009: create the NavigateToPose ActionClient here.
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self._subscription = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10,
        )

    def clicked_point_callback(self, msg):
        self.get_logger().info(
            f"Clicked point: x={msg.point.x:.3f}, y={msg.point.y:.3f}"
        )

        # Learner slice S11-C010: create an empty NavigateToPose goal here.
        goal_msg = NavigateToPose.Goal()

        # Learner slice S11-C011: set the goal pose frame here.
        goal_msg.pose.header.frame_id = 'map'

        # Learner slice S11-C012: copy the clicked point x coordinate here.
        goal_msg.pose.pose.position.x = msg.point.x

        # Learner slice S11-C014: copy the clicked point y coordinate here.
        goal_msg.pose.pose.position.y = msg.point.y

        # Learner slice S11-C015: set a valid default orientation here.
        goal_msg.pose.pose.orientation.w = 1.0

        # Learner slice S11-C016: wait for the Action Server here.
        self._action_client.wait_for_server()

        # Learner slice S11-C017: send goal_msg asynchronously here.
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Learner slice S11-C018: register goal_response_callback here.
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Expose the constructed goal for bounded local verification.
        self._last_goal_msg = goal_msg

    def goal_response_callback(self, future):
        # Learner slice S11-C020: extract the goal handle here.
        goal_handle = future.result()

        # Learner slice S11-C021: handle a rejected goal here.
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected")
            return

        # Learner slice S11-C023: request the accepted goal's final result here.
        self._get_result_future = goal_handle.get_result_async()

        # Learner slice S11-C024: register result_callback here.
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        # Learner slice S11-C025: extract the result response here.
        result_response = future.result()

        # Learner slice S11-C026: extract the final action status here.
        status = result_response.status

        # Learner slice S11-C028: handle the succeeded status here.
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Navigation succeeded")

        # Learner slice S11-C029: handle all non-success final statuses here.
        else:
            self.get_logger().warn(f"Navigation ended with status: {status}")

    def feedback_callback(self, feedback_msg):
        # Learner slice S11-C031: extract the NavigateToPose feedback here.
        feedback = feedback_msg.feedback

        # Learner slice S11-C033: extract the remaining distance here.
        distance_remaining = feedback.distance_remaining

        # Learner slice S11-C034: log the remaining distance here.
        self.get_logger().info(f"Distance remaining: {distance_remaining:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
