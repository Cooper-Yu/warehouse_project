"""Shelf detection and bounded C9-style stepwise attach service."""

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
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

from shelf_detection_server.leg_geometry import (
    LegPairMeasurement,
    detect_leg_pair,
    normalize_angle,
    shelf_normal_yaw,
)


def c9_center_lock_enabled(
    step: int,
    x: float,
    min_steps: int,
    lock_distance: float,
) -> bool:
    return step >= min_steps and x <= lock_distance


def c9_locked_drive_distance(x: float, scale: float) -> float:
    return max(x, 0.0) * max(scale, 0.0)


def bounded_yaw_correction(
    yaw: float, gain: float, max_abs_correction: float
) -> float:
    limit = max(max_abs_correction, 0.0)
    return max(-limit, min(yaw * gain, limit))


def alignment_yaw_command(
    yaw: float,
    coarse_gain: float,
    max_abs_correction: float,
    coarse_speed: float,
    fine_threshold: float,
    fine_gain: float,
    fine_speed: float,
) -> tuple:
    """Return damped correction, positive speed, and regime name."""
    fine = abs(yaw) <= max(fine_threshold, 0.0)
    gain = fine_gain if fine else coarse_gain
    speed = fine_speed if fine else coarse_speed
    return (
        bounded_yaw_correction(yaw, gain, max_abs_correction),
        speed,
        "fine" if fine else "coarse",
    )


def shelf_heading_aligned(yaw: float, tolerance: float) -> bool:
    return abs(yaw) <= max(tolerance, 0.0)


def shelf_staging_error(
    midpoint_x: float,
    midpoint_y: float,
    shelf_heading: float,
    standoff_distance: float,
) -> tuple:
    """Return the base-frame displacement to a centered staging pose."""
    standoff = max(standoff_distance, 0.0)
    return (
        midpoint_x - standoff * math.cos(shelf_heading),
        midpoint_y - standoff * math.sin(shelf_heading),
    )


def staging_motion_command(
    error_x: float,
    error_y: float,
    max_forward_yaw: float,
    max_reverse_distance: float,
    max_reverse_yaw: float,
) -> Optional[tuple]:
    """Return bounded yaw, signed distance, and staging motion mode."""
    distance = math.hypot(error_x, error_y)
    if distance <= 0.0:
        return 0.0, 0.0, "forward"

    forward_yaw = math.atan2(error_y, error_x)
    if abs(forward_yaw) <= max(max_forward_yaw, 0.0):
        return forward_yaw, distance, "forward"

    reverse_yaw = normalize_angle(forward_yaw + math.pi)
    if (
        distance <= max(max_reverse_distance, 0.0)
        and abs(reverse_yaw) <= max(max_reverse_yaw, 0.0)
    ):
        return reverse_yaw, -distance, "reverse"
    return None


def planar_yaw_from_quaternion(rotation) -> float:
    """Return planar yaw from a geometry quaternion."""
    sin_yaw = 2.0 * (
        rotation.w * rotation.z + rotation.x * rotation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class ShelfDetectionServer(Node):
    def __init__(self) -> None:
        super().__init__("shelf_detection_server")
        self.declare_parameter("intensity_threshold", 8000.0)
        self.declare_parameter("min_cluster_size", 2)
        self.declare_parameter("max_x_difference", 0.75)
        self.declare_parameter("min_leg_separation", 0.25)
        self.declare_parameter("detection_timeout", 3.0)
        self.declare_parameter("staging_only", False)
        self.declare_parameter("entry_only", False)
        self.declare_parameter("entry_refine_only", False)
        self.declare_parameter(
            "entry_refine_required_completed_distance", 0.303
        )
        self.declare_parameter(
            "entry_refine_confirmed_completed_distance", 0.0
        )
        self.declare_parameter("entry_refine_confirmation_tolerance", 0.01)
        self.declare_parameter("entry_refine_distance", 0.47)
        self.declare_parameter("entry_refine_max_distance", 0.50)
        self.declare_parameter("entry_refine_speed", 0.03)
        self.declare_parameter("entry_refine_timeout", 25.0)
        self.declare_parameter("target_base_frame", "robot_base_footprint")
        self.declare_parameter("forward_speed", 0.10)
        self.declare_parameter("rotate_speed", 0.20)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("max_detected_yaw", 0.60)
        self.declare_parameter("alignment_heading_gain", 1.0)
        self.declare_parameter("alignment_max_yaw_correction", 0.40)
        self.declare_parameter("alignment_fine_yaw_threshold", 0.20)
        self.declare_parameter("alignment_fine_heading_gain", 0.50)
        self.declare_parameter("alignment_fine_rotate_speed", 0.05)
        self.declare_parameter("alignment_standoff_distance", 1.00)
        self.declare_parameter("alignment_position_tolerance", 0.08)
        self.declare_parameter("alignment_retry_count", 6)
        self.declare_parameter("alignment_max_drive_distance", 0.75)
        self.declare_parameter("alignment_max_travel_yaw", 1.20)
        self.declare_parameter("alignment_short_drive_distance", 0.20)
        self.declare_parameter("alignment_short_forward_speed", 0.05)
        self.declare_parameter("alignment_max_reverse_distance", 0.15)
        self.declare_parameter("alignment_max_reverse_yaw", 1.20)
        self.declare_parameter("alignment_reverse_speed", 0.03)
        self.declare_parameter("forward_step_distance", 0.20)
        self.declare_parameter("center_distance_tolerance", 0.20)
        self.declare_parameter("center_lateral_tolerance", 0.08)
        self.declare_parameter("center_lock_distance", 0.35)
        self.declare_parameter("center_lock_min_steps", 2)
        self.declare_parameter("center_drive_scale", 1.0)
        self.declare_parameter("cart_frame_retry_count", 6)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("odom_lookup_timeout", 1.0)
        self.declare_parameter("measured_drive_timeout_scale", 3.0)
        self.declare_parameter("measured_rotation_timeout_scale", 3.0)
        self.declare_parameter("measured_yaw_tolerance", 0.01)
        self.declare_parameter("alignment_settle_timeout", 2.0)
        self.declare_parameter("alignment_settle_sample_count", 3)
        self.declare_parameter("alignment_settle_yaw_tolerance", 0.01)
        self.declare_parameter("alignment_required_consecutive_samples", 2)
        self.declare_parameter("entry_odom_yaw_tolerance", 0.03)
        # Simulation shelf is approximately square: 0.7406 m / 2.
        # Real and non-square shelves must override this calibrated value.
        self.declare_parameter("final_drive_distance", 0.3703)
        # Keep the full maneuver bounded while allowing one slower short
        # staging correction before constrained shelf entry.
        self.declare_parameter("movement_timeout", 75.0)
        self.declare_parameter("elevator_publish_count", 5)

        self._latest_scan: Optional[LaserScan] = None
        self._scan_sequence = 0
        self._scan_lock = threading.Lock()
        self._latest_odom_yaw_sample: Optional[tuple] = None
        self._odom_lock = threading.Lock()
        self._scan_group = MutuallyExclusiveCallbackGroup()
        self._odom_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=self._scan_group,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
            callback_group=self._odom_group,
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
            "/approach_shelf ready: detection-only false, safe-standoff "
            "shelf-normal attach true"
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = scan
            self._scan_sequence += 1

    def _odom_callback(self, odom: Odometry) -> None:
        stamp = odom.header.stamp
        stamp_nanoseconds = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        )
        yaw = planar_yaw_from_quaternion(odom.pose.pose.orientation)
        with self._odom_lock:
            self._latest_odom_yaw_sample = (stamp_nanoseconds, yaw)

    def _handle_request(
        self, request: GoToLoading.Request, response: GoToLoading.Response
    ) -> GoToLoading.Response:
        staging_only = bool(self.get_parameter("staging_only").value)
        entry_only = bool(self.get_parameter("entry_only").value)
        entry_refine_only = bool(
            self.get_parameter("entry_refine_only").value
        )
        if sum((staging_only, entry_only, entry_refine_only)) > 1:
            self._publish_stop()
            response.complete = False
            self.get_logger().error(
                "complete=false: staging_only, entry_only, and "
                "entry_refine_only are mutually exclusive"
            )
            return response

        if entry_refine_only:
            if not request.attach_to_shelf:
                self._publish_stop()
                response.complete = False
                self.get_logger().error(
                    "complete=false: entry-refine-only requires explicit "
                    "attach_to_shelf=true confirmation"
                )
                return response
            response.complete = self._perform_entry_refine_only()
            return response

        target = self._wait_for_cart_frame()
        if target is None:
            self._publish_stop()
            response.complete = False
            self.get_logger().error(
                "complete=false: shelf detection timed out"
            )
            return response

        self._publish_cart_frame(*target[:3])
        if not request.attach_to_shelf:
            _, x, y, shelf_heading = target
            self.get_logger().info(
                "heading observation: mode=detection-only "
                f"midpoint_bearing={math.atan2(y, x):.3f} "
                f"shelf_normal_yaw={shelf_heading:.3f}"
            )
            response.complete = True
            self.get_logger().info(
                "complete=true: detection-only cart_frame published"
            )
            return response

        if staging_only:
            response.complete = self._perform_staging_only(target)
            return response

        if entry_only:
            response.complete = self._perform_entry_only(target)
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

    def _perform_staging_only(self, initial_target: tuple) -> bool:
        """Align at safe standoff, then stop before entry or elevator."""
        deadline = time.monotonic() + float(
            self.get_parameter("movement_timeout").value
        )
        alignment = self._align_at_safe_standoff(initial_target, deadline)
        self._publish_stop()
        if alignment is None:
            self.get_logger().error(
                "staging-only stopped: safe-standoff alignment failed"
            )
            return False

        target, _accepted_odom_yaw = alignment
        frame_id, x, y, shelf_heading = target
        self._publish_cart_frame(frame_id, x, y)
        self.get_logger().info(
            "complete=true: staging-only safe-standoff accepted; "
            f"x={x:.3f} y={y:.3f} "
            f"shelf_normal_yaw={shelf_heading:.3f}; "
            "no shelf entry or elevator command was issued"
        )
        return True

    def _perform_entry_refine_only(self) -> bool:
        """Finish the confirmed partial entry by bounded odom distance."""
        required_completed = max(
            0.0,
            float(
                self.get_parameter(
                    "entry_refine_required_completed_distance"
                ).value
            ),
        )
        confirmed_completed = max(
            0.0,
            float(
                self.get_parameter(
                    "entry_refine_confirmed_completed_distance"
                ).value
            ),
        )
        confirmation_tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "entry_refine_confirmation_tolerance"
                ).value
            ),
        )
        if abs(confirmed_completed - required_completed) > (
            confirmation_tolerance
        ):
            self._publish_stop()
            self.get_logger().error(
                "entry-refine-only rejected: completed-distance "
                f"confirmation={confirmed_completed:.3f} required="
                f"{required_completed:.3f} tolerance="
                f"{confirmation_tolerance:.3f}"
            )
            return False

        distance = max(
            0.0,
            float(self.get_parameter("entry_refine_distance").value),
        )
        max_distance = max(
            0.0,
            float(self.get_parameter("entry_refine_max_distance").value),
        )
        speed = max(
            0.0,
            float(self.get_parameter("entry_refine_speed").value),
        )
        if distance <= 0.0 or distance > max_distance or speed <= 0.0:
            self._publish_stop()
            self.get_logger().error(
                "entry-refine-only rejected: invalid bounded motion "
                f"distance={distance:.3f}/{max_distance:.3f} "
                f"speed={speed:.3f}"
            )
            return False

        deadline = time.monotonic() + max(
            0.0,
            float(self.get_parameter("entry_refine_timeout").value),
        )
        accepted_odom_yaw = self._wait_for_stable_odom_yaw(deadline)
        if accepted_odom_yaw is None:
            self._publish_stop()
            self.get_logger().error(
                "entry-refine-only rejected: stopped odom yaw unavailable"
            )
            return False

        self.get_logger().warning(
            "entry-refine-only started from confirmed partial entry: "
            f"completed={confirmed_completed:.3f} "
            f"remaining_target={distance:.3f} speed={speed:.3f} "
            f"accepted_odom_yaw={accepted_odom_yaw:.3f}"
        )
        if not self._drive_forward_measured(distance, deadline, speed):
            self._publish_stop()
            return False
        if not self._accepted_odom_heading_ok(
            accepted_odom_yaw, deadline
        ):
            self._publish_stop()
            return False

        self._publish_stop()
        self.get_logger().info(
            "complete=true: entry-refine-only bounded cart-center "
            "distance complete; stopped before final push and elevator"
        )
        return True

    def _perform_entry_only(self, initial_target: tuple) -> bool:
        """Approach the cart center, then stop before final push or lift."""
        complete = self._perform_stepwise_attach(
            initial_target,
            stop_before_final_push=True,
            require_standoff_observation_only=True,
        )
        self._publish_stop()
        if not complete:
            self.get_logger().error(
                "entry-only stopped before cart-center acceptance"
            )
            return False
        self.get_logger().info(
            "complete=true: entry-only cart-center accepted; "
            "final push and elevator command were not issued"
        )
        return True

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
            return None
        return self._transform_detection_to_base(measurement)

    def _perform_stepwise_attach(
        self,
        initial_target: tuple,
        stop_before_final_push: bool = False,
        require_standoff_observation_only: bool = False,
    ) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter("movement_timeout").value
        )
        if require_standoff_observation_only:
            alignment = self._verify_safe_standoff_without_motion(
                initial_target, deadline
            )
        else:
            alignment = self._align_at_safe_standoff(
                initial_target, deadline
            )
        if alignment is None:
            self.get_logger().error(
                "attach stopped: safe-standoff shelf alignment failed"
            )
            return False
        target, accepted_odom_yaw = alignment
        step = 0
        center_approach_complete = False
        while rclpy.ok() and time.monotonic() < deadline:
            frame_id, x, y, shelf_heading = target
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
                f"step={step} x={x:.3f} y={y:.3f} "
                f"midpoint_bearing={yaw:.3f} "
                f"shelf_normal_yaw={shelf_heading:.3f}"
            )
            if not self._accepted_odom_heading_ok(
                accepted_odom_yaw, deadline
            ):
                return False
            if abs(y) > center_y:
                self.get_logger().error(
                    "attach stopped: midpoint left the centered corridor; "
                    f"y={y:.3f} tolerance={center_y:.3f}"
                )
                return False
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
                if not self._drive_forward_measured(
                    locked_distance, deadline
                ):
                    return False
                if not self._accepted_odom_heading_ok(
                    accepted_odom_yaw, deadline
                ):
                    return False
                center_approach_complete = True
                break

            scan_before_motion = self._current_scan_sequence()
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
            if not self._accepted_odom_heading_ok(
                accepted_odom_yaw, deadline
            ):
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
                    if locked_distance > 0.0 and not (
                        self._drive_forward_measured(
                            locked_distance, deadline
                        )
                    ):
                        return False
                    if not self._accepted_odom_heading_ok(
                        accepted_odom_yaw, deadline
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

        if stop_before_final_push:
            self._publish_stop()
            self.get_logger().info(
                "entry-only boundary reached: cart-center approach "
                "complete; stopped before final push and elevator"
            )
            return True

        final_distance = float(
            self.get_parameter("final_drive_distance").value
        )
        if not self._drive_forward_measured(final_distance, deadline):
            return False
        if not self._accepted_odom_heading_ok(
            accepted_odom_yaw, deadline
        ):
            return False
        self._publish_stop()
        self._publish_elevator_up()
        return True

    def _verify_safe_standoff_without_motion(
        self, initial_target: tuple, deadline: float
    ) -> Optional[tuple]:
        """Require fresh consecutive staging acceptance without correction."""
        target = initial_target
        standoff = max(
            0.0,
            float(self.get_parameter("alignment_standoff_distance").value),
        )
        position_tolerance = max(
            0.0,
            float(self.get_parameter("alignment_position_tolerance").value),
        )
        yaw_tolerance = max(
            0.0, float(self.get_parameter("yaw_tolerance").value)
        )
        required_samples = max(
            1,
            int(
                self.get_parameter(
                    "alignment_required_consecutive_samples"
                ).value
            ),
        )

        for sample in range(1, required_samples + 1):
            if time.monotonic() >= deadline:
                break
            frame_id, x, y, shelf_heading = target
            error_x, error_y = shelf_staging_error(
                x, y, shelf_heading, standoff
            )
            position_error = math.hypot(error_x, error_y)
            self.get_logger().info(
                "entry-only standoff sample: "
                f"consecutive={sample}/{required_samples} "
                f"position_error={position_error:.3f} "
                f"shelf_normal_yaw={shelf_heading:.3f}"
            )
            if (
                position_error > position_tolerance
                or not shelf_heading_aligned(
                    shelf_heading, yaw_tolerance
                )
            ):
                self._publish_stop()
                self.get_logger().error(
                    "entry-only rejected: fresh safe-standoff geometry "
                    "is outside position or heading tolerance"
                )
                return None
            accepted_odom_yaw = self._wait_for_stable_odom_yaw(deadline)
            if accepted_odom_yaw is None:
                self._publish_stop()
                self.get_logger().error(
                    "entry-only rejected: accepted odom yaw did not settle"
                )
                return None
            if sample >= required_samples:
                self.get_logger().info(
                    "entry-only safe-standoff accepted without motion: "
                    f"position_error={position_error:.3f} "
                    f"shelf_normal_yaw={shelf_heading:.3f} "
                    f"accepted_odom_yaw={accepted_odom_yaw:.3f}"
                )
                return target, accepted_odom_yaw

            scan_before_observation = self._current_scan_sequence()
            target = self._recover_cart_frame_after_motion(
                scan_before_observation, deadline
            )
            if target is None:
                self._publish_stop()
                self.get_logger().error(
                    "entry-only rejected: fresh safe-standoff observation "
                    "unavailable"
                )
                return None

        self._publish_stop()
        self.get_logger().error(
            "entry-only rejected before consecutive safe-standoff acceptance"
        )
        return None

    def _align_at_safe_standoff(
        self, initial_target: tuple, deadline: float
    ) -> Optional[tuple]:
        target = initial_target
        retry_count = max(
            1, int(self.get_parameter("alignment_retry_count").value)
        )
        standoff = max(
            0.0,
            float(
                self.get_parameter("alignment_standoff_distance").value
            ),
        )
        position_tolerance = max(
            0.0,
            float(
                self.get_parameter("alignment_position_tolerance").value
            ),
        )
        max_drive = max(
            0.0,
            float(
                self.get_parameter("alignment_max_drive_distance").value
            ),
        )
        max_travel_yaw = max(
            0.0,
            float(
                self.get_parameter("alignment_max_travel_yaw").value
            ),
        )
        short_drive_distance = max(
            0.0,
            float(
                self.get_parameter(
                    "alignment_short_drive_distance"
                ).value
            ),
        )
        short_forward_speed = max(
            0.0,
            float(
                self.get_parameter(
                    "alignment_short_forward_speed"
                ).value
            ),
        )
        max_reverse_distance = max(
            0.0,
            float(
                self.get_parameter(
                    "alignment_max_reverse_distance"
                ).value
            ),
        )
        max_reverse_yaw = max(
            0.0,
            float(
                self.get_parameter("alignment_max_reverse_yaw").value
            ),
        )
        reverse_speed = max(
            0.0,
            float(self.get_parameter("alignment_reverse_speed").value),
        )
        yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        max_detected_yaw = float(
            self.get_parameter("max_detected_yaw").value
        )
        required_aligned_samples = max(
            1,
            int(
                self.get_parameter(
                    "alignment_required_consecutive_samples"
                ).value
            ),
        )
        aligned_samples = 0
        correction_count = 0
        observation_limit = retry_count + required_aligned_samples

        for observation in range(1, observation_limit + 1):
            if time.monotonic() >= deadline:
                break
            _, x, y, shelf_heading = target
            if abs(shelf_heading) > max_detected_yaw:
                self.get_logger().error(
                    "safe-standoff alignment rejected: shelf heading "
                    f"{shelf_heading:.3f} exceeds "
                    f"{max_detected_yaw:.3f}"
                )
                return None

            error_x, error_y = shelf_staging_error(
                x, y, shelf_heading, standoff
            )
            position_error = math.hypot(error_x, error_y)
            self.get_logger().info(
                "safe-standoff alignment sample: "
                f"observation={observation}/{observation_limit} "
                f"corrections={correction_count}/{retry_count} "
                f"x={x:.3f} y={y:.3f} "
                f"shelf_normal_yaw={shelf_heading:.3f} "
                f"staging_error={position_error:.3f}"
            )

            scan_before_motion = self._current_scan_sequence()
            if position_error > position_tolerance:
                aligned_samples = 0
                if correction_count >= retry_count:
                    self.get_logger().error(
                        "safe-standoff alignment stopped: correction "
                        "budget exhausted before position tolerance"
                    )
                    break
                correction_count += 1
                if max_drive <= 0.0:
                    self.get_logger().error(
                        "safe-standoff alignment rejected: maximum staging "
                        "drive is not positive"
                    )
                    return None
                motion = staging_motion_command(
                    error_x,
                    error_y,
                    max_travel_yaw,
                    max_reverse_distance,
                    max_reverse_yaw,
                )
                if motion is None:
                    forward_yaw = math.atan2(error_y, error_x)
                    reverse_yaw = normalize_angle(forward_yaw + math.pi)
                    self.get_logger().error(
                        "safe-standoff staging rejected: travel yaw "
                        f"{forward_yaw:.3f} exceeds {max_travel_yaw:.3f}; "
                        "reverse equivalent rejected: "
                        f"distance={position_error:.3f}/"
                        f"{max_reverse_distance:.3f} "
                        f"yaw={reverse_yaw:.3f}/{max_reverse_yaw:.3f}"
                    )
                    return None
                travel_yaw, signed_distance, motion_mode = motion
                if motion_mode == "reverse":
                    minimum_safe_x = max(
                        standoff - max_reverse_distance, 0.0
                    )
                    if x < minimum_safe_x:
                        self.get_logger().error(
                            "safe-standoff reverse staging rejected: "
                            f"shelf x={x:.3f} below safe minimum "
                            f"{minimum_safe_x:.3f}"
                        )
                        return None
                staging_entry_odom_yaw = (
                    self._wait_for_stable_odom_yaw(deadline)
                )
                if staging_entry_odom_yaw is None:
                    self.get_logger().error(
                        "safe-standoff staging rejected: entry odom "
                        "heading did not settle"
                    )
                    return None
                if abs(travel_yaw) > yaw_tolerance and not (
                    self._rotate_measured(travel_yaw, deadline)
                ):
                    return None
                if abs(travel_yaw) > yaw_tolerance and (
                    self._wait_for_stable_odom_yaw(deadline) is None
                ):
                    return None
                staging_distance = min(abs(signed_distance), max_drive)
                signed_staging_distance = math.copysign(
                    staging_distance, signed_distance
                )
                staging_speed = None
                if motion_mode == "reverse":
                    staging_speed = reverse_speed
                    self.get_logger().warning(
                        "safe-standoff reverse-equivalent staging "
                        "selected: "
                        f"distance={signed_staging_distance:.3f} "
                        f"yaw={travel_yaw:.3f} "
                        f"speed={staging_speed:.3f}"
                    )
                elif staging_distance <= short_drive_distance:
                    staging_speed = short_forward_speed
                    self.get_logger().info(
                        "safe-standoff short staging speed selected: "
                        f"distance={staging_distance:.3f} "
                        f"speed={staging_speed:.3f}"
                    )
                if not self._drive_forward_measured(
                    signed_staging_distance,
                    deadline,
                    staging_speed,
                ):
                    return None
                post_drive_odom_yaw = self._wait_for_stable_odom_yaw(
                    deadline
                )
                if post_drive_odom_yaw is None:
                    self.get_logger().error(
                        "safe-standoff staging rejected: post-drive odom "
                        "heading did not settle"
                    )
                    return None
                restore_yaw = normalize_angle(
                    staging_entry_odom_yaw - post_drive_odom_yaw
                )
                self.get_logger().info(
                    "safe-standoff staging heading restore: "
                    f"entry_odom_yaw={staging_entry_odom_yaw:.3f} "
                    f"post_drive_odom_yaw={post_drive_odom_yaw:.3f} "
                    f"restore_yaw={restore_yaw:.3f}"
                )
                if abs(restore_yaw) > yaw_tolerance and not (
                    self._rotate_measured(restore_yaw, deadline)
                ):
                    return None
                if abs(restore_yaw) > yaw_tolerance and (
                    self._wait_for_stable_odom_yaw(deadline) is None
                ):
                    return None
            elif not shelf_heading_aligned(
                shelf_heading, yaw_tolerance
            ):
                aligned_samples = 0
                if correction_count >= retry_count:
                    self.get_logger().error(
                        "safe-standoff alignment stopped: correction "
                        "budget exhausted before heading tolerance"
                    )
                    break
                correction_count += 1
                (
                    correction,
                    correction_speed,
                    correction_regime,
                ) = alignment_yaw_command(
                    shelf_heading,
                    float(
                        self.get_parameter("alignment_heading_gain").value
                    ),
                    float(
                        self.get_parameter(
                            "alignment_max_yaw_correction"
                        ).value
                    ),
                    float(self.get_parameter("rotate_speed").value),
                    float(
                        self.get_parameter(
                            "alignment_fine_yaw_threshold"
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            "alignment_fine_heading_gain"
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            "alignment_fine_rotate_speed"
                        ).value
                    ),
                )
                self.get_logger().info(
                    "safe-standoff yaw correction selected: "
                    f"regime={correction_regime} "
                    f"observed={shelf_heading:.3f} "
                    f"target={correction:.3f} "
                    f"speed={correction_speed:.3f}"
                )
                if abs(correction) <= 0.0 or not self._rotate_measured(
                    correction, deadline, correction_speed
                ):
                    return None
                if self._wait_for_stable_odom_yaw(deadline) is None:
                    return None
            else:
                accepted_odom_yaw = self._wait_for_stable_odom_yaw(deadline)
                if accepted_odom_yaw is None:
                    self._publish_stop()
                    self.get_logger().error(
                        "safe-standoff alignment rejected: accepted odom "
                        "yaw did not settle"
                    )
                    return None
                aligned_samples += 1
                self.get_logger().info(
                    "safe-standoff alignment candidate: "
                    f"consecutive={aligned_samples}/"
                    f"{required_aligned_samples} "
                    f"position_error={position_error:.3f} "
                    f"shelf_normal_yaw={shelf_heading:.3f} "
                    f"accepted_odom_yaw={accepted_odom_yaw:.3f}"
                )
                if aligned_samples >= required_aligned_samples:
                    self.get_logger().info(
                        "safe-standoff alignment accepted: "
                        f"position_error={position_error:.3f} "
                        f"shelf_normal_yaw={shelf_heading:.3f} "
                        f"accepted_odom_yaw={accepted_odom_yaw:.3f}"
                    )
                    return target, accepted_odom_yaw

            target = self._recover_cart_frame_after_motion(
                scan_before_motion, deadline
            )
            if target is None:
                self.get_logger().error(
                    "safe-standoff alignment stopped: fresh shelf "
                    "observation unavailable"
                )
                return None

        self._publish_stop()
        self.get_logger().error(
            "safe-standoff alignment exhausted before consecutive "
            "position and heading acceptance"
        )
        return None

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

    def _lookup_odom_yaw_sample(self) -> Optional[tuple]:
        with self._odom_lock:
            return self._latest_odom_yaw_sample

    def _lookup_odom_yaw(self) -> Optional[float]:
        sample = self._lookup_odom_yaw_sample()
        return None if sample is None else sample[1]

    def _wait_for_odom_yaw(self, deadline: float) -> Optional[float]:
        lookup_timeout = max(
            0.0, float(self.get_parameter("odom_lookup_timeout").value)
        )
        lookup_deadline = min(deadline, time.monotonic() + lookup_timeout)
        while rclpy.ok() and time.monotonic() < lookup_deadline:
            yaw = self._lookup_odom_yaw()
            if yaw is not None:
                return yaw
            time.sleep(0.05)
        return None

    def _wait_for_stable_odom_yaw(
        self, deadline: float
    ) -> Optional[float]:
        """Wait for consecutive fresh odom samples with settled yaw."""
        self._publish_stop()
        settle_timeout = max(
            0.0,
            float(self.get_parameter("alignment_settle_timeout").value),
        )
        settle_deadline = min(
            deadline, time.monotonic() + settle_timeout
        )
        required_samples = max(
            2,
            int(
                self.get_parameter(
                    "alignment_settle_sample_count"
                ).value
            ),
        )
        yaw_tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "alignment_settle_yaw_tolerance"
                ).value
            ),
        )
        last_stamp = None
        last_yaw = None
        stable_samples = 0

        while rclpy.ok() and time.monotonic() < settle_deadline:
            sample = self._lookup_odom_yaw_sample()
            if sample is None:
                time.sleep(0.05)
                continue
            stamp, yaw = sample
            if stamp == last_stamp:
                time.sleep(0.05)
                continue

            if last_yaw is None or abs(
                normalize_angle(yaw - last_yaw)
            ) <= yaw_tolerance:
                stable_samples += 1
            else:
                stable_samples = 1
            last_stamp = stamp
            last_yaw = yaw

            if stable_samples >= required_samples:
                self.get_logger().info(
                    "post-rotation odom settled: "
                    f"yaw={yaw:.3f} samples={stable_samples}/"
                    f"{required_samples} tolerance={yaw_tolerance:.3f}"
                )
                return yaw
            time.sleep(0.05)

        self._publish_stop()
        self.get_logger().error(
            "post-rotation odom settling failed: "
            f"stable_samples={stable_samples}/{required_samples}"
        )
        return None

    def _accepted_odom_heading_ok(
        self, accepted_yaw: float, deadline: float
    ) -> bool:
        current_yaw = self._wait_for_odom_yaw(deadline)
        if current_yaw is None:
            self._publish_stop()
            self.get_logger().error(
                "attach stopped: constrained-entry odom yaw unavailable"
            )
            return False

        drift = normalize_angle(current_yaw - accepted_yaw)
        tolerance = max(
            0.0,
            float(
                self.get_parameter("entry_odom_yaw_tolerance").value
            ),
        )
        self.get_logger().info(
            "constrained-entry odom heading guard: "
            f"accepted={accepted_yaw:.3f} current={current_yaw:.3f} "
            f"drift={drift:.3f} tolerance={tolerance:.3f}"
        )
        if abs(drift) > tolerance:
            self._publish_stop()
            self.get_logger().error(
                "attach stopped: odom heading drifted outside tolerance; "
                f"drift={drift:.3f} tolerance={tolerance:.3f}"
            )
            return False
        return True

    def _drive_forward_measured(
        self,
        distance: float,
        deadline: float,
        speed_override: Optional[float] = None,
    ) -> bool:
        """Drive a signed distance using odom magnitude and always stop."""
        speed = (
            float(self.get_parameter("forward_speed").value)
            if speed_override is None
            else float(speed_override)
        )
        target_distance = abs(distance)
        if target_distance <= 0.0:
            self._publish_stop()
            return True
        if speed <= 0.0 or time.monotonic() >= deadline:
            self._publish_stop()
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
            time.monotonic()
            + max(2.0, target_distance / speed * timeout_scale),
        )
        lookup_timeout = max(
            0.0, float(self.get_parameter("odom_lookup_timeout").value)
        )
        last_odom_time = time.monotonic()
        traveled = 0.0
        command = Twist()
        direction = 1.0 if distance > 0.0 else -1.0
        command.linear.x = direction * speed
        motion_name = "forward" if direction > 0.0 else "reverse"
        self.get_logger().info(
            f"measured {motion_name} drive started: "
            f"target_distance={distance:.3f} "
            f"speed={command.linear.x:.3f}"
        )
        try:
            while rclpy.ok() and time.monotonic() < motion_deadline:
                position = self._lookup_odom_xy()
                if position is not None:
                    last_odom_time = time.monotonic()
                    traveled = math.hypot(
                        position[0] - start[0], position[1] - start[1]
                    )
                    if traveled >= target_distance:
                        self.get_logger().info(
                            f"measured {motion_name} drive complete: "
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
            f"measured {motion_name} drive stopped before target: "
            f"target={distance:.3f} traveled={traveled:.3f}"
        )
        return False

    def _rotate_measured(
        self,
        yaw: float,
        deadline: float,
        speed_override: Optional[float] = None,
    ) -> bool:
        speed = (
            float(self.get_parameter("rotate_speed").value)
            if speed_override is None
            else float(speed_override)
        )
        target = abs(yaw)
        tolerance = max(
            0.0,
            float(self.get_parameter("measured_yaw_tolerance").value),
        )
        if target <= tolerance:
            self._publish_stop()
            return True
        if speed <= 0.0 or time.monotonic() >= deadline:
            self._publish_stop()
            return False

        start = self._wait_for_odom_yaw(deadline)
        if start is None:
            self._publish_stop()
            self.get_logger().error(
                "measured yaw rejected: odom TF unavailable"
            )
            return False

        timeout_scale = max(
            1.0,
            float(
                self.get_parameter(
                    "measured_rotation_timeout_scale"
                ).value
            ),
        )
        motion_deadline = min(
            deadline,
            time.monotonic() + max(2.0, target / speed * timeout_scale),
        )
        lookup_timeout = max(
            0.0, float(self.get_parameter("odom_lookup_timeout").value)
        )
        last_odom_time = time.monotonic()
        last_yaw = start
        traveled = 0.0
        direction = math.copysign(1.0, yaw)
        command = Twist()
        command.angular.z = direction * speed
        self.get_logger().info(
            "measured yaw correction started: "
            f"target={yaw:.3f} speed={speed:.3f}"
        )
        try:
            while rclpy.ok() and time.monotonic() < motion_deadline:
                current_yaw = self._lookup_odom_yaw()
                if current_yaw is not None:
                    last_odom_time = time.monotonic()
                    delta = normalize_angle(current_yaw - last_yaw)
                    last_yaw = current_yaw
                    traveled += direction * delta
                    if traveled >= target - tolerance:
                        self.get_logger().info(
                            "measured yaw correction complete: "
                            f"target={yaw:.3f} "
                            f"traveled={direction * traveled:.3f}"
                        )
                        return True
                elif time.monotonic() - last_odom_time >= lookup_timeout:
                    self.get_logger().error(
                        "measured yaw stopped: odom TF stale"
                    )
                    return False
                self._cmd_vel_pub.publish(command)
                time.sleep(0.05)
        finally:
            self._publish_stop()

        self.get_logger().error(
            "measured yaw stopped before target: "
            f"target={yaw:.3f} traveled={direction * traveled:.3f}"
        )
        return False

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

    def _transform_detection_to_base(
        self, measurement: LegPairMeasurement
    ) -> Optional[tuple]:
        target_frame = str(self.get_parameter("target_base_frame").value)
        source_frame = measurement.frame_id
        if not source_frame:
            return None
        if source_frame == target_frame:
            return (
                target_frame,
                measurement.midpoint_x,
                measurement.midpoint_y,
                measurement.shelf_normal_yaw,
            )
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )

            def transform_xy(x: float, y: float) -> tuple:
                point = PointStamped()
                point.header.frame_id = source_frame
                point.point.x = x
                point.point.y = y
                transformed = do_transform_point(point, transform)
                return transformed.point.x, transformed.point.y

            midpoint_x, midpoint_y = transform_xy(
                measurement.midpoint_x, measurement.midpoint_y
            )
            left_x, left_y = transform_xy(
                measurement.left_x, measurement.left_y
            )
            right_x, right_y = transform_xy(
                measurement.right_x, measurement.right_y
            )
            return (
                target_frame,
                midpoint_x,
                midpoint_y,
                shelf_normal_yaw(left_x, left_y, right_x, right_y),
            )
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
    # Keep scan, odom, and the long-running service independently schedulable.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
