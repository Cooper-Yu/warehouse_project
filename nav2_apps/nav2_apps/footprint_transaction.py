"""Compensating transaction for two live Nav2 costmap footprints."""

import ast
import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

import rclpy
from geometry_msgs.msg import PolygonStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node


GLOBAL_COSTMAP = "/global_costmap/global_costmap"
LOCAL_COSTMAP = "/local_costmap/local_costmap"
TARGETS = (GLOBAL_COSTMAP, LOCAL_COSTMAP)


@dataclass(frozen=True)
class FootprintSnapshot:
    footprint: str
    padding: float


@dataclass(frozen=True)
class TransactionResult:
    success: bool
    rollback_verified: bool
    reason: str


def parse_polygon(value: str) -> List[List[float]]:
    """Parse and validate the four-point configured rectangle."""
    parsed = ast.literal_eval(value)
    if not isinstance(parsed, list) or len(parsed) != 4:
        raise ValueError("footprint must contain four points")
    points = []
    for item in parsed:
        if not isinstance(item, list) or len(item) != 2:
            raise ValueError("each footprint point must contain x and y")
        point = [float(item[0]), float(item[1])]
        if not all(math.isfinite(coordinate) for coordinate in point):
            raise ValueError("footprint coordinates must be finite")
        points.append(point)
    return points


def expected_padded_edges(value: str, padding: float) -> List[float]:
    """Return padded edge lengths for an axis-aligned rectangle."""
    points = parse_polygon(value)
    xs = sorted({point[0] for point in points})
    ys = sorted({point[1] for point in points})
    if len(xs) != 2 or len(ys) != 2:
        raise ValueError("loaded footprint must be an axis-aligned rectangle")
    length = xs[1] - xs[0] + 2.0 * padding
    width = ys[1] - ys[0] + 2.0 * padding
    return sorted([length, length, width, width])


def polygon_edges(message: PolygonStamped) -> List[float]:
    points = message.polygon.points
    if len(points) != 4:
        return []
    edges = []
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        edges.append(math.hypot(second.x - first.x, second.y - first.y))
    return sorted(edges)


def edges_match(
    observed: Sequence[float],
    expected: Sequence[float],
    tolerance: float,
) -> bool:
    if len(observed) != len(expected):
        return False
    return all(
        abs(actual - wanted) <= tolerance
        for actual, wanted in zip(sorted(observed), sorted(expected))
    )


def run_compensating_transaction(
    targets: Sequence[str],
    desired: str,
    snapshot: Callable[[str], FootprintSnapshot],
    set_value: Callable[[str, str], bool],
    read_value: Callable[[str], str],
    verify_desired: Callable[[Dict[str, FootprintSnapshot]], bool],
    verify_restored: Callable[[Dict[str, FootprintSnapshot]], bool],
) -> TransactionResult:
    """Apply both targets and restore snapshots after partial failure."""
    originals: Dict[str, FootprintSnapshot] = {}
    try:
        for target in targets:
            originals[target] = snapshot(target)
    except Exception as error:
        return TransactionResult(False, True, f"snapshot failed: {error}")

    failure = ""
    try:
        for target in targets:
            if not set_value(target, desired):
                failure = f"set rejected by {target}"
                break
            if read_value(target) != desired:
                failure = f"readback mismatch on {target}"
                break
        if not failure and not verify_desired(originals):
            failure = "published desired footprint verification failed"
    except Exception as error:
        failure = f"transaction exception: {error}"
    if not failure:
        return TransactionResult(True, True, "desired footprint verified")

    rollback_ok = True
    for target in targets:
        try:
            restored = set_value(target, originals[target].footprint)
        except Exception:
            restored = False
        rollback_ok = restored and rollback_ok
    for target in targets:
        try:
            restored = read_value(target) == originals[target].footprint
        except Exception:
            restored = False
        rollback_ok = restored and rollback_ok
    try:
        restored_shape = verify_restored(originals)
    except Exception:
        restored_shape = False
    rollback_ok = restored_shape and rollback_ok
    return TransactionResult(False, rollback_ok, failure)


class RosFootprintBackend:
    """ROS service/topic adapter used while the robot remains stopped."""

    def __init__(self, node: Node, timeout: float, tolerance: float) -> None:
        self.node = node
        self.timeout = timeout
        self.tolerance = tolerance
        self._get_clients = {
            target: node.create_client(
                GetParameters, f"{target}/get_parameters"
            )
            for target in TARGETS
        }
        self._set_clients = {
            target: node.create_client(
                SetParameters, f"{target}/set_parameters"
            )
            for target in TARGETS
        }
        self._messages: Dict[str, Optional[PolygonStamped]] = {
            target: None for target in TARGETS
        }
        self._subscriptions = [
            node.create_subscription(
                PolygonStamped,
                "/global_costmap/published_footprint",
                lambda message: self._capture(GLOBAL_COSTMAP, message),
                10,
            ),
            node.create_subscription(
                PolygonStamped,
                "/local_costmap/published_footprint",
                lambda message: self._capture(LOCAL_COSTMAP, message),
                10,
            ),
        ]

    def _capture(self, target: str, message: PolygonStamped) -> None:
        self._messages[target] = message

    def _wait_future(self, future):
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("parameter service response timed out")
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not future.done() or future.result() is None:
            raise RuntimeError("parameter service returned no response")
        return future.result()

    def _wait_client(self, client) -> None:
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                raise TimeoutError("parameter service unavailable")

    def snapshot(self, target: str) -> FootprintSnapshot:
        client = self._get_clients[target]
        self._wait_client(client)
        request = GetParameters.Request()
        request.names = ["footprint", "footprint_padding"]
        response = self._wait_future(client.call_async(request))
        if len(response.values) != 2:
            raise RuntimeError(f"incomplete parameter snapshot from {target}")
        footprint, padding = response.values
        if footprint.type != ParameterType.PARAMETER_STRING:
            raise TypeError(f"footprint on {target} is not a string")
        if padding.type != ParameterType.PARAMETER_DOUBLE:
            raise TypeError(f"footprint_padding on {target} is not a double")
        return FootprintSnapshot(
            footprint=footprint.string_value,
            padding=padding.double_value,
        )

    def set_value(self, target: str, value: str) -> bool:
        client = self._set_clients[target]
        self._wait_client(client)
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = "footprint"
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=value,
        )
        request.parameters = [parameter]
        response = self._wait_future(client.call_async(request))
        return (
            len(response.results) == 1
            and response.results[0].successful
        )

    def read_value(self, target: str) -> str:
        return self.snapshot(target).footprint

    def _verify_messages(
        self, expected: Dict[str, List[float]]
    ) -> bool:
        self._messages = {target: None for target in TARGETS}
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(self._messages[target] is not None for target in TARGETS):
                break
        for target in TARGETS:
            message = self._messages[target]
            if message is None:
                return False
            if not edges_match(
                polygon_edges(message), expected[target], self.tolerance
            ):
                return False
        return True

    def verify_loaded(
        self,
        desired: str,
        originals: Dict[str, FootprintSnapshot],
    ) -> bool:
        expected = {
            target: expected_padded_edges(
                desired, originals[target].padding
            )
            for target in TARGETS
        }
        return self._verify_messages(expected)

    def verify_restored(
        self, originals: Dict[str, FootprintSnapshot]
    ) -> bool:
        expected = {
            target: expected_padded_edges(
                originals[target].footprint,
                originals[target].padding,
            )
            for target in TARGETS
        }
        return self._verify_messages(expected)


def apply_footprint(
    node: Node,
    desired: str,
    timeout: float,
    tolerance: float,
) -> TransactionResult:
    """Apply and verify a footprint, compensating to live snapshots."""
    parse_polygon(desired)
    backend = RosFootprintBackend(node, timeout, tolerance)
    return run_compensating_transaction(
        TARGETS,
        desired,
        backend.snapshot,
        backend.set_value,
        backend.read_value,
        lambda originals: backend.verify_loaded(desired, originals),
        backend.verify_restored,
    )


def apply_loaded_footprint(
    node: Node,
    desired: str,
    timeout: float,
    tolerance: float,
) -> TransactionResult:
    """Backward-compatible wrapper for the loaded-footprint slice."""
    return apply_footprint(node, desired, timeout, tolerance)
