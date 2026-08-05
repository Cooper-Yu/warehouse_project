import ast
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as NavPath

from nav2_apps import explicit_start_pair_probe as probe


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "nav2_apps"
    / "explicit_start_pair_probe.py"
)


def test_pair_probe_is_diagnostic_only_and_uses_one_explicit_start():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert "request.use_start = True" in source
    assert "request.start = deepcopy(start)" in source
    assert "ComputePathToPose" in source
    assert "ActionClient" in source
    assert "create_publisher" not in source
    assert "create_client" not in source
    assert "goToPose" not in source
    assert "NavigateToPose" not in source
    assert "cmd_vel" in source
    assert any(
        isinstance(node, ast.FunctionDef) and node.name == "_request"
        for node in ast.walk(tree)
    )


def test_pose_delta_is_wrap_safe():
    first = PoseStamped()
    second = PoseStamped()
    first.pose.orientation.z = 0.99995
    first.pose.orientation.w = 0.01
    second.pose.orientation.z = -0.99995
    second.pose.orientation.w = 0.01
    position, yaw = probe._pose_delta(first, second)
    assert position == 0.0
    assert yaw < 0.05


def test_costmap_digest_changes_with_grid_data():
    first = Costmap()
    first.header.frame_id = "map"
    first.metadata.resolution = 0.05
    first.metadata.size_x = 2
    first.metadata.size_y = 1
    first.data = [0, 255]
    second = Costmap()
    second.header.frame_id = "map"
    second.metadata.resolution = 0.05
    second.metadata.size_x = 2
    second.metadata.size_y = 1
    second.data = [0, 100]
    assert probe._costmap_digest(first) != probe._costmap_digest(second)


def test_costmap_subscription_uses_nav2_type_and_transient_local_qos():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    assert "from nav2_msgs.msg import Costmap" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "OccupancyGrid" not in source


def test_usable_path_requires_map_frame_and_two_finite_poses():
    class Result:
        pass

    result = Result()
    result.path = NavPath()
    result.path.header.frame_id = "map"
    result.path.poses = [PoseStamped(), PoseStamped()]
    assert probe._path_is_usable(result, "map")
    result.path.header.frame_id = "odom"
    assert not probe._path_is_usable(result, "map")
