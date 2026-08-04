import ast
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).parents[1] / "launch" / "pathplanner.launch.py"
)


def _source_and_tree():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    return source, ast.parse(source)


def test_lateral_execution_launch_argument_defaults_false():
    _source, tree = _source_and_tree()
    declarations = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "lateral_execution_enabled"
    ]

    assert len(declarations) == 1
    keywords = {keyword.arg: keyword.value for keyword in declarations[0].keywords}
    assert isinstance(keywords["default_value"], ast.Constant)
    assert keywords["default_value"].value == "false"


def test_lateral_execution_argument_only_reaches_simulation_shelf_node():
    source, tree = _source_and_tree()
    shelf_nodes = []
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "Node"
        ):
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        package = keywords.get("package")
        if (
            isinstance(package, ast.Constant)
            and package.value == "shelf_detection_server"
        ):
            shelf_nodes.append((node, keywords))

    assert len(shelf_nodes) == 1
    node, keywords = shelf_nodes[0]
    condition_source = ast.get_source_segment(source, keywords["condition"])
    node_source = ast.get_source_segment(source, node)

    assert condition_source == "IfCondition(use_sim_time)"
    assert '"lateral_execution_enabled": ParameterValue(' in node_source
    assert "lateral_execution_enabled," in node_source
    assert '"alignment_fine_min_rotate_speed": 0.015' in node_source
    assert '"alignment_fine_speed_gain": 0.50' in node_source
    assert '"alignment_heading_hold_tolerance": 0.04' in node_source


def test_real_navigation_branch_does_not_receive_lateral_argument():
    source, tree = _source_and_tree()
    navigation_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_navigation_nodes"
    ]
    real_call = next(
        node
        for node in navigation_calls
        if "UnlessCondition(use_sim_time)"
        in ast.get_source_segment(source, node)
    )

    assert "lateral_execution_enabled" not in ast.get_source_segment(
        source, real_call
    )
