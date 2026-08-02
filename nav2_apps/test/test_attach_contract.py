import ast
from pathlib import Path


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    )


def _called_methods(function):
    return [
        node.func.attr
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]


def test_attach_service_owns_motion_and_elevator_sequence():
    repository = Path(__file__).parents[2]
    server_source = (
        repository
        / "shelf_detection_server"
        / "shelf_detection_server"
        / "server.py"
    )
    mission_source = (
        repository / "nav2_apps" / "nav2_apps" / "move_shelf_to_ship.py"
    )

    server_tree = ast.parse(server_source.read_text(encoding="utf-8"))
    mission_tree = ast.parse(mission_source.read_text(encoding="utf-8"))

    server_attach = _function(server_tree, "_perform_stepwise_attach")
    server_calls = _called_methods(server_attach)
    assert "_align_at_safe_standoff" in server_calls
    assert "_drive_forward_measured" in server_calls
    assert "_apply_pre_lock_yaw" not in server_calls
    assert "_publish_elevator_up" in server_calls

    alignment = _function(server_tree, "_align_at_safe_standoff")
    alignment_calls = _called_methods(alignment)
    assert "_rotate_measured" in alignment_calls
    assert "_rotate_open_loop" not in alignment_calls
    assert "_recover_cart_frame_after_motion" in alignment_calls

    mission_attach = _function(mission_tree, "_request_stepwise_attach")
    assigned_true = any(
        isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and target.attr == "attach_to_shelf"
            for target in node.targets
        )
        and isinstance(node.value, ast.Constant)
        and node.value.value is True
        for node in ast.walk(mission_attach)
    )
    assert assigned_true
