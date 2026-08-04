import ast
from pathlib import Path


SERVER_PATH = (
    Path(__file__).parents[1]
    / "shelf_detection_server"
    / "server.py"
)


def _server_tree():
    return ast.parse(SERVER_PATH.read_text(encoding="utf-8"))


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _called_names(function):
    return [
        node.func.id
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
    ]


def test_safe_standoff_calls_pure_shadow_policy_and_state_mapping():
    alignment = _function(_server_tree(), "_align_at_safe_standoff")
    calls = _called_names(alignment)

    assert calls.count("classify_lateral_center") == 1
    assert calls.count("next_state_from_decision") == 1


def test_shadow_log_contains_minimum_explainable_fields():
    source = SERVER_PATH.read_text(encoding="utf-8")

    assert "lateral-centering shadow observation:" in source
    for field in (
        "lateral_error=",
        "tolerance=",
        "fresh=",
        "safe_zone=",
        "heading_ok=",
        "decision=",
        "state=",
    ):
        assert field in source


def test_shadow_outputs_do_not_control_branches_or_returns():
    alignment = _function(_server_tree(), "_align_at_safe_standoff")
    controlled_names = {"lateral_decision", "lateral_state"}

    for node in ast.walk(alignment):
        if isinstance(node, (ast.If, ast.While)):
            used = {
                child.id
                for child in ast.walk(node.test)
                if isinstance(child, ast.Name)
            }
            assert controlled_names.isdisjoint(used)
        if isinstance(node, ast.Return) and node.value is not None:
            used = {
                child.id
                for child in ast.walk(node.value)
                if isinstance(child, ast.Name)
            }
            assert controlled_names.isdisjoint(used)


def test_shadow_outputs_are_not_passed_to_motion_helpers():
    alignment = _function(_server_tree(), "_align_at_safe_standoff")
    motion_helpers = {"_rotate_measured", "_drive_forward_measured"}

    for node in ast.walk(alignment):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in motion_helpers
        ):
            continue
        used = {
            child.id
            for child in ast.walk(node)
            if isinstance(child, ast.Name)
        }
        assert "lateral_decision" not in used
        assert "lateral_state" not in used
