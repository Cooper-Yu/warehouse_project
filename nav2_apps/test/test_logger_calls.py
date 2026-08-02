import ast
from pathlib import Path


def test_rclpy_logger_calls_use_one_preformatted_message():
    source = (
        Path(__file__).parents[1]
        / "nav2_apps"
        / "move_shelf_to_ship.py"
    )
    tree = ast.parse(source.read_text(encoding="utf-8"))

    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Attribute):
            continue
        if call.func.attr not in {"debug", "info", "warning", "error"}:
            continue
        assert len(call.args) == 1, (
            "rclpy RcutilsLogger calls must receive one preformatted message"
        )
