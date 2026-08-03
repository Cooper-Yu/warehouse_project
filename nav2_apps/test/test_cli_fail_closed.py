import sys

import pytest

from nav2_apps import move_shelf_to_ship


def test_unknown_mission_flag_fails_before_ros_initialization(monkeypatch):
    ros_initialized = []
    navigator_created = []
    monkeypatch.setattr(
        move_shelf_to_ship.rclpy,
        "init",
        lambda **_kwargs: ros_initialized.append(True),
    )
    monkeypatch.setattr(
        move_shelf_to_ship,
        "BasicNavigator",
        lambda: navigator_created.append(True),
    )

    with pytest.raises(SystemExit) as error:
        move_shelf_to_ship.main(["--return-onyl"])

    assert error.value.code == 2
    assert not ros_initialized
    assert not navigator_created


def test_ros_arguments_are_removed_before_strict_mission_parsing():
    args = move_shelf_to_ship._parse_application_args(
        [
            "--return-only",
            "--ros-args",
            "-r",
            "__node:=return_mission",
        ]
    )

    assert args.return_only


def test_process_name_is_removed_when_main_uses_sys_argv(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "/installed/move_shelf_to_ship.py",
            "--return-only",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
    )

    args = move_shelf_to_ship._parse_application_args()

    assert args.return_only
