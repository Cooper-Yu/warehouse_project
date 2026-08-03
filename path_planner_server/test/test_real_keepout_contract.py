from collections import Counter
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).parents[2]
MAP_CONFIG = REPOSITORY / "map_server" / "config"
PLANNER = REPOSITORY / "path_planner_server"


def _read_pgm(path):
    with path.open(encoding="ascii") as stream:
        magic = stream.readline().strip()
        comment = stream.readline().strip()
        width, height = map(int, stream.readline().split())
        maximum = int(stream.readline())
        values = [int(value) for value in stream.read().split()]
    return magic, comment, width, height, maximum, values


def test_real_keepout_map_and_mask_metadata_match():
    map_yaml = yaml.safe_load(
        (MAP_CONFIG / "warehouse_map_keepout_real.yaml").read_text()
    )
    mask_yaml = yaml.safe_load(
        (MAP_CONFIG / "warehouse_map_keepout_real_mask.yaml").read_text()
    )

    assert map_yaml["resolution"] == mask_yaml["resolution"] == 0.05
    assert map_yaml["origin"] == mask_yaml["origin"] == [-2.55, -5.76, 0]
    assert map_yaml["image"] == "warehouse_map_real.pgm"
    assert mask_yaml["image"] == "warehouse_map_keepout_real_mask.pgm"


def test_real_keepout_mask_has_aligned_binary_geometry():
    magic, comment, width, height, maximum, values = _read_pgm(
        MAP_CONFIG / "warehouse_map_keepout_real_mask.pgm"
    )

    assert magic == "P2"
    assert "0=keepout, 254=free" in comment
    assert (width, height, maximum) == (263, 169, 255)
    assert len(values) == width * height
    assert Counter(values) == Counter({254: 43787, 0: 660})


def test_real_global_costmap_uses_keepout_filter():
    planner = yaml.safe_load(
        (PLANNER / "config" / "planner_real.yaml").read_text()
    )
    parameters = planner["global_costmap"]["global_costmap"]["ros__parameters"]

    assert parameters["filters"] == ["keepout_filter"]
    assert parameters["keepout_filter"]["plugin"] == (
        "nav2_costmap_2d::KeepoutFilter"
    )
    assert parameters["keepout_filter"]["enabled"] is True
    assert parameters["keepout_filter"]["filter_info_topic"] == (
        "/costmap_filter_info"
    )


def test_launch_selects_matching_real_mask_default():
    source = (
        PLANNER / "launch" / "pathplanner.launch.py"
    ).read_text(encoding="utf-8")

    assert "warehouse_map_keepout_sim_mask.yaml" in source
    assert "warehouse_map_keepout_real_mask.yaml" in source
    assert "default_value=PythonExpression" in source
