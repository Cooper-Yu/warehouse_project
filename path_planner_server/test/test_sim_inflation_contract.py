from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"


def _parameters(filename, node_name):
    data = yaml.safe_load((CONFIG / filename).read_text(encoding="utf-8"))
    return data[node_name][node_name]["ros__parameters"]


def test_sim_global_and_local_inflation_are_synchronized():
    global_parameters = _parameters("planner_sim.yaml", "global_costmap")
    local_parameters = _parameters("controller_sim.yaml", "local_costmap")

    global_inflation = global_parameters["inflation_layer"]
    local_inflation = local_parameters["inflation_layer"]

    for inflation in (global_inflation, local_inflation):
        assert inflation["plugin"] == "nav2_costmap_2d::InflationLayer"
        assert inflation["inflation_radius"] == 0.50
        assert inflation["cost_scaling_factor"] == 10.0


def test_real_inflation_profiles_are_not_changed_by_sim_tuning():
    real_global = _parameters("planner_real.yaml", "global_costmap")
    real_local = _parameters("controller_real.yaml", "local_costmap")

    assert "inflation_radius" not in real_global["inflation_layer"]
    assert "inflation_radius" not in real_local["inflation_layer"]
