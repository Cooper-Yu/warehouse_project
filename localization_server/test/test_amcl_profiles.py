from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"


def _parameters(filename):
    return yaml.safe_load(
        (CONFIG_DIR / filename).read_text(encoding="utf-8")
    )["amcl"]["ros__parameters"]


def test_simulation_amcl_uses_probabilistic_beam_skipping():
    params = _parameters("amcl_config_sim.yaml")

    assert params["scan_topic"] == "/scan_localization"
    assert params["laser_model_type"] == "likelihood_field_prob"
    assert params["do_beamskip"] is True
    assert params["beam_skip_distance"] == 0.5
    assert params["beam_skip_threshold"] == 0.3
    assert params["beam_skip_error_threshold"] == 0.9


def test_real_amcl_profile_is_not_switched_to_beam_skipping():
    params = _parameters("amcl_config_real.yaml")

    assert params["scan_topic"] == "/scan_localization"
    assert params["laser_model_type"] == "likelihood_field"
    assert params["do_beamskip"] is False
