from pathlib import Path

import yaml


PROJECT = Path(__file__).resolve().parents[2]
MAP_CONFIG = PROJECT / "map_server" / "config"


def _read_p2(path):
    tokens = []
    comment = ""
    for line in path.read_text(encoding="ascii").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            comment = stripped
            continue
        tokens.extend(stripped.split())
    magic = tokens.pop(0)
    width = int(tokens.pop(0))
    height = int(tokens.pop(0))
    maximum = int(tokens.pop(0))
    return magic, width, height, maximum, [int(v) for v in tokens], comment


def test_current_sim_keepout_metadata_matches_cleared_map():
    map_yaml = yaml.safe_load(
        (MAP_CONFIG / "warehouse_map_keepout_sim.yaml").read_text()
    )
    mask_yaml = yaml.safe_load(
        (MAP_CONFIG / "warehouse_map_keepout_sim_mask.yaml").read_text()
    )

    assert map_yaml["resolution"] == mask_yaml["resolution"] == 0.05
    assert map_yaml["origin"] == mask_yaml["origin"] == [-1.01, -4.24, 0]
    assert mask_yaml["mode"] == "scale"
    assert mask_yaml["image"] == "warehouse_map_keepout_sim_mask.pgm"


def test_current_sim_keepout_mask_has_aligned_binary_geometry():
    magic, width, height, maximum, values, comment = _read_p2(
        MAP_CONFIG / "warehouse_map_keepout_sim_mask.pgm"
    )

    assert (magic, width, height, maximum) == ("P2", 153, 127, 255)
    assert len(values) == width * height
    assert set(values) == {0, 254}
    assert 1000 < values.count(0) < 2000
    assert "0=keepout, 254=free" in comment
