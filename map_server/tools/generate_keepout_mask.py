#!/usr/bin/env python3
"""Generate the simulation keepout mask with loaded-footprint clearance."""

import argparse
import math
from pathlib import Path


WIDTH = 159
HEIGHT = 139
RESOLUTION = 0.05
ORIGIN_X = -1.26
ORIGIN_Y = -4.79

# RViz clicked points accepted for the raw central keepout polygon.
RAW_KEEPOUT_POLYGON = (
    (3.4589109421, -0.5354431868),
    (4.0597615242, -1.4365822077),
    (1.6563576460, -3.1530382633),
    (1.0412006378, -2.3377218246),
)

# Loaded footprint is +/-0.40 x +/-0.45 m.  The keepout filter is evaluated
# after the ordinary inflation layer, so the mask itself must include a
# rotation-independent clearance.  Include the configured 0.01 m padding.
LOADED_CLEARANCE = math.hypot(0.40, 0.45) + 0.01


def _point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection_x:
                inside = not inside
        previous = current
    return inside


def _segment_distance(x, y, start, end):
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(x - x1, y - y1)
    ratio = ((x - x1) * dx + (y - y1) * dy) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    closest_x = x1 + ratio * dx
    closest_y = y1 + ratio * dy
    return math.hypot(x - closest_x, y - closest_y)


def _inside_expanded_keepout(x, y):
    if _point_in_polygon(x, y, RAW_KEEPOUT_POLYGON):
        return True
    previous = RAW_KEEPOUT_POLYGON[-1]
    for current in RAW_KEEPOUT_POLYGON:
        if _segment_distance(x, y, previous, current) <= LOADED_CLEARANCE:
            return True
        previous = current
    return False


def render_mask():
    lines = [
        "P2",
        (
            "# warehouse_map_keepout_sim_mask: 0=keepout, 254=free; "
            f"loaded_clearance={LOADED_CLEARANCE:.4f}m"
        ),
        f"{WIDTH} {HEIGHT}",
        "255",
    ]
    for image_row in range(HEIGHT):
        map_row = HEIGHT - image_row - 1
        values = []
        for column in range(WIDTH):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            y = ORIGIN_Y + (map_row + 0.5) * RESOLUTION
            value = "0" if _inside_expanded_keepout(x, y) else "254"
            values.append(value)
        lines.append(" ".join(values))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "config"
            / "warehouse_map_keepout_sim_mask.pgm"
        ),
    )
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    expected = render_mask()
    if args.check:
        if not args.output.exists() or args.output.read_text() != expected:
            raise SystemExit(
                "keepout mask is not generated from current geometry"
            )
        print(
            "keepout mask matches generator: "
            f"loaded_clearance={LOADED_CLEARANCE:.4f}m"
        )
        return

    args.output.write_text(expected)
    print(
        f"wrote {args.output}: loaded_clearance={LOADED_CLEARANCE:.4f}m"
    )


if __name__ == "__main__":
    main()
