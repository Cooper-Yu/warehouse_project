#!/usr/bin/env python3
"""Generate the Checkpoint 12 real-robot keepout mask from RViz map points."""

from pathlib import Path


WIDTH = 263
HEIGHT = 169
RESOLUTION = 0.05
ORIGIN_X = -2.55
ORIGIN_Y = -5.76

# Learner-selected /clicked_point vertices, recorded in map frame order.
POLYGON = (
    (2.4070746898651123, -1.252284049987793),
    (3.1255667209625244, -1.6741911172866821),
    (2.1789815425872803, -3.316209316253662),
    (1.3920619487762451, -2.9171078205108643),
)


def point_in_polygon(x, y, polygon):
    """Return True when a point lies inside the ordered polygon."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        crosses = (y1 > y) != (y2 > y)
        if crosses:
            boundary_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < boundary_x:
                inside = not inside
        previous = current
    return inside


def main():
    output = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "warehouse_map_keepout_real_mask.pgm"
    )

    rows = []
    keepout_cells = 0
    for image_row in range(HEIGHT):
        map_row = HEIGHT - 1 - image_row
        values = []
        for column in range(WIDTH):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            y = ORIGIN_Y + (map_row + 0.5) * RESOLUTION
            keepout = point_in_polygon(x, y, POLYGON)
            values.append("0" if keepout else "254")
            keepout_cells += int(keepout)
        rows.append(" ".join(values))

    contents = "\n".join(
        [
            "P2",
            "# warehouse_map_keepout_real_mask: 0=keepout, 254=free",
            f"{WIDTH} {HEIGHT}",
            "255",
            *rows,
            "",
        ]
    )
    output.write_text(contents, encoding="ascii")

    print(f"MASK_FILE={output}")
    print(f"KEEP_OUT_CELLS={keepout_cells}")
    print(f"TOTAL_CELLS={WIDTH * HEIGHT}")


if __name__ == "__main__":
    main()
