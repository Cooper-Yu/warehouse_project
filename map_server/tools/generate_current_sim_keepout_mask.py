#!/usr/bin/env python3
"""Generate the current The Construct simulation keepout mask."""

from pathlib import Path


WIDTH = 153
HEIGHT = 127
RESOLUTION = 0.05
ORIGIN_X = -1.01
ORIGIN_Y = -4.24

# Learner-selected /clicked_point vertices in ordered map-frame coordinates.
POLYGON = (
    (3.328113317489624, -0.5806533694267273),
    (3.9539997577667236, -1.6789807081222534),
    (1.8975154161453247, -3.03273344039917),
    (0.7990207672119141, -2.125974416732788),
)


def point_in_polygon(x, y, polygon):
    """Return True when a point lies inside the ordered polygon."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            boundary_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < boundary_x:
                inside = not inside
        previous = current
    return inside


def main():
    output = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "warehouse_map_current_sim_keepout_mask.pgm"
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
            "# warehouse_map_current_sim_keepout_mask: 0=keepout, 254=free",
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
