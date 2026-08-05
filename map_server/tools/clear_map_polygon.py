#!/usr/bin/env python3
"""Create a derived ROS map with one bounded world-coordinate polygon cleared."""

import argparse
import pathlib
import re
import sys


def read_metadata(path):
    text = path.read_text(encoding="utf-8")

    def value(name):
        match = re.search(rf"^{name}:\s*(.+?)\s*$", text, re.MULTILINE)
        if not match:
            raise ValueError(f"missing {name!r} in {path}")
        return match.group(1)

    image = value("image")
    resolution = float(value("resolution"))
    origin_text = value("origin").strip("[]")
    origin = [float(item.strip()) for item in origin_text.split(",")]
    if len(origin) < 2:
        raise ValueError("map origin must contain at least x and y")
    return text, image, resolution, origin[0], origin[1]


def read_pgm(path):
    data = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if index < len(data) and data[index] == ord("#"):
            while index < len(data) and data[index] not in (10, 13):
                index += 1
            continue
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        tokens.append(data[start:index].decode("ascii"))

    magic, width_text, height_text, maximum_text = tokens
    if magic != "P5":
        raise ValueError(f"only binary P5 PGM is supported, got {magic}")
    width, height, maximum = int(width_text), int(height_text), int(maximum_text)
    if maximum != 255:
        raise ValueError(f"only 8-bit PGM is supported, got max value {maximum}")
    if index >= len(data) or not chr(data[index]).isspace():
        raise ValueError("PGM header is not followed by a whitespace separator")
    if data[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = bytearray(data[index:])
    if len(pixels) != width * height:
        raise ValueError(
            f"PGM pixel count mismatch: expected {width * height}, got {len(pixels)}"
        )
    return width, height, pixels


def point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def parse_point(text):
    try:
        x_text, y_text = text.split(",", 1)
        return float(x_text), float(y_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("point must be X,Y") from error


def main():
    parser = argparse.ArgumentParser(
        description="Clear a bounded polygon in a ROS PGM map without changing the source map."
    )
    parser.add_argument("--input-yaml", required=True, type=pathlib.Path)
    parser.add_argument("--output-prefix", required=True, type=pathlib.Path)
    parser.add_argument(
        "--point",
        required=True,
        action="append",
        type=parse_point,
        help="polygon vertex in map-frame world coordinates as X,Y; repeat >= 3 times",
    )
    parser.add_argument(
        "--clear-unknown",
        action="store_true",
        help="also turn unknown (205) cells into free cells inside the polygon",
    )
    args = parser.parse_args()

    if len(args.point) < 3:
        parser.error("at least three --point vertices are required")

    metadata, image_name, resolution, origin_x, origin_y = read_metadata(args.input_yaml)
    input_pgm = args.input_yaml.parent / image_name
    width, height, pixels = read_pgm(input_pgm)

    min_x = min(point[0] for point in args.point)
    max_x = max(point[0] for point in args.point)
    min_y = min(point[1] for point in args.point)
    max_y = max(point[1] for point in args.point)

    changed = 0
    selected = 0
    for image_row in range(height):
        grid_y = height - 1 - image_row
        world_y = origin_y + (grid_y + 0.5) * resolution
        if not min_y <= world_y <= max_y:
            continue
        for grid_x in range(width):
            world_x = origin_x + (grid_x + 0.5) * resolution
            if not min_x <= world_x <= max_x:
                continue
            if not point_in_polygon(world_x, world_y, args.point):
                continue
            selected += 1
            offset = image_row * width + grid_x
            old_value = pixels[offset]
            should_clear = old_value < 205 or (args.clear_unknown and old_value == 205)
            if should_clear and old_value != 254:
                pixels[offset] = 254
                changed += 1

    if changed == 0:
        raise RuntimeError(
            "polygon changed zero occupied cells; check map-frame coordinates before proceeding"
        )

    output_pgm = args.output_prefix.with_suffix(".pgm")
    output_yaml = args.output_prefix.with_suffix(".yaml")
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    output_pgm.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
    )
    output_yaml.write_text(
        re.sub(
            r"^image:\s*.+?$",
            f"image: {output_pgm.name}",
            metadata,
            count=1,
            flags=re.MULTILINE,
        ),
        encoding="utf-8",
    )

    print(f"input_map={args.input_yaml}")
    print(f"output_yaml={output_yaml}")
    print(f"output_pgm={output_pgm}")
    print(f"map_size={width}x{height} resolution={resolution}")
    print(f"world_bounds=({min_x:.3f},{min_y:.3f})..({max_x:.3f},{max_y:.3f})")
    print(f"polygon_cells={selected} changed_cells={changed}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
