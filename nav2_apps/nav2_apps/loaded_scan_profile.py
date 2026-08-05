#!/usr/bin/env python3
"""Capture and compare read-only LaserScan profiles around shelf lifting."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def build_profile(samples: Sequence[Dict]) -> Dict:
    """Build per-beam validity, median and robust spread from scan samples."""
    if not samples:
        raise ValueError("at least one scan sample is required")
    first = samples[0]
    beam_count = len(first["ranges"])
    if beam_count == 0 or first["angle_increment"] == 0.0:
        raise ValueError("scan geometry is empty or invalid")
    for sample in samples:
        if (
            len(sample["ranges"]) != beam_count
            or sample["angle_min"] != first["angle_min"]
            or sample["angle_increment"] != first["angle_increment"]
        ):
            raise ValueError("scan geometry changed during capture")

    beams = []
    for index in range(beam_count):
        valid = []
        for sample in samples:
            value = sample["ranges"][index]
            if (
                math.isfinite(value)
                and sample["range_min"] <= value <= sample["range_max"]
            ):
                valid.append(value)
        beam = {
            "angle": first["angle_min"]
            + index * first["angle_increment"],
            "valid_fraction": len(valid) / len(samples),
            "median": None,
            "spread": None,
        }
        if valid:
            beam["median"] = statistics.median(valid)
            beam["spread"] = _percentile(valid, 0.90) - _percentile(
                valid, 0.10
            )
        beams.append(beam)
    return {
        "sample_count": len(samples),
        "frame_id": first["frame_id"],
        "angle_min": first["angle_min"],
        "angle_increment": first["angle_increment"],
        "range_min": first["range_min"],
        "range_max": first["range_max"],
        "beams": beams,
    }


def _intervals(indices: Sequence[int], beams: Sequence[Dict]) -> List[Dict]:
    if not indices:
        return []
    groups = []
    start = previous = indices[0]
    for index in indices[1:]:
        if index != previous + 1:
            groups.append((start, previous))
            start = index
        previous = index
    groups.append((start, previous))
    return [
        {
            "start_index": start,
            "end_index": end,
            "beam_count": end - start + 1,
            "angle_min_rad": beams[start]["angle"],
            "angle_max_rad": beams[end]["angle"],
            "angle_min_deg": math.degrees(beams[start]["angle"]),
            "angle_max_deg": math.degrees(beams[end]["angle"]),
        }
        for start, end in groups
    ]


def compare_profiles(
    baseline: Dict,
    loaded: Dict,
    near_distance: float = 1.0,
    minimum_range_change: float = 0.25,
    minimum_valid_fraction: float = 0.60,
    maximum_spread: float = 0.08,
    minimum_beams: int = 3,
) -> Dict:
    """Find stable new near returns and separately report lost beams."""
    if len(baseline["beams"]) != len(loaded["beams"]):
        raise ValueError("baseline and loaded scan sizes differ")
    if (
        baseline["angle_min"] != loaded["angle_min"]
        or baseline["angle_increment"] != loaded["angle_increment"]
    ):
        raise ValueError("baseline and loaded scan geometry differs")

    candidate_indices = []
    lost_indices = []
    for index, (before, after) in enumerate(
        zip(baseline["beams"], loaded["beams"])
    ):
        if (
            before["valid_fraction"] >= minimum_valid_fraction
            and after["valid_fraction"] < 1.0 - minimum_valid_fraction
        ):
            lost_indices.append(index)
        after_median = after["median"]
        if (
            after_median is None
            or after["valid_fraction"] < minimum_valid_fraction
            or after["spread"] is None
            or after["spread"] > maximum_spread
            or after_median > near_distance
        ):
            continue
        before_median = before["median"]
        new_or_closer = (
            before_median is None
            or before_median - after_median >= minimum_range_change
        )
        if new_or_closer:
            candidate_indices.append(index)

    candidates = [
        item
        for item in _intervals(candidate_indices, loaded["beams"])
        if item["beam_count"] >= minimum_beams
    ]
    lost = [
        item
        for item in _intervals(lost_indices, loaded["beams"])
        if item["beam_count"] >= minimum_beams
    ]
    return {
        "filter_candidates": candidates,
        "lost_beam_intervals": lost,
        "thresholds": {
            "near_distance": near_distance,
            "minimum_range_change": minimum_range_change,
            "minimum_valid_fraction": minimum_valid_fraction,
            "maximum_spread": maximum_spread,
            "minimum_beams": minimum_beams,
        },
    }


class ScanCapture(Node):
    """Read-only LaserScan collector."""

    def __init__(self, topic: str, sample_count: int) -> None:
        super().__init__("loaded_scan_profile")
        self.sample_count = sample_count
        self.samples: List[Dict] = []
        self.create_subscription(LaserScan, topic, self._scan, 10)
        self.get_logger().info(
            "read-only loaded scan profile ready; no publishers created"
        )

    def _scan(self, message: LaserScan) -> None:
        if len(self.samples) >= self.sample_count:
            return
        self.samples.append(
            {
                "frame_id": message.header.frame_id,
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "ranges": [float(value) for value in message.ranges],
            }
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--baseline")
    parser.add_argument("--sample-count", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--near-distance", type=float, default=1.0)
    parser.add_argument("--minimum-range-change", type=float, default=0.25)
    parser.add_argument("--minimum-valid-fraction", type=float, default=0.60)
    parser.add_argument("--maximum-spread", type=float, default=0.08)
    parser.add_argument("--minimum-beams", type=int, default=3)
    return parser


def main(args: Optional[Sequence[str]] = None) -> None:
    options = _parser().parse_args(args)
    if options.sample_count <= 0 or options.timeout <= 0.0:
        raise SystemExit("sample-count and timeout must be positive")
    rclpy.init()
    node = ScanCapture(options.topic, options.sample_count)
    deadline = time.monotonic() + options.timeout
    try:
        while (
            rclpy.ok()
            and len(node.samples) < options.sample_count
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.10)
        if len(node.samples) < options.sample_count:
            node.get_logger().error(
                "SCAN_PROFILE_UNCERTAIN: "
                f"received={len(node.samples)} required={options.sample_count}"
            )
            raise SystemExit(2)
        profile = build_profile(node.samples)
        document = {
            "label": options.label,
            "topic": options.topic,
            "profile": profile,
        }
        if options.baseline:
            baseline = json.loads(
                Path(options.baseline).read_text(encoding="utf-8")
            )["profile"]
            document["comparison"] = compare_profiles(
                baseline,
                profile,
                options.near_distance,
                options.minimum_range_change,
                options.minimum_valid_fraction,
                options.maximum_spread,
                options.minimum_beams,
            )
        output = Path(options.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(document, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        node.get_logger().info(
            "SCAN_PROFILE_RESULT "
            + json.dumps(
                {
                    "label": options.label,
                    "samples": profile["sample_count"],
                    "frame": profile["frame_id"],
                    "output": str(output),
                    "comparison": document.get("comparison"),
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
