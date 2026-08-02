"""Pure reflective-leg geometry extraction shared by shelf tools."""

import math
from dataclasses import dataclass
from typing import List, Optional

from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class LegCandidate:
    x: float
    y: float
    low_index: int
    high_index: int
    size: int


@dataclass(frozen=True)
class LegPairMeasurement:
    frame_id: str
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    midpoint_x: float
    midpoint_y: float
    lateral_separation: float
    euclidean_separation: float


def _candidate_at(
    scan: LaserScan, index: int, source: LegCandidate
) -> LegCandidate:
    angle = scan.angle_min + index * scan.angle_increment
    distance = scan.ranges[index]
    return LegCandidate(
        distance * math.cos(angle),
        distance * math.sin(angle),
        source.low_index,
        source.high_index,
        source.size,
    )


def _candidate(scan: LaserScan, cluster: List[int]) -> LegCandidate:
    source = LegCandidate(0.0, 0.0, cluster[0], cluster[-1], len(cluster))
    return _candidate_at(scan, cluster[len(cluster) // 2], source)


def _best_pair(
    candidates: List[LegCandidate],
    min_leg_separation: float,
    max_x_difference: float,
):
    best = None
    best_score = math.inf
    for index, first in enumerate(candidates):
        for second in candidates[index + 1:]:
            separation = abs(first.y - second.y)
            x_difference = abs(first.x - second.x)
            if (
                separation < min_leg_separation
                or x_difference > max_x_difference
            ):
                continue
            score = abs((first.y + second.y) / 2.0) + x_difference
            if score < best_score:
                best = (first, second)
                best_score = score
    return best


def detect_leg_pair(
    scan: LaserScan,
    intensity_threshold: float,
    min_cluster_size: int,
    max_x_difference: float,
    min_leg_separation: float,
) -> Optional[LegPairMeasurement]:
    """Return the selected reflective leg pair without publishing anything."""
    if not scan.ranges or not scan.intensities:
        return None

    count = min(len(scan.ranges), len(scan.intensities))
    clusters: List[List[int]] = []
    current: List[int] = []
    for index in range(count):
        intensity = scan.intensities[index]
        distance = scan.ranges[index]
        valid = (
            math.isfinite(intensity)
            and intensity >= intensity_threshold
            and math.isfinite(distance)
            and scan.range_min <= distance <= scan.range_max
        )
        if valid:
            current.append(index)
        else:
            if len(current) >= min_cluster_size:
                clusters.append(current)
            current = []
    if len(current) >= min_cluster_size:
        clusters.append(current)

    candidates = []
    for cluster in clusters:
        candidate = _candidate(scan, cluster)
        if candidate.x > 0.0:
            candidates.append(candidate)

    pair = _best_pair(
        candidates,
        min_leg_separation=min_leg_separation,
        max_x_difference=max_x_difference,
    )
    if pair is None:
        return None

    first, second = pair
    first_index = (
        first.high_index if first.y < second.y else first.low_index
    )
    second_index = (
        second.low_index if first.y < second.y else second.high_index
    )
    first = _candidate_at(scan, first_index, first)
    second = _candidate_at(scan, second_index, second)
    left, right = sorted((first, second), key=lambda item: item.y)
    dx = right.x - left.x
    dy = right.y - left.y
    return LegPairMeasurement(
        frame_id=scan.header.frame_id,
        left_x=left.x,
        left_y=left.y,
        right_x=right.x,
        right_y=right.y,
        midpoint_x=(left.x + right.x) / 2.0,
        midpoint_y=(left.y + right.y) / 2.0,
        lateral_separation=abs(dy),
        euclidean_separation=math.hypot(dx, dy),
    )
