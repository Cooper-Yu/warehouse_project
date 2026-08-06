#!/usr/bin/env python3
"""Real-robot full shelf transport entry point.

The mission state machine is shared with move_shelf_to_ship.py. This wrapper
only supplies real TF/footprint defaults and requires the real loading,
shipping, and return poses; initial pose is intentionally left to RViz AMCL.
"""
import argparse
import sys
from typing import Optional, Sequence

from nav2_apps import move_shelf_to_ship as mission

REAL_ODOM_FRAME = "robot_odom"
REAL_BASE_FRAME = "robot_base_footprint"
REAL_UNLOADED_FOOTPRINT = "[[0.25, 0.25], [-0.25, 0.25], [-0.25, -0.25], [0.25, -0.25]]"
REAL_LOADED_FOOTPRINT = "[[0.40, 0.40], [-0.40, 0.40], [-0.40, -0.40], [0.40, -0.40]]"
REAL_LOADING_POSE = (0.0429566167, 0.6762173176, -1.5707963268)
REAL_SHIPPING_POSE = (-2.5911982059, 1.8469729424, 1.5707963268)
REAL_RETURN_POSE = (-4.307, 0.209, 0.198)


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Run the simulation mission state machine on the real robot. "
            "Publish initial pose manually in RViz before starting."
        )
    )
    for prefix in ("loading", "shipping", "return"):
        for axis in ("x", "y", "yaw"):
            p.add_argument(f"--{prefix}-{axis}", type=float, required=False, default=(REAL_LOADING_POSE if prefix == "loading" else REAL_SHIPPING_POSE if prefix == "shipping" else REAL_RETURN_POSE)[{"x":0,"y":1,"yaw":2}[axis]])
    p.add_argument("--loaded-footprint", default=REAL_LOADED_FOOTPRINT)
    p.add_argument("--unloaded-footprint", default=REAL_UNLOADED_FOOTPRINT)
    p.add_argument("--shelf-service", default="/approach_shelf")
    p.add_argument("--frame-id", default="map")
    p.add_argument("--base-frame", default=REAL_BASE_FRAME)
    p.add_argument("--odom-frame", default=REAL_ODOM_FRAME)
    p.add_argument("--cmd-vel-topic", default="/cmd_vel")
    p.add_argument("--elevator-up-topic", default="/elevator_up")
    p.add_argument("--elevator-down-topic", default="/elevator_down")
    p.add_argument("--elevator-up-count", type=int, default=5)
    p.add_argument("--elevator-down-count", type=int, default=5)
    p.add_argument("--elevator-up-wait", type=float, default=8.0)
    p.add_argument("--elevator-down-wait", type=float, default=8.0)
    p.add_argument("--exit-distance", type=float, default=1.0)
    p.add_argument("--exit-speed", type=float, default=0.05)
    p.add_argument("--confirm-lift-accepted", action="store_true")
    p.add_argument("--stop-at-shipping", action="store_true")
    return p


def _pose_args(args, prefix: str):
    return [
        f"--{prefix}-x", str(getattr(args, f"{prefix}_x")),
        f"--{prefix}-y", str(getattr(args, f"{prefix}_y")),
        f"--{prefix}-yaw", str(getattr(args, f"{prefix}_yaw")),
    ]


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    if args.base_frame != REAL_BASE_FRAME or args.odom_frame != REAL_ODOM_FRAME:
        print("CONFIGURATION_ERROR: real TF frames must be robot_odom and robot_base_footprint", file=sys.stderr)
        return 2
    if args.elevator_up_count < 3 or args.elevator_down_count < 3:
        print("CONFIGURATION_ERROR: elevator counts must be at least 3", file=sys.stderr)
        return 2
    if not 0.0 < args.exit_speed <= 0.05:
        print("CONFIGURATION_ERROR: exit speed must be in (0, 0.05]", file=sys.stderr)
        return 2

    delegated = [
        "--frame-id", args.frame_id,
        "--base-frame", args.base_frame,
        "--odom-frame", args.odom_frame,
        "--cmd-vel-topic", args.cmd_vel_topic,
        "--shelf-service", args.shelf_service,
        "--loaded-footprint", args.loaded_footprint,
        "--unloaded-footprint", args.unloaded_footprint,
        "--elevator-up-topic", args.elevator_up_topic,
        "--elevator-down-topic", args.elevator_down_topic,
        "--elevator-up-count", str(args.elevator_up_count),
        "--elevator-down-count", str(args.elevator_down_count),
        "--elevator-up-wait", str(args.elevator_up_wait),
        "--elevator-down-wait", str(args.elevator_down_wait),
        "--exit-distance", str(args.exit_distance),
        "--exit-speed", str(args.exit_speed),
    ]
    delegated += _pose_args(args, "loading") + _pose_args(args, "shipping") + _pose_args(args, "return")
    if args.stop_at_shipping:
        delegated.append("--stop-at-shipping")
    # No --initial-* arguments: BasicNavigator consumes the manually published RViz AMCL pose.
    return mission.main(delegated)


if __name__ == "__main__":
    raise SystemExit(main())