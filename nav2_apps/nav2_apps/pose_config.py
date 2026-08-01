"""Pure pose-configuration validation for navigation applications."""

from typing import Any, Optional, Tuple


def optional_initial_pose(args: Any) -> Optional[Tuple[float, float, float]]:
    """Return a complete optional initial pose or reject partial input."""
    values = (args.initial_x, args.initial_y, args.initial_yaw)
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError(
            "--initial-x, --initial-y, and --initial-yaw must be supplied "
            "together"
        )
    return values
