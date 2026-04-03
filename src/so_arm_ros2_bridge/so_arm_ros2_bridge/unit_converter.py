from __future__ import annotations

import math
from typing import Dict, Tuple

# ──────────────────────────────────────────────────────────────────────────────
# Type aliases
# ──────────────────────────────────────────────────────────────────────────────
Radians = float
Degrees = float
Percent = float   # [0, 100] — used for gripper only


# ──────────────────────────────────────────────────────────────────────────────
# Degree / radian fundamentals
# ──────────────────────────────────────────────────────────────────────────────

def deg_to_rad(deg: Degrees) -> Radians:
    return math.radians(deg)


def rad_to_deg(rad: Radians) -> Degrees:
    return math.degrees(rad)


# ──────────────────────────────────────────────────────────────────────────────
# Gripper special case
# ──────────────────────────────────────────────────────────────────────────────

def gripper_pct_to_rad(pct: Percent, joint_min_rad: Radians, joint_max_rad: Radians) -> Radians:
    """
    Map LeRobot gripper percentage [0, 100] → URDF joint radian range.

    Convention:
        pct = 0   →  joint_min_rad  (fully closed / tightest position)
        pct = 100 →  joint_max_rad  (fully open)
    """
    pct_clamped = max(0.0, min(100.0, pct))
    return joint_min_rad + (pct_clamped / 100.0) * (joint_max_rad - joint_min_rad)


def gripper_rad_to_pct(rad: Radians, joint_min_rad: Radians, joint_max_rad: Radians) -> Percent:
    """
    Map URDF radian value → LeRobot gripper percentage [0, 100].
    """
    span = joint_max_rad - joint_min_rad
    if abs(span) < 1e-9:
        return 0.0
    pct = (rad - joint_min_rad) / span * 100.0
    return max(0.0, min(100.0, pct))


# ──────────────────────────────────────────────────────────────────────────────
# Full-arm batch conversions
# ──────────────────────────────────────────────────────────────────────────────

def lerobot_to_ros(
    positions_deg: Dict[str, float],
    joint_limits: Dict[str, Tuple[float, float]],  # {name: (min_rad, max_rad)}
    gripper_name: str = "gripper",
) -> Dict[str, float]:
    """
    Convert a dict of LeRobot-normalised values to ROS2 radians.

    Body joints:  degrees → radians, then clamped to URDF limits.
    Gripper joint: percent [0,100] → radians via linear interpolation.

    Returns dict {joint_name: radians}.
    """
    result: Dict[str, float] = {}
    for name, value in positions_deg.items():
        if name in joint_limits:
            lo, hi = joint_limits[name]
        else:
            lo, hi = -math.pi, math.pi   # safe fallback

        if name == gripper_name:
            rad = gripper_pct_to_rad(value, lo, hi)
        else:
            rad = deg_to_rad(value)
            rad = max(lo, min(hi, rad))

        result[name] = rad
    return result


def ros_to_lerobot(
    positions_rad: Dict[str, float],
    joint_limits: Dict[str, Tuple[float, float]],
    gripper_name: str = "gripper",
) -> Dict[str, float]:
    """
    Convert ROS2 radians to LeRobot-normalised values.

    Body joints:  radians → degrees, clamped to URDF limits.
    Gripper joint: radians → percentage [0, 100].

    Returns dict {joint_name: degrees_or_percent}.
    """
    result: Dict[str, float] = {}
    for name, rad in positions_rad.items():
        if name in joint_limits:
            lo, hi = joint_limits[name]
        else:
            lo, hi = -math.pi, math.pi

        # Clamp first, always
        rad_clamped = max(lo, min(hi, rad))

        if name == gripper_name:
            result[name] = gripper_rad_to_pct(rad_clamped, lo, hi)
        else:
            result[name] = rad_to_deg(rad_clamped)
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Safety checks
# ──────────────────────────────────────────────────────────────────────────────

def check_joint_limits(
    positions_rad: Dict[str, float],
    joint_limits: Dict[str, Tuple[float, float]],
    tolerance: float = 1e-3,
) -> Tuple[bool, str]:
    """
    Return (ok, message).  ok=False if any joint is outside its URDF limits.
    `tolerance` (radians) allows small numerical overshoot from float conversion.
    """
    violations = []
    for name, rad in positions_rad.items():
        if name not in joint_limits:
            continue
        lo, hi = joint_limits[name]
        if rad < lo - tolerance or rad > hi + tolerance:
            violations.append(
                f"  {name}: {math.degrees(rad):.2f}° outside "
                f"[{math.degrees(lo):.2f}°, {math.degrees(hi):.2f}°]"
            )
    if violations:
        return False, "Joint limit violations:\n" + "\n".join(violations)
    return True, ""


def velocity_guard(
    current_rad: Dict[str, float],
    target_rad: Dict[str, float],
    max_delta_rad: float,
) -> Tuple[Dict[str, float], bool]:
    """
    Clamp joint targets so no joint moves more than `max_delta_rad` per step.

    Returns (clamped_targets, was_clamped).
    This is a single-step guard, not a trajectory interpolator.  For smooth
    large motions, use MoveIt2 trajectory execution instead of direct commands.
    """
    clamped = {}
    any_clamped = False
    for name, target in target_rad.items():
        current = current_rad.get(name, target)
        delta = target - current
        if abs(delta) > max_delta_rad:
            clamped[name] = current + math.copysign(max_delta_rad, delta)
            any_clamped = True
        else:
            clamped[name] = target
    return clamped, any_clamped
