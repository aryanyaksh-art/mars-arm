"""Arm kinematics. solve_ik() is a line-for-line mirror of solveIK() in the firmware."""
import math

from . import config as c


def solve_ik(x, y, z):
    """Servo angles (base, shoulder, elbow) that put the jaws at x y z, or None if out of reach."""
    r = math.hypot(x, y)
    if r <= abs(c.GRIP_Y) + 1:
        return None
    yaw = math.atan2(y, x) - math.asin(c.GRIP_Y / r)  # turn so the offset jaws land on the target
    rp = math.sqrt(r * r - c.GRIP_Y ** 2)             # reach within the arm's vertical plane
    zw = z - c.BASE_H
    d = math.hypot(rp, zw)
    if d > c.L1 + c.L2 - 2 or d < abs(c.L1 - c.L2) + 2:
        return None
    upper = math.degrees(math.atan2(zw, rp) + math.acos((c.L1 ** 2 + d * d - c.L2 ** 2) / (2 * c.L1 * d)))
    inner = math.degrees(math.acos((c.L1 ** 2 + c.L2 ** 2 - d * d) / (2 * c.L1 * c.L2)))
    if not (c.UPPER_LIMITS[0] <= upper <= c.UPPER_LIMITS[1] and c.INNER_LIMITS[0] <= inner <= c.INNER_LIMITS[1]):
        return None
    angles = (90 + math.degrees(yaw), upper, inner)
    if any(a < 0 or a > 180 for a in angles):
        return None
    return angles


def forearm_point(base, shoulder, elbow, along, above=0.0, side=0.0):
    """Position (x, y, z) of a point fixed to the forearm: `along` mm from the elbow axis down the
    forearm, `above` mm off its axis (toward the sky when the forearm is level), `side` mm sideways."""
    yaw, upper, inner = math.radians(base - 90), math.radians(shoulder), math.radians(elbow)
    forearm = upper + inner - math.pi
    r = c.L1 * math.cos(upper) + along * math.cos(forearm) - above * math.sin(forearm)
    z = c.BASE_H + c.L1 * math.sin(upper) + along * math.sin(forearm) + above * math.cos(forearm)
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (r * cos - side * sin, r * sin + side * cos, z)


def forearm_axes(base, shoulder, elbow):
    """Unit vectors along the forearm and sideways across it, as (x, y, z)."""
    yaw = math.radians(base - 90)
    forearm = math.radians(shoulder + elbow) - math.pi
    return ((math.cos(forearm) * math.cos(yaw), math.cos(forearm) * math.sin(yaw), math.sin(forearm)),
            (-math.sin(yaw), math.cos(yaw), 0.0))


def forward(base, shoulder, elbow):
    """Elbow and jaw positions (x, y, z) for the given servo angles."""
    return forearm_point(base, shoulder, elbow, 0.0), forearm_point(base, shoulder, elbow, c.L2, 0.0, c.GRIP_Y)


def tag_position(base, shoulder, elbow):
    """Centre of the calibration tag on the forearm."""
    return forearm_point(base, shoulder, elbow, c.TAG_ALONG_MM, c.TAG_ABOVE_MM, c.TAG_SIDE_MM)
