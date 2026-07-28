"""Host-side port of the arm's kinematics and joint-frame mapping.

Line-for-line equivalent of firmware/lib/ArmKinematics/ArmKinematics.cpp and the
frame-mapping constants in firmware/include/config.h. Kept here so the mock arm
and the live bridge agree with the firmware — and with each other — about what
q1/q2/q3 mean.

The firmware is the authority. If any constant below changes there, change it
here too; mini_ranka_description/test/test_kinematics.py checks the URDF against
the same numbers and will fail loudly if the three drift apart.
"""

import math

# --- firmware/include/config.h ------------------------------------------------
L1 = 0.0843   # base plane -> shoulder-pitch axis [m]
L2 = 0.2000   # upper arm  [m]
L3 = 0.2000   # forearm    [m]

# Encoder-frame relationship, per joint:
#   DIR  = encoder count direction vs the IK joint frame (+1 same, -1 opposite)
#   GEAR = motor-side reduction (J2 belt is 3:1; J1/J3 are direct drive)
JOINT_DIR = (-1.0, 1.0, 1.0)
JOINT_GEAR = (1.0, 3.0, 1.0)

# Software travel limits, applied in the firmware before move().
JOINT_LIMITS = ((-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0))

# The names the URDF uses. Order matches q1/q2/q3 throughout.
JOINT_NAMES = ("joint1", "joint2", "joint3")

# --- default trajectory (TRAJ_PX / TRAJ_PY / TRAJ_PZ) -------------------------
TRAJ_POINTS = ((0.2, 0.1, 0.15),
               (0.2, 0.1, 0.25),
               (0.2, -0.1, 0.25),
               (0.2, -0.1, 0.15))
TRAJ_SEGMENT_TIME_S = 2.0


def fk(t1: float, t2: float, t3: float) -> tuple[float, float, float]:
    """Joint angles [rad] -> tool position (x, y, z) [m]."""
    r = L2 * math.cos(t2) + L3 * math.cos(t2 + t3)
    return (math.cos(t1) * r,
            math.sin(t1) * r,
            L1 + L2 * math.sin(t2) + L3 * math.sin(t2 + t3))


def ik(x: float, y: float, z: float) -> tuple[float, float, float] | None:
    """Tool position [m] -> joint angles (t1, t2, t3) [rad], elbow-down.

    Returns None when the point is out of reach. The firmware's ik() has no such
    guard and would hand back NaN (main.cpp checks reachability separately before
    calling it); returning None here keeps a bad waypoint from poisoning the twin
    with NaN joint states, which RViz renders as the arm vanishing.
    """
    r = math.hypot(x, y)
    t1 = math.atan2(y, x)

    c3 = (r * r + (z - L1) ** 2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if not -1.0 <= c3 <= 1.0:
        return None
    # Negative sine branch = elbow-down, matching the firmware's -sqrt(...).
    t3 = math.atan2(-math.sqrt(1.0 - c3 * c3), c3)
    t2 = math.atan2(z - L1, r) - math.atan2(L3 * math.sin(t3),
                                            L2 + L3 * math.cos(t3))
    return (t1, t2, t3)


def motor_to_joint(index: int, measured: float, home_offset: float) -> float:
    """Encoder reading [rad] -> IK-frame joint angle [rad].

    Mirrors main.cpp:
        q = JOINT_DIR[i] * (meas[i] - home_offset[i]) / JOINT_GEAR[i]
    """
    return JOINT_DIR[index] * (measured - home_offset) / JOINT_GEAR[index]


def joint_to_motor(index: int, q: float, home_offset: float) -> float:
    """IK-frame joint angle [rad] -> commanded encoder-frame angle [rad]."""
    return JOINT_DIR[index] * JOINT_GEAR[index] * q + home_offset


def clamp_to_limits(q: tuple[float, float, float]) -> tuple[float, float, float]:
    """Clamp joint angles to the firmware's software travel limits."""
    return tuple(min(max(v, lo), hi)
                 for v, (lo, hi) in zip(q, JOINT_LIMITS))  # type: ignore[return-value]


def trajectory_at(elapsed_s: float,
                  points=TRAJ_POINTS,
                  segment_time_s: float = TRAJ_SEGMENT_TIME_S,
                  smooth: bool = False) -> tuple[float, float, float]:
    """Cartesian setpoint at `elapsed_s`, cycling through `points`.

    Same behaviour as lib/Trajectory: fixed time per segment, wraps around, and
    is time-based rather than tick-based. `smooth` selects the cubic smoothstep
    easing (TrajectoryEasing::SMOOTH); the firmware defaults to LINEAR.
    """
    n = len(points)
    if n == 0:
        return (0.0, 0.0, 0.0)
    if segment_time_s <= 0.0:
        return points[0]

    total = elapsed_s / segment_time_s
    i0 = int(total) % n
    i1 = (i0 + 1) % n
    a = total - math.floor(total)
    s = a * a * (3.0 - 2.0 * a) if smooth else a

    p0, p1 = points[i0], points[i1]
    return tuple(p0[k] + s * (p1[k] - p0[k]) for k in range(3))  # type: ignore[return-value]
