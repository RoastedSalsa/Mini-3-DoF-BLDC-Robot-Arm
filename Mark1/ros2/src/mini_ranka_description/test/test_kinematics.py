#!/usr/bin/env python3
"""Assert that mini_ranka.urdf agrees with the firmware's kinematics.

The URDF is CAD geometry reworked onto the firmware's joint convention (see the
header comment in urdf/mini_ranka.urdf). That rework is exactly the kind of
thing that silently goes wrong — a flipped axis still *looks* plausible in RViz
while the twin quietly mirrors the arm. So we check it numerically instead:
walk the URDF's link tree and compare the pose of tool_link against fk() from
firmware/lib/ArmKinematics/ArmKinematics.cpp, ported below.

    python3 test/test_kinematics.py        # standalone
    pytest test/test_kinematics.py         # or under colcon test

Run this after any Onshape re-export.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

# --- firmware/include/config.h ------------------------------------------------
L1, L2, L3 = 0.0843, 0.2, 0.2
JOINT_LIMIT = 2.0        # J1_MIN..J3_MAX
BELT_RATIO = 3.0         # JOINT_GEAR[1]

URDF = os.path.join(os.path.dirname(__file__), "..", "urdf", "mini_ranka.urdf")

# Sub-millimetre. The CAD carries small lateral offsets (~2 mm of link-2 Y) that
# fk() does not model, so we compare along the axes fk() actually claims.
TOL = 1e-4


# --- port of ArmKinematics.cpp fk() ------------------------------------------
def fk_firmware(t1, t2, t3):
    r = L2 * math.cos(t2) + L3 * math.cos(t2 + t3)
    return np.array([
        math.cos(t1) * r,
        math.sin(t1) * r,
        L1 + L2 * math.sin(t2) + L3 * math.sin(t2 + t3),
    ])


# --- minimal URDF forward kinematics -----------------------------------------
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def axis_R(axis, theta):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def load(path):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o, ax, mim = j.find("origin"), j.find("axis"), j.find("mimic")
        joints[j.get("name")] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": np.array([float(v) for v in (o.get("xyz", "0 0 0").split()
                                                if o is not None else "0 0 0".split())]),
            "rpy": np.array([float(v) for v in (o.get("rpy", "0 0 0").split()
                                                if o is not None else "0 0 0".split())]),
            "axis": np.array([float(v) for v in ax.get("xyz").split()])
            if ax is not None else np.array([0.0, 0.0, 1.0]),
            "limit": j.find("limit"),
            "mimic": (mim.get("joint"), float(mim.get("multiplier", 1.0)),
                      float(mim.get("offset", 0.0))) if mim is not None else None,
        }
    return [l.get("name") for l in root.findall("link")], joints


def fk_urdf(links, joints, q):
    """World transforms of every link, resolving mimic joints from q."""
    angles = dict(q)
    for name, j in joints.items():
        if j["mimic"]:
            ref, mult, off = j["mimic"]
            angles[name] = mult * q.get(ref, 0.0) + off

    world = {l: None for l in links}
    for r in set(links) - {j["child"] for j in joints.values()}:
        world[r] = np.eye(4)

    for _ in range(len(joints) + 1):
        for name, j in joints.items():
            if world.get(j["parent"]) is not None and world.get(j["child"]) is None:
                M = np.eye(4)
                M[:3, :3] = rpy_to_R(*j["rpy"])
                M[:3, 3] = j["xyz"]
                if j["type"] in ("revolute", "continuous"):
                    R = np.eye(4)
                    R[:3, :3] = axis_R(j["axis"], angles.get(name, 0.0))
                    M = M @ R
                world[j["child"]] = world[j["parent"]] @ M
    return world


# --- tests -------------------------------------------------------------------
CASES = [
    (0.0, 0.0, 0.0),
    (0.5, 0.0, 0.0),
    (-1.2, 0.0, 0.0),
    (0.0, 0.6, 0.0),
    (0.0, 0.0, -0.8),
    (0.0, 0.4, -0.9),
    (0.7, -0.3, 0.5),
    (-1.5, 1.1, -1.3),
    (1.9, -1.9, 1.9),
]


def test_tool_matches_firmware_fk():
    links, joints = load(URDF)
    worst = 0.0
    for t1, t2, t3 in CASES:
        w = fk_urdf(links, joints, {"joint1": t1, "joint2": t2, "joint3": t3})
        got = w["tool_link"][:3, 3]
        want = fk_firmware(t1, t2, t3)
        err = float(np.max(np.abs(got - want)))
        worst = max(worst, err)
        assert err < TOL, (
            f"q=({t1}, {t2}, {t3})  urdf={np.round(got, 6)}  "
            f"fk()={np.round(want, 6)}  err={err:.2e}"
        )
    print(f"tool_link matches fk() across {len(CASES)} configs, worst {worst:.2e} m")


def test_zero_pose_is_arm_along_plus_x():
    """The whole point of the ${pi} yaw on joint1."""
    links, joints = load(URDF)
    w = fk_urdf(links, joints, {})
    got = w["tool_link"][:3, 3]
    want = np.array([L2 + L3, 0.0, L1])
    assert np.max(np.abs(got - want)) < TOL, f"zero pose {got}, expected {want}"
    print(f"zero pose = {np.round(got, 6)}  (arm straight out along +X at z=L1)")


def test_joint_axes_match_ik_frame():
    """+q1 CCW about world +Z; +q2/+q3 raise the arm (axis (0,-1,0))."""
    links, joints = load(URDF)
    w = fk_urdf(links, joints, {})
    expected = {"joint1": [0, 0, 1], "joint2": [0, -1, 0], "joint3": [0, -1, 0]}
    for name, want in expected.items():
        j = joints[name]
        got = w[j["parent"]][:3, :3] @ rpy_to_R(*j["rpy"]) @ j["axis"]
        assert np.max(np.abs(got - np.array(want, float))) < 1e-4, \
            f"{name} world axis {np.round(got, 5)}, expected {want}"
        print(f"{name} world axis = {np.round(got, 5)}")


def test_link_lengths_match_config_h():
    """Axis-to-axis distances must equal L1/L2/L3, or the CAD has drifted."""
    links, joints = load(URDF)
    w = fk_urdf(links, joints, {})
    o = {n: (w[joints[n]["parent"]] @ np.append(joints[n]["xyz"], 1.0))[:3]
         for n in ("joint1", "joint2", "joint3")}
    tool = w["tool_link"][:3, 3]

    assert abs(o["joint2"][2] - L1) < 1e-3, f"L1: got {o['joint2'][2]}, want {L1}"
    # Compare along X only: fk() models a planar arm, the CAD has ~2 mm of
    # lateral (Y) build offset that the firmware deliberately ignores.
    assert abs(abs(o["joint3"][0] - o["joint2"][0]) - L2) < 1e-3, \
        f"L2: got {abs(o['joint3'][0] - o['joint2'][0])}, want {L2}"
    assert abs(abs(tool[0] - o["joint3"][0]) - L3) < 1e-3, \
        f"L3: got {abs(tool[0] - o['joint3'][0])}, want {L3}"
    print(f"L1={o['joint2'][2]:.4f}  L2={abs(o['joint3'][0] - o['joint2'][0]):.4f}  "
          f"L3={abs(tool[0] - o['joint3'][0]):.4f}")


def test_limits_and_belt_match_config_h():
    _, joints = load(URDF)
    for name in ("joint1", "joint2", "joint3"):
        lim = joints[name]["limit"]
        assert lim is not None, f"{name} has no <limit> (software travel limits)"
        assert abs(float(lim.get("lower")) + JOINT_LIMIT) < 1e-9
        assert abs(float(lim.get("upper")) - JOINT_LIMIT) < 1e-9
    ref, mult, _ = joints["joint2_belt"]["mimic"]
    assert ref == "joint2" and abs(mult - BELT_RATIO) < 1e-9, \
        f"belt mimics {ref} x{mult}, expected joint2 x{BELT_RATIO}"
    print(f"limits +/-{JOINT_LIMIT} rad on joint1..3; belt mimics joint2 x{mult}")


if __name__ == "__main__":
    for fn in (test_zero_pose_is_arm_along_plus_x, test_joint_axes_match_ik_frame,
               test_link_lengths_match_config_h, test_limits_and_belt_match_config_h,
               test_tool_matches_firmware_fk):
        print(f"\n--- {fn.__name__} ---")
        fn()
    print("\nAll kinematics checks passed.")
