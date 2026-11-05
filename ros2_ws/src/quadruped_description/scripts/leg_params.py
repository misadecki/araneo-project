#!/usr/bin/env python3

import sys
import numpy as np
import math
import xml.etree.ElementTree as ET

def find_elem(elem, tag):
    sub = elem.find(tag)
    if sub is None:
        raise ValueError(f"Joint '{elem.get('name')}' has not tag <{tag}>")
    return sub

def vec(elem, attr, default=0.0):
    s = elem.get(attr) if elem is not None else None
    return [float(v) for v in s.split()] if s else [default] * 3

def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx

def transform(joint):
    o = joint.find("origin")
    rpy = vec(o, "rpy")
    xyz = vec(o, "xyz")
    M = np.eye(4)
    M[:3, :3] = rot(*rpy)
    M[:3, 3] = xyz
    return M

def axis_of(joint):
    a = joint.find("axis")
    return np.array(vec(a, "xyz")) if a is not None else np.array([1.0, 0, 0])

base_link = ET.parse(sys.argv[1]).getroot()
joints = {j.get("name"): j for j in base_link.iter("joint")}
joint_of_child = {find_elem(j, "child").get("link"): j for j in joints.values()}

def pose(link):
    M = np.eye(4)
    while link in joint_of_child:
        j = joint_of_child[link]
        M = transform(j) @ M
        link = find_elem(j, "parent").get("link")
    return M

legs = [("FL", "left_front"), ("FR", "right_front"), ("BL", "left_back"), ("BR", "right_back")]
np.set_printoptions(precision=5, suppress=True)

for label, n in legs:
    coxa, tibia = joints[n + "_coxa"], joints[n + "_tibia"]
    H = pose(find_elem(coxa, "child").get("link"))
    K = pose(find_elem(tibia, "child").get("link"))
    F = pose(n + "_foot")

    offset = H[:3, 3]
    d = K[:3, 3] - offset
    ax = K[:3, :3] @ axis_of(tibia)

    perp = np.array([-ax[1], ax[0]])

    if perp @ d[:2] < 0:
        perp = -perp

    yaw = np.arctan2(perp[1], perp[0])
    c, s = np.cos(yaw), np.sin(yaw)

    def to_leg(v):
        # return: R(-yaw) * v
        return np.array([v[0] * c + v[1] * s, -v[0] * s + v[1] * c, v[2]])
    # len = sqrt(x^2 + y^2 + z^2)
    dl = to_leg(d)
    femur_len = math.hypot(dl[0], dl[2])
    femur_ang = np.arctan2(dl[2], dl[0])

    f = to_leg(F[:3, 3] - K[:3, 3])
    tibia_len = math.hypot(f[0], f[2])
    lateral = f[1] + dl[1]
    q2_zero = np.arctan2(f[0], f[2])

    coxa_axis = H[:3, :3] @ axis_of(coxa)
    tibia_axis = to_leg(ax)
    q1_sign = 1 if coxa_axis[2] > 0 else -1
    q2_sign = 1 if tibia_axis[1] > 0 else -1

    print(f"{label} ({n})")
    print(f"  offset      = {offset}")
    print(f"  yaw         = {yaw:.4f} rad ({np.degrees(yaw):.1f} deg)")
    print(f"  femur_len   = {femur_len:.4f}   femur_ang = {femur_ang:.4f}")
    print(f"  tibia_len   = {tibia_len:.4f}   dy = {lateral * 1000:.1f} mm")
    print(f"  q1_sign     = {q1_sign:+d}   (coxa in body: {coxa_axis})")
    print(f"  q2_sign     = {q2_sign:+d}   (tibia in leg: {tibia_axis}")
    print(f"  q2_zero     = {q2_zero:.4f} rad ({np.degrees(q2_zero):.1f} deg)")
    print(f"femur_ang = {femur_ang:.4f} rad ({np.degrees(femur_ang):.1f} deg)")
    print(f"lateral = {lateral:.4f}");
    print()

