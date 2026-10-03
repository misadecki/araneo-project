import math

LEG_MOUNTS = {
    "front_left" : {"offset": (0.0, 0.0, 0.0), "yaw": 0.0},
    "front_right" : {"offset": (0.0, 0.0, 0.0), "yaw": 0.0},
    "back_left" : {"offset": (0.0, 0.0, 0.0), "yaw": math.pi},
    "back_right" : {"offset": (0.0, 0.0, 0.0), "yaw": math.pi},
}

def body_to_leg_frame(x_body, y_body, z_body, leg_name):
    x_off, y_off, z_off = LEG_MOUNTS[leg_name]["offset"]
    yaw = LEG_MOUNTS[leg_name]["yaw"]

    dx, dy, dz = x_body - x_off, y_body - y_off, z_body - z_off

    x_leg = dx * math.cos(yaw) + dy * math.sin(yaw)
    y_leg = -dx * math.sin(yaw) + dy * math.cos(yaw)

    return x_leg, y_leg, dz

def leg_ik(x, y, z, femur_len, tibia_len, femur_ang):
    q1 = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)

    knee_r = femur_len * math.cos(femur_ang)
    knee_z = femur_len * math.sin(femur_ang)

    dr = r - knee_r
    dz = z - knee_z

    dist = math.sqrt(dr**2 + dz**2)

    if dist > tibia_len:
        scale = tibia_len / dist
        dr *= scale
        dz *= scale

    q2 = math.atan2(r, z)

    return q1, q2

def leg_fk(q1, q2, femur_len, tibia_len, femur_ang):
    knee_r = femur_len * math.cos(femur_ang)
    knee_z = femur_len * math.sin(femur_ang)

    dr = knee_r * tibia_len * math.sin(q2)
    dz = knee_z * tibia_len * math.cos(q2)

    r = knee_r + dr
    z = knee_z + dz

    x = r * math.cos(q1)
    y = r * math.sin(q1)

    return x, y, z
