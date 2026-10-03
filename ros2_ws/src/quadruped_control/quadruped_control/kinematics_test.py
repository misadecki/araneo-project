from kinematics import LEG_MOUNTS, leg_ik, leg_fk
import math

FEMUR_LEN = 0.07
FEMUR_ANG = 0.0
TIBIA_LEN = 0.1

target_body_m = (0.1, 0.0, -0.15)

for leg_name in LEG_MOUNTS:
    q1, q2 = leg_ik(*target_body_m, FEMUR_LEN, TIBIA_LEN, FEMUR_ANG)
    x_fk, y_fk, z_fk = leg_fk(q1, q2, FEMUR_LEN, TIBIA_LEN, FEMUR_ANG)

    assert math.isclose(x_fk, target_body_m[0], abs_tol=1e-3), f"x_des = \
    {target_body_m[0]}, x = {x_fk}"
    assert math.isclose(y_fk, target_body_m[1], abs_tol=1e-3), f"y_des = \
    {target_body_m[1]}, x = {y_fk}"
    assert math.isclose(z_fk, target_body_m[2], abs_tol=1e-3), f"z_des = \
    {target_body_m[2]}, z = {z_fk}"


