# Araneo: Quadruped Robot Control (ROS 2 + micro-ROS)

Control for a quadruped with two degrees of freedom per leg (coxa + tibia).
A motion command (`geometry_msgs/Twist`) passes through a gait generator and
inverse kinematics (IK), then goes to simulation (RViz) and to servos
via micro-ROS.

The robot walks forward, backward, and turns (including turning in place).

## Table of Contents

1. [Architecture](#architecture)
2. [Packages](#packages)
3. [Requirements](#requirements)
4. [Building](#building)
5. [Tests](#tests)
6. [Running](#running)
7. [Parameters](#parameters)
8. [Topics and Messages](#topics-and-messages)
9. [Running on the Real Robot](#running-on-the-real-robot)
10. [Troubleshooting](#troubleshooting)

## Architecture

```
Controller (keyboard / PS4 gamepad)
   │  geometry_msgs/Twist          /cmd_vel
   ▼
gait_node        speed (v, ω) → per-leg speed → trajectory (q1, q2)
   │             → forward kinematics (FK) → foot positions in body frame
   │  quadruped_msgs/FootTargets   /foot_targets
   ▼
ik_node          body frame → leg frame → IK → reachability check
   │             → model angles → URDF / servo angles
   ├─ sensor_msgs/JointState       /joint_cmd  → robot_state_publisher → RViz
   └─ quadruped_msgs/ServoCmd      /servo_cmd  → micro-ROS agent → USB
                                                  → microcontroller (Zephyr) → PWM → servos
```

Rule: `gait_node` does not know joint angles, and `ik_node` does not know the gait.
Foot positions (`FootTargets`) are the interface between them.

### Turning

Each leg has its own speed `s_i`, computed from the rigid-body motion equation:

```
s_i = (v − ω · arm_i) / v_ref,      arm_i = y_i / v_ref
```

where `y_i` is the lateral foot position relative to the body longitudinal axis.
Speeds are normalized to `|s_i| ≤ 1` while preserving proportions between legs,
and then ramp toward the target value within acceleration limit `a_max`.

Because there are two degrees of freedom, a leg executes only the longitudinal
component of foot speed, so the feet slip when turning.

## Packages

| Package | Contents |
|---|---|
| `quadruped_msgs` | `FootTargets` and `ServoCmd` messages. |
| `quadruped_kinematics` | FK, IK, frame transforms, model ↔ URDF angle conversion. |
| `quadruped_config` | Loading leg configuration from ROS parameters (`legs.yaml`). |
| `quadruped_gait` | Gait and trajectory generator and tests (`tools/`). |
| `quadruped_gait_node` | `gait_node`. |
| `quadruped_control` | `ik_node`, kinematics check. |
| `quadruped_description` | URDF, STL meshes, RViz config, `leg_params.py`. |
| `quadruped_bringup` | Launch file and config files (`gait.yaml`, `legs.yaml`, `ps4.config.yaml`). |

## Requirements

- Ubuntu with ROS 2 Jazzy installed,
  `colcon`, `rosdep`.
- C++17 or newer compiler, CMake.
- For visualization: `rviz2`, `robot_state_publisher`.
- For control: `teleop_twist_keyboard` or `joy` + `teleop_twist_joy`.
- For hardware operation: micro-ROS agent
  (`micro_ros_agent`) and microcontroller firmware (Zephyr + micro-ROS client).
- For plots: Python 3 with `pandas`, `matplotlib`, `numpy`.

You can check micro-ROS
[here](https://github.com/micro-ROS/micro_ros_zephyr_module/tree/jazzy).

System dependencies:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install pandas matplotlib numpy
```

## Building

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Building a single package:

```bash
colcon build --packages-select quadruped_gait
```

After changing messages (`quadruped_msgs`), rebuild the packages that
use them (`--packages-up-to quadruped_control quadruped_gait_node`).

## Tests

Gait and trajectory tests are separate programs in
`quadruped_gait/tools`. They are not part of `ros2 launch`. They write CSV files
**to the current directory**, so run them from a separate output directory.
It is convenient to create it next to the workspace and use relative paths:

```bash
cd ros2_ws
mkdir -p results && cd results
```

### Gait Generator Test

```bash
../build/quadruped_gait/gait_generator_test ../src/quadruped_bringup/config/gait.yaml
echo $?
```

For `trot` and `walk` gaits, the test runs through a scenario: standing, driving forward,
stop, driving backward, stop, turning in place, stop, left arc, stop.
It checks:

| Condition | Meaning |
|---|---|
| min legs in stance | Minimum number of legs in stance during steady-state motion (trot ≥ 2, walk ≥ 3). |
| speed signs per segment | Signs of each leg's speed match `v − ω·arm_i` (checks speed split when turning). |
| idle position in the end | After the final stop, all legs stand in the idle position. |
| q1_max, q2_max | Maximum joint angular speeds do not exceed `SERVO_LIMIT_DPS`. |

Exit code: `0` all OK, `1` a condition failed, `2` YAML load error.

Output: `gait_trot.csv`, `gait_walk.csv` (time, command, speeds `s_i`, angles
`q1_i`, `q2_i` for all four legs, in radians).

### Trajectory Test

```bash
../build/quadruped_gait/test_traj ../src/quadruped_bringup/config/gait.yaml
```

Writes `traj.csv` (`q1`, `q2` angles and foot position from forward kinematics for
two gait periods). The `w` parameter (angular speed of `q1` in stance) is not
read from YAML, but computed from `q1_m` and the duty factor of the selected gait
in `make_traj`.

### Kinematics Test

The `quadruped_control` package includes `kinematics_check`, which verifies
consistency between FK and IK (`legIK(legFK(q)) ≈ q`). Check the target name in
`quadruped_control/CMakeLists.txt`.

### Plots

From the `results` directory, after running the tests:

```bash
python3 ../src/quadruped_gait/tools/charts.py
```

The script plots:

- an overview of the entire scenario (`s_i` speeds and command) and details of
  `q1`, `q2` in steady-state time windows (one for each motion),
- saves them as PDF.

The scenario takes about 2 minutes of simulated time because the acceleration ramp
(`a_max`) is long, so full-scenario plots are for overview and details are viewed
in windows.

## Running

### RViz Simulation (No Hardware)

```bash
source ros2_ws/install/setup.bash
ros2 launch quadruped_bringup gait.launch.py
```

In a second terminal, keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i` forward, `,` backward, `j` / `l` turn in place, `u` / `o`
forward arc. The keyboard publishes only on keypress, so during
tests increase `cmd_timeout` to about 1 s, otherwise the robot stops
between key repeats.

PS4 gamepad control (`ps4.config.yaml`): `joy_node` + `teleop_twist_joy`.

In RViz you only see leg movement, because `JointState` does not move the body.
This project does not publish body odometry (for now).

Test command without a controller:

```bash
# turn left in place
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# drive forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

Monitoring:

```bash
ros2 topic hz   /foot_targets     # ~50 Hz
ros2 topic echo /foot_targets
ros2 topic echo /joint_cmd
ros2 node list
```

## Parameters

Files: `quadruped_bringup/config/gait.yaml` (gait) and `legs.yaml`
(leg geometry and mounting).

| Parameter | Description |
|---|---|
| `gait` | Gait pattern: `walk` or `trot`. |
| `q1_m` | Coxa angle amplitude [rad]. |
| `q2_st` | Tibia angle in stance (idle position) [rad]. |
| `dq2` | Tibia angle change in swing [rad]. |
| `Tsw` | Swing time [s]. Gait period follows from `Tsw` and duty factor. |
| `a_max` | Maximum speed acceleration `s` [1/s]. Determines startup and braking smoothness. |
| `rate_hz` | Gait timer frequency (default 50). |
| `speed_scale` | `linear.x` scale (recommended `1 / v_ref` when `linear.x` is in m/s). |
| `turn_scale` | `angular.z` scale (tuning turn sharpness). |
| `v_ref` | Foot speed in stance at `s = 1`, i.e. maximum robot speed [m/s]. Approximation: foot arc radius times `w`. Tune by measurement. |
| `cmd_timeout` | Time without a command after which the robot stops [s]. |
| `time_scale` | Gait time slowdown (for testing, default 1.0). |
| `ik_tol` | Allowed IK residual [m]. Above this, the command is rejected. |

## Topics and Messages

| Topic | Type | Publisher → Subscriber | QoS |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | controller → `gait_node` | default |
| `/foot_targets` | `quadruped_msgs/FootTargets` | `gait_node` → `ik_node` | `best_effort`, depth 1 |
| `/joint_cmd` | `sensor_msgs/JointState` | `ik_node` → `robot_state_publisher` | `best_effort`, depth 1 |
| `/servo_cmd` | `quadruped_msgs/ServoCmd` | `ik_node` → micro-ROS agent | `best_effort`, depth 1 |

`FootTargets`: `float32[12] position`, foot positions in the body frame,
leg order FL, FR, BL, BR, `(x, y, z)` [m] for each.

`ServoCmd`: `seq` and `angle_rad[8]`, joint angles in order
`left_front_coxa`, `left_front_tibia`, `right_front_coxa`, `right_front_tibia`,
`left_back_coxa`, `left_back_tibia`, `right_back_coxa`, `right_back_tibia`.

Publisher and subscriber must have compatible QoS (a `best_effort` publisher with a default
`reliable` subscription will not connect and will give no error—only silence).

## Troubleshooting

| Symptom | Cause |
|---|---|
| `ik_node` publishes nothing, no error | Mismatched type or QoS on `/foot_targets` (must be `FootTargets` and `best_effort` on both sides). |
| "Wrong" leg moves | Different leg order in `legs.yaml`, `names`, `FootTargets`, and `angle_rad`. Everywhere: FL, FR, BL, BR. |
| Robot turns the other way | Sign of `y` in `mount.offset` or `q1_dir`. |
| "Out of reach" warnings in `ik_node` | IK residual greater than `ik_tol`: foot path outside the 2-DOF reachable workspace or `ik_tol` too small. |
| Robot stops between keypresses | `cmd_timeout` too small for `teleop_twist_keyboard`. |
| `invalid_argument ... need to be positive` | Parameter (`q1_m`, `w`, `Tsw`) equal to zero. `w` is computed by `make_traj`, not YAML. |
| Tests do not write CSV where expected | CSVs are created in the current directory; run from the `results` directory. |
| `q2_max` above servo limit | Peak swing speed: increase `Tsw` or decrease `dq2`. |
