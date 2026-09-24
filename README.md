# Roboflock — Autonomous Mobile Robot (ROS 2)

The complete ROS 2 workspace for **Roboflock**, an autonomous four-wheel differential-drive robot that navigates in real time by fusing LiDAR and GPS. This repo holds the source; the full write-up, setup, and architecture live in the documentation site below.

**[Read the full documentation »](https://roboflock-documentation.readthedocs.io/en/latest/)**

<!-- Add a short demo clip or photo of the robot here:
![Roboflock](docs/roboflock.jpg) -->

Here's a demo of the robot running SLAM:
https://github.com/user-attachments/assets/d75cb297-7618-43ef-b84b-9466528e4aed

## What it does
- Autonomous navigation with **Nav2**, **SLAM**, and `robot_localization`, fusing **LiDAR + GPS**.
- Custom **ROS 2 nodes** for 2D LiDAR visualization and a differential-drive controller interfaced with **ODrive** motor modules.
- Microcontroller-driven **ultrasonic sensing** as an independent obstacle-detection failsafe.
- Runs on an **NVIDIA Jetson Orin Nano** with a custom-wired power and motor-driver stack.

## Stack
ROS 2 · C++ / Python · Nav2 · SLAM · robot_localization · ODrive · LiDAR · GPS · NVIDIA Jetson Orin Nano

## Repository layout
```
src/    ROS 2 packages (nodes, controllers, launch, config)
```

## Build
Standard ROS 2 colcon workspace:
```bash
cd roboflock_ws
colcon build
source install/setup.bash
```
See the [documentation](https://roboflock-documentation.readthedocs.io/en/latest/) for dependencies, hardware setup, and launch instructions.

## Debug navigation without peripherals (ROS 2 Humble)

Install the navigation dependencies once:
```bash
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization
```

Build and start only the navigation processes, leaving lifecycle nodes unconfigured:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bring_up
source install/setup.bash
ros2 launch bring_up bringup.launch.py hardware:=false enable_mapping:=false autostart:=false
```

This mode does not start sensor drivers, GPS receivers, localization, beacon
commands, robot state publisher, or motors. It does not supply fake sensor data
or transforms. Navigation cannot become operational until those inputs exist.
Inspect the processes from a second sourced terminal:
```bash
ros2 node list
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```
Both lifecycle queries should report `unconfigured`. This checks process startup,
not controller configuration, path planning, or obstacle avoidance.

`nav2.launch.py` can also be launched directly with `autostart:=false` for the
same navigation-only inspection. It loads `config/nav2_params.yaml`; override it
with `params_file:=/absolute/path/to/params.yaml`.

Mapping is owned by the main bringup, not the navigation launcher:
- `slam:=true` (default) starts one SLAM Toolbox instance.
- `slam:=false map:=/absolute/path/to/map.yaml` starts a lifecycle-managed map
  server instead. It does not start AMCL; localization must align the robot's
  pose with the saved map. The map must be aligned to the GPS datum; an arbitrary
  SLAM map is not automatically georeferenced.
- `enable_mapping:=false` starts neither map source (for debugging or an
  externally supplied map).

`hardware:=true` remains the normal bringup default. Motor startup remains
excluded until command timeout and stop protections are implemented. For the
currently disconnected Jetson, use the full debug command above. `use_sim_time`
is forwarded to navigation and mapping; leave it false without a `/clock` source.


## Localization ownership and hardware-free integration tests

The main bringup selects localization from `slam`:

| Mode | `map -> odom` owner | `odom -> base_link` owner |
| --- | --- | --- |
| `slam:=true` | SLAM Toolbox | Local EKF |
| `slam:=false` | GPS/global EKF | Local EKF |

RF2O supplies `/odom_rf2o` without publishing TF through the bringup-owned launch
and config. The upstream RF2O standalone launch still publishes TF; do not run
it alongside the local EKF. Nav2's controller, navigator, and velocity smoother
use `/odometry/local`. In GPS mode, `navsat_transform_node` reads its matching
YAML section and the global EKF fuses GPS position with IMU and RF2O motion.
`enable_mapping:=false` only disables the map source: it does not change the
localization choice. With `hardware:=false`, no EKFs or sensor drivers start.

Mapping mode deliberately does not run the global GPS EKF or navsat transform.
Automatic beacon goals are excluded from main bringup until their coordinates
are converted into the same map datum (priority 3). Motors remain excluded.

Run the repeatable tests from the workspace root after building and sourcing:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bring_up
source install/setup.bash

# Real Nav2 with a synthetic map, scan, odometry, and TF:
python src/bring_up/scripts/nav2_smoke_test.py

# Real local EKF driven by synthetic IMU and laser odometry:
python src/bring_up/scripts/nav2_smoke_test.py --localization local

# Both real EKFs and navsat_transform, also fed synthetic GPS fixes:
python src/bring_up/scripts/nav2_smoke_test.py --localization gps
```

Each test uses localhost-only ROS domain 87, refuses an occupied domain, and
shuts down its own processes afterward. Use `--domain-id 88` if needed. Your
normal ROS session can remain running in its original domain. Logs default to
`/tmp/roboflock-nav2-smoke.log`; override with `--log /tmp/my-test.log`.

Success requires all seven Nav2 lifecycle nodes to become active, a successful
path to `(1, 0)` in the test map, the expected TF chain and publisher set, and
filtered odometry in EKF modes. The fixture represents a stationary robot in an
empty environment; it does not execute a drive goal or validate collision
avoidance, tracking accuracy, SLAM scan matching, or motor control.

Before real GPS operation, measure and publish the GPS antenna transform from
`base_link` to `gps`, verify IMU mounting and ENU heading, and calibrate magnetic
declination/yaw offset for the deployment location. The fixture's identity
sensor transforms and GPS coordinates are test data, not hardware calibration.
A saved map also requires a consistent datum/orientation across restarts; the
current automatic datum is not a persisted map alignment. These remain hardware
commissioning requirements, even when the integration tests pass.
