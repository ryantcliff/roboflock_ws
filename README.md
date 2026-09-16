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
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```
See the [documentation](https://roboflock-documentation.readthedocs.io/en/latest/) for dependencies, hardware setup, and launch instructions.

## Run

Use the unified launch entry point for all supported modes:

```bash
# Gazebo, SLAM, Nav2, simulated GPS, and beacon following
ros2 launch bring_up robot.launch.py mode:=simulation

# Physical sensors, localization, safety, and motor control
ros2 launch bring_up robot.launch.py mode:=teleop

# Physical system with Nav2 beacon following enabled
ros2 launch bring_up robot.launch.py mode:=autonomous
```

Keyboard commands must enter through the safety controller:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel/teleop
```

The motor controller only consumes `/cmd_vel/safe`. An emergency stop is latched
until its input is released and the reset service is called:

```bash
ros2 topic pub --once /safety/emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

The systemd unit is an instance template where the instance is the Linux user:

```bash
sudo cp install/bring_up/share/bring_up/systemd/robot@.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot@"${USER}".service
```
