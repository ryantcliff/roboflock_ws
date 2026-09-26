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
- **Beacon following**: drives after a person carrying a GPS beacon and stops 3 m short.
- **Return home** to a Meshtastic home station, even after the station is moved.
- Runs on an **NVIDIA Jetson Orin Nano** with a custom-wired power and motor-driver stack.

## Stack
ROS 2 Humble · C++ / Python · Nav2 · SLAM · robot_localization · ODrive · LiDAR · GPS (u-blox) · HC-12 · Meshtastic · NVIDIA Jetson Orin Nano

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

`hardware:=true` remains the normal bringup default. Motors start only in
`follow:=true` mode (see below), behind the joystick e-stop. `use_sim_time`
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
localization choice. With `hardware:=false`, no EKFs or sensor drivers start
(unless `sim:=true`, which runs the EKFs on simulated sensors).

Mapping mode deliberately does not run the global GPS EKF or navsat transform.

Run the repeatable tests from the workspace root after building and sourcing:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bring_up
source install/setup.bash

# Real Nav2 with a synthetic map, scan, odometry, and TF:
python src/bring_up/scripts/nav2_smoke_test.py

# Real local EKF driven by synthetic IMU and laser odometry:
python src/bring_up/scripts/nav2_smoke_test.py --localization local

# Both real EKFs and navsat_transform, fed synthetic GPS fixes; also drives the
# robot 10 m and checks /odometry/global and /odometry/gps agree within 0.2 m:
python src/bring_up/scripts/nav2_smoke_test.py --localization gps

# Beacon GPS -> map conversion (beacon_goalpose + navsat /fromLL):
python src/beacon_pkg/scripts/beacon_smoke_test.py

# Full beacon following and return home on a simulated robot (~5 min):
python src/bring_up/scripts/follow_smoke_test.py --beacon-speed 1.2

# Unit tests (e-stop, GPS monitor, mission logic, Meshtastic parsing, ultrasonic stop):
python3 -m pytest src/bring_up/test/test_e_stop.py src/bring_up/test/test_gps_monitor.py \
  src/bring_up/test/test_mission_manager.py src/bring_up/test/test_meshtastic_bridge.py \
  src/bring_up/test/test_ultrasonic_estop.py src/bring_up/test/test_bno085_imu.py
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


## Beacon following and return home

The robot follows a person (Tom) carrying a GPS beacon, and can drive back to a
home station that may be moved at any time. Both run on the GPS-aligned `map`
frame, so they require `slam:=false`.

```bash
# On the robot (hardware):
ros2 launch bring_up bringup.launch.py follow:=true slam:=false

# Also connect the Meshtastic home station:
# (defaults: meshtastic_port:=/dev/meshtastic home_node_id:=!6c743480)
ros2 launch bring_up bringup.launch.py follow:=true slam:=false station:=true

# Hardware-free simulation (fake robot, sensors and walking beacon; real EKFs and Nav2):
ros2 launch bring_up bringup.launch.py follow:=true slam:=false hardware:=false sim:=true
```

Change mode from the Meshtastic app (text message) or from the Jetson:
```bash
ros2 topic pub --once /station/command std_msgs/msg/String "data: home"   # follow | home | stop | status
```

| Piece | What it does |
| --- | --- |
| HC-12 radio (`beacon_receiver.launch.py`) | Beacon GPS NMEA into `/dev/ttyTHS1`, published as `/gps/beacon/fix` |
| `beacon_goalpose` / `home_goalpose` | Convert beacon / home lat-lon to `map` via navsat `/fromLL`; targets over 20 m away are pulled in so they stay on the 50 m rolling costmap |
| `mission_manager` | Modes `idle`, `follow` (stop 3 m short, `follow_beacon.xml`), `home` (stop 1.5 m short, `return_home.xml`); cancels the Nav2 goal when the target goes stale (3 s) or the mode changes; status on `/robot/status` |
| `meshtastic_bridge` (`station:=true`) | Home station positions to `/home/fix`, republished at 1 Hz until 30 min old; text commands to `/station/command`; status back over the mesh |
| `twist_mux` | Nav2 `/cmd_vel`, joystick `/cmd_vel_joy`, ultrasonic `/cmd_vel_estop` in; `/cmd_vel_mux` out to `diff_drive_controller` |
| `e_stop` | PS4 Cross stops, Options arms; losing the joystick for 0.5 s stops. Motors stay idle until armed |
| `diff_drive_controller` | ODrives; idles on e-stop, commands zero after 0.5 s without `/cmd_vel` |

Nav2 uses Regulated Pure Pursuit at 10 Hz (MPPI was too heavy for the Orin
Nano), up to 1.2 m/s, braking at 1.5 m/s². Acceleration is 0.5 m/s², matching
the ODrive velocity ramp, so the robot lags a steady walker and catches up
when they stop.

### Hardware assignment
- **Robot:** u-blox ZED-F9P (dual-band) on USB (`/dev/ublox_gps`, see
  `config/robot_gps.yaml`), HC-12 receiver on `/dev/ttyTHS1`, Meshtastic node on USB.
- **Beacon (Tom):** NEO-M8P, NMEA GGA at 9600 baud into the HC-12 transmitter.
  Tom also carries a Meshtastic handheld paired with their phone for commands.
- **Home station:** Meshtastic node with GPS on, smart position broadcast on, and
  channel position precision 32 bits (lower precision rounds the position off).
- **Meshtastic radios** (LILYGO T-Beams, all role CLIENT):

  | Label | Node | Use |
  | --- | --- | --- |
  | ROBT | `!6c73d700` | Robot, USB to the Jetson (`/dev/meshtastic`), GPS off |
  | HOME | `!6c743480` | Home station, GPS on, smart broadcast |
  | TOM | `!bb80e074` | Tom's handheld, paired with the phone app; allowed to send commands |

  Channel 0 on all three is the private `roboflock` channel (position precision
  32). Its key is on the Jetson in `~/.config/roboflock/meshtastic_primary_psk.b64`
  (not in git). Channel 1 `fromJetson` carries robot status.
- The Python `meshtastic` library is a PyPI dependency (`pip install --user meshtastic==2.7.11`).

### IMU: no compass yet
The IMU board on the robot is an **MPU-6050** (WHO_AM_I `0x68`), not an MPU-9250.
It has no magnetometer, so `mpu9250driver` publishes a fixed yaw (about -135°) and
logs `Remote I/O error` for every compass read. GPS localization takes its absolute
heading from the IMU, so **don't trust GPS localization or follow mode until a
BNO085 is fitted**.

- Wiring (either board): VCC to **3.3 V** (pin 1, not 5 V), GND to pin 6, SDA to pin
  27, SCL to pin 28 (`/dev/i2c-1`). Mount it flat with X forward and Y left.
- BNO085 support is ready but untested on hardware (`bring_up/bno085_imu.py`, fuses
  the magnetometer on the chip). Install its libraries, then select it:
  ```bash
  pip install --user adafruit-circuitpython-bno08x adafruit-extended-bus
  ros2 launch bring_up bringup.launch.py follow:=true slam:=false imu:=bno085
  ```
  Check it answers at `0x4A` with `i2cdetect -y -r 1`. Outdoors, point the robot
  east and north and set navsat's `yaw_offset` / `magnetic_declination_radians`
  in `config/ekf_navsat_params.yaml` so east reads 0.
- `mpu9250driver` calibrates at startup by treating its current pose as level, so
  it can't show a mounting tilt, and it keeps publishing if the sensor stops
  answering.

### Known issue: tf2 deadlock
On Humble (`tf2_ros` 0.25.23), a lidar scan stamped ahead of TF can deadlock
Nav2's costmap TF listener (`MessageFilter` vs `testTransformableRequests`), and
the controller then freezes with "extrapolation into the future" errors. The
simulated lidar stamps scans 0.1 s back like `rplidar_ros`. If the real robot
freezes this way, check the lidar's scan stamps first.

### Hardware commissioning checklist
1. `sudo usermod -aG dialout roboflock` (serial ports), then install the udev
   rules for `/dev/ublox_gps`, `/dev/rplidar_usb`, `/dev/meshtastic` and the PS4 controller:
   `sudo cp src/bring_up/udev/99-roboflock.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`.
   Use a data USB-C cable for the
   ZED-F9P; with a charge-only cable the board powers up but never appears.
2. Check for a dual-band (L1/L2) antenna for the ZED-F9P.
3. GPS antenna offset measured (0.16 m forward, 0.63 m above ground) and set in the URDF.
   IMU: the MPU-6050 is wired to `/dev/i2c-1` but not mounted, and has no compass.
   Fit a BNO085 (see "IMU: no compass yet"), mount it, then check ENU heading
   and set magnetic declination.
4. Wheels raised: e-stop (Cross, Options, unplugging the joystick), 1.5 m/s²
   braking, and the stale `/cmd_vel` watchdog.
5. Stationary GPS: log `/odometry/global` for 5 min and measure drift.
6. Meshtastic radios configured (see Hardware assignment). Still to do: pair Tom's
   phone with TOM, and test commands and a home position outdoors.
7. Open field: walker at least 10 m ahead, spotter holding the PS4 controller.
