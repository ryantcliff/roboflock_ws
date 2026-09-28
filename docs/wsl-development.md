# RoboFlock development on WSL Ubuntu 22.04

This workspace can reproduce the Jetson's ROS 2 application-development stack
on an Ubuntu 22.04 WSL distribution. It does not reproduce the Jetson's ARM64
hardware, JetPack, CUDA, GPIO, or native I2C interfaces.

## Bootstrap

Clone the repository into Ubuntu 22.04, select the development branch, and run:

```bash
cd roboflock_ws
bash scripts/bootstrap_ubuntu_22_04.sh
```

The script follows the current ROS-supported apt-source setup, installs ROS 2
Humble Desktop and development tools, resolves workspace packages with
`rosdep`, installs the pinned Python hardware libraries, and builds with
`colcon --symlink-install`. It is idempotent: rerunning it asks apt, rosdep,
pip, and colcon to converge the existing installation on the same result.

Inspect the command plan without changing the machine or workspace:

```bash
bash scripts/bootstrap_ubuntu_22_04.sh --dry-run
```

The bootstrap deliberately does not edit `.bashrc`. Activate ROS and this
workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/roboflock_ws/install/setup.bash
```

Adjust the workspace path if the clone is elsewhere.

## Verify

Run the complete build and test verification:

```bash
bash scripts/verify_dev_environment.sh
```

For a quick dependency and existing-build check:

```bash
bash scripts/verify_dev_environment.sh --skip-build
```

Missing hardware is reported as a warning and does not make the software check
fail. Simulation and smoke tests prove ROS behavior only; they do not prove
physical commissioning.

## Connect USB devices to WSL

Windows does not automatically expose USB serial devices, LiDAR, GPS radios,
or controllers inside WSL. From an elevated PowerShell terminal, install
`usbipd-win`, update WSL, and share each device once:

```powershell
winget install --interactive --exact dorssel.usbipd-win
wsl --update
usbipd list
usbipd bind --busid <BUSID>
```

Keep the WSL distribution running. Then attach the shared device from a normal,
non-administrator PowerShell terminal:

```powershell
usbipd attach --wsl --busid <BUSID>
```

Back in Ubuntu, confirm the device with `lsusb` and check that its interface
appears under `/dev/ttyUSB*`, `/dev/ttyACM*`, or `/dev/input/js*`. Repeat the
attach command after reconnecting a device or restarting Windows.

The repository's Jetson udev rules describe stable robot device names, but
their physical matches must be checked against the USB devices forwarded to
WSL. Jetson GPIO and `/dev/i2c-1` are not equivalent to WSL USB passthrough;
test those interfaces on the Jetson.

## Pinned non-ROS Python packages

The bootstrap installs these user-scoped versions to match the recreated
environment:

- `meshtastic==2.7.11`
- `odrive==0.6.11.post1`
- `adafruit-circuitpython-bno08x==1.3.3`
- `adafruit-extended-bus==1.0.2`
- `cffi==2.1.1`

ROS dependencies remain declared in package manifests and are resolved through
`rosdep`, avoiding a second manually maintained ROS package list.
