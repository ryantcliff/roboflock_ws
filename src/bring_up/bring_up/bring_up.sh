#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd "${HOME}/roboflock_ws"
source install/setup.bash

exec ros2 launch bring_up robot.launch.py mode:=autonomous
