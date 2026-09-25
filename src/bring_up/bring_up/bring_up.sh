#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/roboflock/roboflock_ws/   # absolute path
source install/setup.bash

trap 'kill 0' EXIT   # stop all nodes (incl. ros2 run's children) on exit/Ctrl+C

ros2 run joy joy_node &            # background it so script continues
# e_stop defaults are PS4 via hid-generic: Cross = 1 stops, Options = 9 arms
ros2 run bring_up e_stop &
ros2 run bring_up ps4_teleop --ros-args -p cmd_vel_topic:=cmd_vel &
ros2 run bring_up diff_drive_controller

