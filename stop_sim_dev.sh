#!/bin/bash

echo "Stopping RoboFlock simulation environment..."

# Ask processes to shut down normally first
pkill -TERM -f "ros2 launch bring_up sim.launch.py" 2>/dev/null || true
pkill -TERM -f "slam_toolbox"                       2>/dev/null || true
pkill -TERM -f "rviz2"                              2>/dev/null || true
pkill -TERM -f "teleop_twist_keyboard"              2>/dev/null || true
pkill -TERM -f "odom_tf_broadcaster"                2>/dev/null || true
pkill -TERM -f "robot_state_publisher"              2>/dev/null || true
pkill -TERM -f "parameter_bridge"                   2>/dev/null || true
pkill -TERM -f "static_transform_publisher"         2>/dev/null || true

# Gazebo / Ignition
pkill -TERM -f "gz sim"                             2>/dev/null || true
pkill -TERM -f "ign gazebo"                         2>/dev/null || true
pkill -TERM -f "gzserver"                           2>/dev/null || true
pkill -TERM -f "gzclient"                           2>/dev/null || true

sleep 2

# Force-kill anything that didn't stop
pkill -KILL -f "slam_toolbox"                       2>/dev/null || true
pkill -KILL -f "rviz2"                              2>/dev/null || true
pkill -KILL -f "teleop_twist_keyboard"              2>/dev/null || true
pkill -KILL -f "odom_tf_broadcaster"                2>/dev/null || true
pkill -KILL -f "robot_state_publisher"              2>/dev/null || true
pkill -KILL -f "parameter_bridge"                   2>/dev/null || true
pkill -KILL -f "static_transform_publisher"         2>/dev/null || true
pkill -KILL -f "gz sim"                             2>/dev/null || true
pkill -KILL -f "ign gazebo"                         2>/dev/null || true
pkill -KILL -f "gzserver"                           2>/dev/null || true
pkill -KILL -f "gzclient"                           2>/dev/null || true

# Clear stale ROS graph information
source /opt/ros/humble/setup.bash

ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo
echo "RoboFlock processes stopped."
echo
echo "Checking for leftovers..."

pgrep -af "gz sim|ign gazebo|gzserver|gzclient|slam_toolbox|rviz2|teleop_twist_keyboard|odom_tf_broadcaster|robot_state_publisher|parameter_bridge|static_transform_publisher" \
    || echo "No RoboFlock simulation processes found."
