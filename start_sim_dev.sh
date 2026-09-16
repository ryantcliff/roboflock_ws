#!/bin/bash

export LIBGL_ALWAYS_SOFTWARE=true
export MESA_GL_VERSION_OVERRIDE=3.3

WORKSPACE="$HOME/roboflock_ws"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE/install/setup.bash"

# Common setup used in every terminal
ROS_ENV="
cd $WORKSPACE
source $ROS_SETUP
source $WS_SETUP
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
"

# Make sure gnome-terminal exists
if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is not installed."
    echo "Install it with:"
    echo "sudo apt install gnome-terminal"
    exit 1
fi

echo "Starting RoboFlock development environment..."

# -------------------------------------------------
# Terminal 1: Gazebo simulation
# -------------------------------------------------

gnome-terminal \
    --title="RoboFlock - Simulation" \
    -- bash -lc "
        $ROS_ENV
        echo '=== RoboFlock Simulation ==='
        ros2 launch bring_up robot.launch.py mode:=simulation
        exec bash
    " &

# Give Gazebo some time to begin starting
sleep 4

# -------------------------------------------------
# Terminal 2: Keyboard teleop
# -------------------------------------------------

gnome-terminal \
    --title="RoboFlock - Teleop" \
    -- bash -lc "
        $ROS_ENV
        echo '=== Keyboard Teleop ==='
        ros2 run teleop_twist_keyboard teleop_twist_keyboard \
            --ros-args -r /cmd_vel:=/cmd_vel/teleop
        exec bash
    " &

# -------------------------------------------------
# Terminal 3: Diagnostics / ROS shell
# -------------------------------------------------

gnome-terminal \
    --title="RoboFlock - Diagnostics" \
    -- bash -lc "
        $ROS_ENV

        echo
        echo '=== RoboFlock Diagnostics ==='
        echo
        echo 'Useful commands:'
        echo
        echo '  ros2 node list'
        echo '  ros2 topic list'
        echo '  ros2 topic hz /scan'
        echo '  ros2 topic echo /odom --once'
        echo '  ros2 topic echo /map --once'
        echo
        echo '  ros2 run tf2_ros tf2_echo odom base_link'
        echo '  ros2 run tf2_ros tf2_echo base_link lidar_link'
        echo '  ros2 run tf2_ros tf2_echo map base_link'
        echo
        echo 'Waiting a few seconds before showing nodes...'
        sleep 8
        ros2 node list
        echo

        exec bash
    " &

echo
echo "RoboFlock terminals launched."
echo
echo "Expected windows:"
echo "  1. Simulation, SLAM, and RViz"
echo "  2. Teleop"
echo "  3. Diagnostics"
