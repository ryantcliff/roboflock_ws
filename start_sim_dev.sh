#!/usr/bin/env bash

export LIBGL_ALWAYS_SOFTWARE=true
export MESA_GL_VERSION_OVERRIDE=3.3

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS 2 Humble setup not found: $ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
    echo "RoboFlock workspace setup not found: $WS_SETUP" >&2
    echo "Run 'colcon build --symlink-install' in $WORKSPACE first." >&2
    exit 1
fi

# Make sure gnome-terminal exists
if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is not installed."
    echo "Install it with:"
    echo "sudo apt install gnome-terminal"
    exit 1
fi

if ! source "$ROS_SETUP"; then
    echo "Failed to source ROS 2 Humble: $ROS_SETUP" >&2
    exit 1
fi

if ! source "$WS_SETUP"; then
    echo "Failed to source the RoboFlock workspace: $WS_SETUP" >&2
    exit 1
fi

if ! ros2 pkg prefix bring_up >/dev/null 2>&1; then
    echo "The bring_up package is not available in the built workspace." >&2
    echo "Rebuild with 'colcon build --symlink-install'." >&2
    exit 1
fi

printf -v WORKSPACE_Q '%q' "$WORKSPACE"
printf -v ROS_SETUP_Q '%q' "$ROS_SETUP"
printf -v WS_SETUP_Q '%q' "$WS_SETUP"

# Common setup used in every terminal. The quoted values also support paths
# containing spaces.
ROS_ENV="cd $WORKSPACE_Q && source $ROS_SETUP_Q && source $WS_SETUP_Q && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1"

launch_terminal() {
    local title="$1"
    local command="$2"

    if ! gnome-terminal \
        --title="$title" \
        -- bash -lc "
            if ! $ROS_ENV; then
                echo
                echo 'RoboFlock environment setup failed.' >&2
                exec bash
            fi

            $command
            status=\$?
            if (( status != 0 )); then
                echo
                echo \"RoboFlock command exited with status \$status.\" >&2
            fi
            exec bash
        "; then
        echo "Failed to open terminal: $title" >&2
        return 1
    fi
}

echo "Starting RoboFlock development environment..."

# -------------------------------------------------
# Terminal 1: Gazebo simulation
# -------------------------------------------------

launch_terminal "RoboFlock - Simulation" "
    echo '=== RoboFlock Simulation ==='
    ros2 launch bring_up robot.launch.py mode:=simulation
" || exit 1

# Give Gazebo some time to begin starting
sleep 4

# -------------------------------------------------
# Terminal 2: Keyboard teleop
# -------------------------------------------------

launch_terminal "RoboFlock - Teleop" "
    echo '=== Keyboard Teleop ==='
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/cmd_vel/teleop
" || exit 1

# -------------------------------------------------
# Terminal 3: Diagnostics / ROS shell
# -------------------------------------------------

launch_terminal "RoboFlock - Diagnostics" "
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
" || exit 1

echo
echo "RoboFlock terminals launched."
echo
echo "Expected windows:"
echo "  1. Simulation, SLAM, and RViz"
echo "  2. Teleop"
echo "  3. Diagnostics"
echo
echo "Workspace: $WORKSPACE"
