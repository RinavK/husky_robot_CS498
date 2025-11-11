#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if we're in a ROS2 workspace
if [ ! -f "install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please run this script from your ROS2 workspace root."
    exit 1
fi

# Terminal 1: Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 launch husky_gazebo husky_ur3.launch.py world:=clearpath_playpen.sdf use_sim_time:=true; \
    exec bash"

# Wait 10 seconds for Gazebo to start
echo "Waiting 10 seconds for Gazebo to initialize..."
sleep 10

# Terminal 2: Launch MoveIt2 move_group
gnome-terminal --tab --title="MoveGroup" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 launch husky_ur3_moveit_config move_group.launch.py use_sim_time:=true; \
    exec bash"

# # Terminal 3: Launch MoveIt2 RViz
# gnome-terminal --tab --title="RViz" -- bash -c "
#     source /opt/ros/humble/setup.bash && \
#     source install/setup.bash && \
#     ros2 launch husky_ur3_moveit_config moveit_rviz.launch.py use_sim_time:=true; \
#     exec bash"

# Terminal 4: Set sim time and spawn gripper controller
gnome-terminal --tab --title="Controllers" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    sleep 3 && \
    ros2 param set /move_group use_sim_time True && \
    ros2 run controller_manager spawner gripper_controller -c /controller_manager; \
    exec bash"

gnome-terminal --tab --title="ROSBAG" -- bash -c "
    source /opt/ros/humble/setup.bash && \
    rm -rf le_bag2 && \
    sleep 5 && \
    ros2 bag record -a -x "/clock" \
    -o le_bag2; \
    exec bash"

echo "All terminals launched!"
sleep 10
source run_movement.bash