#!/bin/bash
# Change directory to the ROS 2 workspace
cd ~/ros2_ws/

# Build the workspace using colcon
colcon build

# Source the workspace setup script
. install/local_setup.bash

# Launch mobile_robotic_course_assessment_task_2023 node with simulated time enabled
gnome-terminal -- ros2 launch mobile_robotic_course_assessment_task_2023 mobile_robotic_course_assessment_task_2023.launch.py use_sim_time:=True

# Launch nav2_bringup node for navigation with simulated time enabled
gnome-terminal -- ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# Launch slam_toolbox node for SLAM with simulated time enabled
gnome-terminal -- ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam:=True

# Launch explore_lite node for exploration
gnome-terminal -x bash -c 'cd ~/ros2_ws/;
. install/local_setup.bash;
ros2 launch explore_lite explore.launch.py; exec bash'

# Launch twist_mux node for controlling twist commands with specified parameters
gnome-terminal -- bash -c 'cd ~/ros2_ws/;
. install/local_setup.bash;
ros2 run twist_mux twist_mux --ros-args --params-file ./src/mobile_robotic_course_assessment_task_2023/config/twist_mux.yaml -r cmd_vel_out:=simple_diff_drive_controller/cmd_vel_unstamped; exec bash'

# Find yellow object and put marker
gnome-terminal -- bash -c 'cd ~/ros2_ws/;
. install/local_setup.bash;
cd ./src/mobile_robotic_course_assessment_task_2023/scripts;
python3 find_yellow_and_put_marker.py; exec bash'