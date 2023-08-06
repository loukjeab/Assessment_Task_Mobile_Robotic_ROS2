# ROS2_WS: ROS2 Workspace for Mobile Robotics Course Assessment Task - 2023

This ROS2 workspace provides a mobile robotics assessment task for the Mobile Robotics SS2023 course.

## Student Information
- Name: Peeranut Noonurak
- Matriculation No.: 7023582

## Package Overview
This ROS2 workspace provides a complete solution for the Mobile Robotics Course Assessment Task in the SS2023 semester. It includes the necessary launch files, configuration files, and code to simulate and control a mobile robot in a Gazebo environment.

## Prerequisites

- ROS 2: Make sure you have ROS 2 installed on your system.
- Bash file: it is necessary to modify the GAZEBO_MODEL_PATH in .bashrc: `export GAZEBO_MODEL_PATH=~/ros2_ws/src/mobile_robotic_course_assessment_task_2023/worlds`
- Gazebo: Install Gazebo simulator if you haven't already.
- ROS 2 Navigation Stack: Install the ROS 2 Navigation Stack for navigation capabilities.
- SLAM Toolbox: Install the SLAM Toolbox for Simultaneous Localization and Mapping (SLAM) functionality.
- Python library: Make sure you have all library used in find_yellow_and_put_marker.py
- Explore-lite package: It requires the explore-lite package contained within the m-explore-ros2 directory. Due to modified parameters in param.yaml, it is essential to use the explore-lite files from this.zip file.

## Usage

To simplify the launch process, you can use the provided step

1. Open a terminal.

2. Navigate to the ROS 2 workspace:

cd ~/ros2_ws/

3. Source the workspace setup script:

. install/local_setup.bash

4. Source ROS 2 Installation: Source the ROS 2 installation setup script to set up the ROS 2 environment:

source /opt/ros/humble/setup.bash

5. Run the Package: Execute the provided shell script to run the package:

bash exam.sh

6. Manually configure RViz:

- In the RViz GUI, click on the "Add" button on the left side of the window.
- Scroll down and select "Marker" under the "rviz_default_plugins" section.
- A new "Marker" panel will appear in the RViz window.
- In the "Marker" panel, locate the "Topic" field and enter `arrow_marker` as the topic name.
- Change the "Fixed Frame" from `odom` to `map` in the "Global Options" tab.

7. Enjoy explore and find the yellow block!

## Troubleshooting: Robot Stopping Exploration

If you encounter the issue of the robot stopping the exploration process, you can follow these steps to resume the exploration:

1. Open a terminal and navigate to the ROS 2 workspace:

cd ~/ros2_ws/

2. Launch the `rqt` program by typing the command:

rqt

3. In the `rqt` GUI, click on the "Plugins" menu tab.

4. Select "Topics" from the dropdown menu.

5. In the "Topics" panel, search for the topic named `/explore/resume`.

6. Click the "+" button next to the topic to add it to the active topics.

7. Double-click on the added topic to open the "Topic Monitor" window.

8. In the "Topic Monitor" window, change the expression from `False` to `True`.

9. Tick the checkbox next to the expression to enable it.

By following these steps, you should be able to resume the exploration process. If the issue persists, you can try rebooting your system and running all the steps again from the beginning.

If you continue to experience difficulties, please seek assistance from the student named above or course instructor.

## exam.sh Explanation

The exam.sh script automates the launch of essential nodes for robot control, navigation, SLAM, and exploration. It performs the following steps:

1. Change to the ROS 2 workspace directory.

2. Build the workspace using colcon.

3. Source the workspace setup script.

4. Launch the `mobile_robotic_course_assessment_task_2023` node for robot control.

5. Launch the `nav2_bringup` node for navigation.

6. Launch the `slam_toolbox` node for SLAM.

7. Launch the `explore_lite` node for exploration.

8. Launch the `twist_mux node` for controlling twist commands.

9. Execute the `find_yellow_and_put_marker.py` script to find a yellow object and place a marker.

By executing the exam.sh script, all the necessary nodes will be launched to control the robot, perform navigation and SLAM, explore the environment, and place markers on yellow objects.

## Additional Notes
- Make sure you follow the Prerequisites and Usage accordingly

- Make sure to configure the RViz settings as mentioned in step 5 to visualize the marker correctly.

- The package provides flexibility for modifying the robot's URDF file, controllers, and other configurations to suit specific requirements. Please refer to the package's source code including comment for guidance on customization.

- If you encounter any issues or have questions regarding the package, feel free to reach out to the student named above for assistance.

- This package was developed as part of the Mobile Robotics course assessment for the SS2023 semester. It serves as a demonstration of the student's understanding and skills in mobile robotics and ROS2.

## License
The code and files in this package are provided under the [MIT License](LICENSE). Feel free to use and modify them for educational and research purposes.
