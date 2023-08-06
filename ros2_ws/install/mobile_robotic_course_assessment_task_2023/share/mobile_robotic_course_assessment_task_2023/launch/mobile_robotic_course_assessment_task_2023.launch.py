import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    # Define the name of the world file
    world_file_name = 'smaze2d.world'

    # Get the path to the world file
    world_path = os.path.join(get_package_share_directory('mobile_robotic_course_assessment_task_2023'), 'worlds', world_file_name)
  
    # Include the Gazebo launch file with the specified world file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world_path}.items()
             )

    # Define the path to the xacro file
    xacro_file = os.path.join(get_package_share_directory('mobile_robotic_course_assessment_task_2023'),
                              'urdf',
                              '8th_car_sensor_2wheel_1castor_camera_collision.urdf')
    # Define the path to the RViz file
    rviz_file = os.path.join(get_package_share_directory('mobile_robotic_course_assessment_task_2023'),
                              'rviz',   
                              'simple_robot_nav2_original.rviz')                          
    # Parse the xacro file
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Node to publish robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Define the position and orientation of the spawned robot entity given by prof.
    position = [0.6, 0.8, 0.11]                   
    orientation = [0.0, 0.0, -0.7]

    # Define the position and orientation of the spawned robot entity in front yellow block to verify find_yellow_and_put_marker.py (optional)
    #position = [19, 25, 2.09]                   
    #orientation = [0.0, 0.0, -1.7]

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'simple_robot',
                                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
               			   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),],
                        output='screen')

    # Launch RViz
    rviz_entity = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', rviz_file])
                        
    # Command to load the joint state controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Command to load the simple diff drive controller    
    load_simple_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'simple_diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_simple_diff_drive_controller],
            )
        ),
        
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz_entity,
    ])

