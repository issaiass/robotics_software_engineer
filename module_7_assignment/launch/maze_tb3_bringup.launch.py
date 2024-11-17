#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-6.794486')
    y_pose = LaunchConfiguration('y_pose', default='3.618')
    world_name = LaunchConfiguration('world', default='maze_map.world')

    # Use PathJoinSubstitution to handle LaunchConfiguration within a path
    world = PathJoinSubstitution([
        get_package_share_directory('module_7_assignment'),
        'worlds',
        world_name
    ])

    # Gazebo server command
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client command
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher command
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot command
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # RViz Node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', 'module_7_assignment/config/rviz.rviz']
    )

    # Define LaunchDescription and add actions
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-6.794486'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='3.618'))
    ld.add_action(DeclareLaunchArgument('world', default_value='maze_map.world'))

    # Add commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(rviz)

    return ld
