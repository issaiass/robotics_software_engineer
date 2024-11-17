#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-6.794486')
    y_pose = LaunchConfiguration('y_pose', default='3.618')
    world_name = LaunchConfiguration('world', default='maze_map.world')
    #LaunchConfiguration('slam_node', default='slam_toolbox')

    # Use PathJoinSubstitution to handle LaunchConfiguration within a path
    world = PathJoinSubstitution([
        get_package_share_directory('module_7_assignment'),
        'worlds',
        world_name
    ])

    env_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('module_7_assignment'), 'launch', 'maze_tb3_bringup.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'x_pose': x_pose,
                          'y_pose': y_pose,
                          'world': world}.items()
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            condition=LaunchConfigurationEquals('slam_node', 'slam_toolbox')
    )

    gmapping = Node(
        package='slam_gmapping', 
        executable='slam_gmapping', 
        output='screen', 
        parameters=[{'use_sim_time':use_sim_time}],
        condition=LaunchConfigurationEquals('slam_node', 'gmapping')
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('world', default_value='corridor.world'))
    ld.add_action(DeclareLaunchArgument('slam_node', default_value='slam_toolbox'))


    # Add commands to the launch description
    ld.add_action(env_bringup)
    ld.add_action(slam_toolbox)
    ld.add_action(gmapping)

    return ld