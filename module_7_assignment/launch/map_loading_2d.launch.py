#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Launch arguments
    map_name = LaunchConfiguration('map', default='corridor.yaml')
    map_yaml_file = PathJoinSubstitution([
        get_package_share_directory('module_7_assignment'),
        'map', 
        map_name
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments="-d module_7_assignment/config/rviz.rviz".split()
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )


    set_map_server_to_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )

    # Command to activate the map server, wrapped in a TimerAction for delay
    set_map_server_to_activate = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                output='screen'
            )
        ]
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument('map', default_value='corridor.yaml'))    

    # Add commands to the launch description
    ld.add_action(map_server)
    ld.add_action(rviz)
    ld.add_action(set_map_server_to_configure)
    ld.add_action(set_map_server_to_activate)


    return ld