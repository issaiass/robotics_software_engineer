#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    line_following = Node(
        package = 'module_5_assignment',
        name = 'line_following_pid',
        executable ='line_follower_pid',

    )
    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(line_following)

    return ld