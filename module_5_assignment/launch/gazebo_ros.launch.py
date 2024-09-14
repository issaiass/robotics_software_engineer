from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    id = LaunchDescription()

    gazebo = ExecuteProcess(
        cmd="gazebo --verbose -s libgazebo_ros_factory.so".split(),
        output="screen"
    )
   
    gazebo_ros_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='robot_spawner',
        arguments="-topic /robot_description -entity robot".split()
    ) 

    id.add_action(gazebo)
    id.add_action(gazebo_ros_spawner)

    return id