import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(): 
    id = LaunchDescription()
    
    module_5_assignment_path = get_package_share_directory('module_5_assignment')
    start_tb3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(module_5_assignment_path, 'launch', 'tb3_line_world.launch.py'))
    )

    start_line_follower_pid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(module_5_assignment_path, 'launch', 'line_follower_pid.launch.py'))
    )

    start_line_follower_pid_tmr = TimerAction(
        period=4.5,
        actions=[start_line_follower_pid]
    )

    id.add_action(start_tb3_world)
    id.add_action(start_line_follower_pid_tmr)
    return id    