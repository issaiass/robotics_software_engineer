import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(): 
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    id = LaunchDescription()
    
    turtlebot3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')
    module_5_assignment_path = get_package_share_directory('module_5_assignment')    
    rviz_config_file = os.path.join(module_5_assignment_path, 'rviz', 'lqr.rviz')

    start_tb3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_path, 'launch', 'empty_world.launch.py'))
    )

    start_lqr_node = Node(
        name='lqr_node', 
        package='module_5_assignment', 
        executable='lqr_node'
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=f"-d {rviz_config_file}".split()
    )

    start_lqr_node_tmr = TimerAction(
        period=4.5,
        actions=[start_lqr_node]
    )

    id.add_action(start_rviz)
    id.add_action(start_tb3_world)
    id.add_action(start_lqr_node_tmr)
    return id    