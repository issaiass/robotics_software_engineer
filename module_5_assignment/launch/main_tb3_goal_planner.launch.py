import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(): 
    id = LaunchDescription()
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    
    turtlebot3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')
    start_tb3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_path, 'launch', 'empty_world.launch.py'))
    )

    start_goal_planner_node = Node(
        name='goal_planner_node', 
        package='module_5_assignment', 
        executable='goal_planner_node'
    )

    start_goal_planner_node_tmr = TimerAction(
        period=4.5,
        actions=[start_goal_planner_node]
    )

    id.add_action(start_tb3_world)
    id.add_action(start_goal_planner_node_tmr)
    return id    