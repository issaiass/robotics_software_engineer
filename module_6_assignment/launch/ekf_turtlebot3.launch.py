#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from distutils.util import strtobool

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    use_custom_ekf = LaunchConfiguration('use_custom_ekf').perform(context)
    use_custom_ekf = bool(strtobool(use_custom_ekf))

    os.environ['TURTLEBOT3_MODEL'] = model
    pkg_share_dir = get_package_share_directory("module_6_assignment")
    parameters_file_path = os.path.join(pkg_share_dir,"config", "ekf.yaml")
    rviz2_file_path = os.path.join(pkg_share_dir,"rviz", "robot_localization.rviz")


    rqt_graph = Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph_node',
            output='screen',
    )

    robot_localization_pkg = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[parameters_file_path],
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_tb3',
            output='screen',
            arguments=f"-d {rviz2_file_path}".split()
    )


    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'empty_world.launch.py')
        )
    )

    ekf_node = Node(
            package='module_6_assignment',
            executable='ekf_node',
            name='ekf_custom_node',
            output='screen',
            parameters=[parameters_file_path],
    )

    robot_localization = ekf_node if use_custom_ekf else robot_localization_pkg
    actions = [rqt_graph, rviz2, turtlebot3_gazebo, robot_localization]
    return actions


def generate_launch_description():
    id = LaunchDescription()
    
    model = DeclareLaunchArgument(
        name='model',
        default_value='waffle_pi',
        description='Parameter for turtlebot3 [waffle_pi, waffle_gps]'
    )
    use_ekf = DeclareLaunchArgument(
        name='use_custom_ekf',
        default_value='false',
        description='robot_localization package (false) / ekf_node (true) [false, true]'
    )
    
    setup = OpaqueFunction(function=launch_setup)

    actions = [model, use_ekf, setup]
    [id.add_action(action) for action in actions]
    return id