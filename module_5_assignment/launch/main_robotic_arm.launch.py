# usage:
# ros2 launch module_5_assignment main_robotic_arm.launch.py urdf_name:=robotic_arm.urdf

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve launch configuration parameters
    urdf_name = LaunchConfiguration('urdf_name').perform(context)
    controllers_name = urdf_name.split('.')[0]

    module_5_assignment_path = get_package_share_directory('module_5_assignment')
    
    # Paths to the included launch files
    rviz_launch_path = os.path.join(module_5_assignment_path, 'launch', 'rviz.launch.py')
    gazebo_ros_launch_path = os.path.join(module_5_assignment_path, 'launch', 'gazebo_ros.launch.py')
    ros2_controllers_launch_path = os.path.join(module_5_assignment_path, 'launch', f'{controllers_name}_controllers.launch.py')
    
    # Include launch descriptions
    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path),
        launch_arguments={'urdf_name': urdf_name}.items()
    )

    start_gazebo_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_ros_launch_path)
    )

    start_ros2_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_controllers_launch_path)
    )

    # Delayed start of Gazebo and controllers
    start_gazebo_ros_tmr = TimerAction(
        period=2.5,
        actions=[start_gazebo_ros]
    )

    start_ros2_control_tmr = TimerAction(
        period=3.5,
        actions=[start_ros2_controllers]
    )

    arm_path_planner =  Node(
        package='module_5_assignment',
        executable='arm_path_planner',
        name='arm_path_planner',
        output='screen',
    )


    return [start_rviz, start_gazebo_ros_tmr, start_ros2_control_tmr, arm_path_planner]

def generate_launch_description(): 
    # Declare launch arguments
    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name', 
        default_value='robotic_arm.urdf', 
        description='Name of the URDF file'
    )

    # Use OpaqueFunction to run launch_setup
    setup = OpaqueFunction(function=launch_setup)

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(urdf_name_arg)
    ld.add_action(setup)
    
    return ld