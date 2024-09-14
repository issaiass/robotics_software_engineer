from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    urdf_name = LaunchConfiguration('urdf_name').perform(context)

    pkg_path = get_package_share_directory('module_5_assignment')
    urdf = os.path.join(pkg_path, 'urdf', urdf_name)
    
    # publish static frames
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf]
    )


    rviz_config = os.path.join(pkg_path, 'rviz', 'config.rviz')
    config = ''.join(f"-d {rviz_config}")

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=config.split()
    )

    processes = [robot_state_publisher, rviz2]

    return processes

def generate_launch_description():
    id = LaunchDescription()

    urdf_name_arg = DeclareLaunchArgument(name='urdf_name', 
                                      default_value='diff_drive.urdf', 
                                      description='Name of the robot'
    )


    # calling the lauch_setup will capture easily the arguments
    setup = OpaqueFunction(function=launch_setup)

    # launch the nodes
    id = LaunchDescription()
    id.add_action(urdf_name_arg)
    id.add_action(setup)
    return id