from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description(): 
    shape = DeclareLaunchArgument(
        'shape',
        default_value='square',
        description='Paramter of possible shapes [square, triangle, star]'
    )
    linear = DeclareLaunchArgument(
        'linear',
        default_value='1.0',
        description='Paramter for circle radius'
    )
    a = DeclareLaunchArgument(
        'a',
        default_value='0.4',
        description='Paramter initial size of the spiral'
    )
    b = DeclareLaunchArgument(
        'b',
        default_value='0.3',
        description='Paramter for growth of the spiral'
    )

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_gui'
    )

    turtlesim_drive_robot = Node(
        package='module_2_assignment',
        executable='command_turtle',
        name='turtlesim_drive_robot',
        parameters=[{'shape': LaunchConfiguration('shape'), 
                     'linear': LaunchConfiguration('linear'),
                     'a': LaunchConfiguration('a'),
                     'b': LaunchConfiguration('b'),                     
                    }],     
    )

    return LaunchDescription([
        shape,
        linear,
        a,
        b,
        turtlesim,
        turtlesim_drive_robot,
    ])