from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description(): 
    shape = DeclareLaunchArgument(
        name='shape',
        default_value='square',
        description='Paramter of possible shapes [square, triangle, star]'
    )

    linear = DeclareLaunchArgument(
        name='linear',
        default_value='1.0',
        description='Paramter for circle radius'
    )

    a = DeclareLaunchArgument(
        name='a',
        default_value='0.4',
        description='Paramter initial size of the spiral'
    )
    
    b = DeclareLaunchArgument(
        name='b',
        default_value='0.3',
        description='Paramter for growth of the spiral'
    )

    cmd_vel_topic = DeclareLaunchArgument(
        name='cmd_vel_topic',
        default_value='/cmd_vel',
        description='Paramter for growth of the spiral'
    )

    pose_topic = DeclareLaunchArgument(
        name='pose_topic',
        default_value='/pose',
        description='Paramter for see the pose topic'
    )


    turtlebot3_enabled = DeclareLaunchArgument(
        name='turtlebot3_enabled',
        default_value='false',
        description='Paramter for differentiate if turtelbot3 is enabled'
    )

    node_name = DeclareLaunchArgument(
        name='node_name',
        default_value='turtle1',
        description='Paramter for differentiate the turtle commander'
    )

    turtlesim_drive_robot = Node(
        package='module_2_assignment',
        executable='command_turtle',
        name=LaunchConfiguration('node_name'),
        parameters=[{
            'shape': LaunchConfiguration('shape'), 
            'linear': LaunchConfiguration('linear'),
            'a': LaunchConfiguration('a'),
            'b': LaunchConfiguration('b'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'node_name': LaunchConfiguration('node_name'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'turtlebot3_enabled': LaunchConfiguration('turtlebot3_enabled')
        }]
    )
    return LaunchDescription([
        shape,
        linear,
        a,
        b,
        cmd_vel_topic,
        pose_topic,
        turtlebot3_enabled,
        node_name,
        turtlesim_drive_robot,
    ])