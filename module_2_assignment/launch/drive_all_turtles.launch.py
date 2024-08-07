import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(): 
    id = LaunchDescription()
    
    module_2_assignment = get_package_share_directory('module_2_assignment')

    start_turtlesim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(module_2_assignment, 'launch', 'clean_turtlesim.launch.py'))
    )

    start_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(module_2_assignment, 'launch', 'spawn_turtles.launch.py'))
    )

    commands = dict(shape=['square', 'triangle', 'star', 'circle', 'spiral'],
                    linear=5*[0.2], a=5*[0.4], b=5*[0.3] 
                    )
    id.add_action(start_turtlesim)
    id.add_action(start_spawner)    

    for n in range(2, 5+2):
        ix = n-2
        drive_turtle = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(module_2_assignment, 'launch', 'drive_turtle.launch.py')
            ), launch_arguments={'shape': commands['shape'][ix], 
                                 'linear': str(commands['linear'][ix]),
                                 'a': str(commands['a'][ix]),
                                 'b': str(commands['b'][ix]),
                                 'cmd_vel_topic': f'/turtle{n}/cmd_vel',
                                 'pose_topic': f'/turtle{n}/pose',
                                 'node_name': f'turtle{n}_drive_node',
                                 'turtlebot3_enabled': 'false',
                                 }.items(),
        )
        id.add_action(drive_turtle)


    return id