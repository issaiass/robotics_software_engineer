from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('module_3_assignment')
    task_name = LaunchConfiguration('task').perform(context)
    
    # Get the varialbe name
    print(f"Task Name: {task_name}")


    # named tasks
    tasks = ['task1', 'task2', 'task3a', 'task3b']
    urdf_suffix = ['robotic_arm', 'robotic_arm', 'mobile_arm', 'ackerman']
    robot_dict = dict(zip(tasks, urdf_suffix))


    if not task_name in tasks:
       print(f'Bad Task Name {task_name}')
       possible_tasks = ' '.join(tasks)
       print(f'Possible tasks are {possible_tasks}')
       return id    

    # urdf
    robot = robot_dict[task_name]
    urdf = os.path.join(pkg_path, 'urdf', f'{task_name}_{robot}.urdf')

    # publish static frames
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf]
    )

    # publish other frames like continuous
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        arguments=[urdf]
    )

    # rviz configuration
    rviz_config = os.path.join(pkg_path, 'rviz', f'rviz_{task_name}.rviz')
    config = ''.join(f"-d {rviz_config}")

    #rviz node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=config.split()
    )

    # actions to load
    processes = [robot_state_publisher, joint_state_publisher, rviz2]

    return processes

def generate_launch_description():
    id = LaunchDescription()
    
    task_arg = DeclareLaunchArgument(
        name='task',
        default_value='task1',
        description='Name of the task to launch. Possibilities are [task1, task2, task3a, task3b]'
    )
    
    setup = OpaqueFunction(function=launch_setup)

    id.add_action(task_arg)
    id.add_action(setup)
    return id