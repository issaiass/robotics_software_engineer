from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('module_5_assignment')
    task_name = LaunchConfiguration('task').perform(context)
    
    # Get the varialbe name
    print(f"Task Name: {task_name}")


    # named tasks
    tasks = dict(task1='tb3_line_world', task2='tb3_goal_planner', task3='robotic_arm', task4='tb3_lqr')

    if not task_name in tasks:
       print(f'Bad Task Name {task_name}')
       possible_tasks = ' '.join(list(tasks.keys()))
       print(f'Possible tasks are {possible_tasks}')
       return id    

    task =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'main_'+tasks.get(task_name)+'.launch.py'))
    )


    # actions to load
    processes = [task]

    return processes

def generate_launch_description():
    id = LaunchDescription()
    
    task_arg = DeclareLaunchArgument(
        name='task',
        default_value='task1',
        description='Name of the task to launch. Possibilities are [task1, task2, task3, task4]'
    )
    
    setup = OpaqueFunction(function=launch_setup)

    id.add_action(task_arg)
    id.add_action(setup)
    return id