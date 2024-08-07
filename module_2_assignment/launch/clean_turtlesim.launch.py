from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import ExecuteProcess


def generate_launch_description(): 

    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_gui',
    )

    clear_turtle1 = ExecuteProcess(
        cmd='ros2 service call /kill turtlesim/srv/Kill \"{name: \'turtle1\'}\"'.split(),
        name="clear_turtle1",
        shell=True
    )
    
    clear_event = RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, clearing turtle1'),
                    clear_turtle1,
                ],
            )
    )

    ld.add_action(turtlesim_node)
    ld.add_action(clear_event)

    return ld