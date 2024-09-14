from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    id = LaunchDescription()
    
    # publish static frames
    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        name='controllers_spawner',
        arguments="joint_state_broadcaster arm_controller gripper_controller".split(),
        output='screen',
    )

    id.add_action(controller_manager) 
    return id