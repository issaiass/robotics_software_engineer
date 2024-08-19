from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os



def launch_setup(context, *args, **kwargs):
    # packge share directories
    module_4_assignment = get_package_share_directory('module_4_assignment')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # world location
    world_name = LaunchConfiguration('world', default='maze.world').perform(context)

    # just to see the sdf or the world file
    if not world_name.endswith('sdf'):
        sdf_name = world_name.split('.')[0]
        world = os.path.join(module_4_assignment, 'models', sdf_name, 'model.sdf')
    if world_name.endswith('world'):
        world = os.path.join(module_4_assignment, 'worlds', world_name)

    # debug info
    print()
    print(f'WORLD NAME : {world_name}')
    print(f'WORLD : {world}')
    print()
   
    # set pose
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.889982')
    y_pose = LaunchConfiguration('y_pose', default='-0.532058')    
    z_pose = LaunchConfiguration('z_pose', default=TextSubstitution(text=str(0.0))) 
   

    # launch turtlebot 3 wafflepi by environment variable in a custom position in a empty world
    turtlebot3_launch_file = os.path.join(turtlebot3_gazebo, 'launch', 'empty_world.launch.py')
    turtlebot3_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_launch_file),
        launch_arguments={
            'use_sim_time':use_sim_time,
            'x_pose':x_pose,
            'y_pose':y_pose,
            'z_pose': z_pose
        }.items()
    )

    # spawn the entity world as the main worl for the robot
    spawn_world = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_world',
        output='screen',
        arguments=f"-file {world} -entity world -x 0.0 -y 0.0 -z 0.0".split()
    )

    # launc the maze solver node
    maze_solver = Node(
        package='module_4_assignment',
        name='maze_solver',
        executable='maze_solver'
    )

    # actions to load
    processes = [turtlebot3_empty_world, spawn_world, maze_solver]

    return processes

def generate_launch_description(): 
    # environment variable for waffle pi
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'


    # launch arguments        
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='false', description='Use simulation time')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value=TextSubstitution(text=str(-2.889982)), description='pose of object in x')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value=TextSubstitution(text=str(-0.532058)), description='pose of object in y')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value=TextSubstitution(text=str(0.0)), description='pose of object in z')

    # calling the lauch_setup will capture easily the arguments
    setup = OpaqueFunction(function=launch_setup)

    # launch the nodes
    id = LaunchDescription()
    id.add_action(use_sim_time_arg)
    id.add_action(x_pose_arg)
    id.add_action(y_pose_arg)
    id.add_action(z_pose_arg)
    id.add_action(setup)
    return id
