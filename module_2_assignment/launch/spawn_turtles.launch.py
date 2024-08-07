from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
import numpy as np
STEP = 1.5


def generate_launch_description(): 
    scalar = np.arange(start=3, stop=9+STEP, step=STEP)
    scalar = scalar.reshape(-1, 1)
    ones_arr = np.ones((5,2))
    xypos = scalar*ones_arr
    turtles = len(xypos)
    ld = LaunchDescription()
    
    
    for nturtle in range(turtles):
       x, y = str(xypos[nturtle-2, 0]), str(xypos[nturtle-2, 1])
       print(f'spawner{nturtle+1}')
       spawner = ExecuteProcess(
          cmd= f'ros2 service call /spawn turtlesim/srv/Spawn "{{x: {x}, y: {y}, theta: 1.5707, name: turtle{nturtle+2}}}"'.split(),
          name=f'spawn_turtle{nturtle+1}',
          shell=True
        )
       if nturtle:
          event = RegisterEventHandler(
            OnExecutionComplete(
                target_action=previous_event,
                on_completion=[LogInfo(msg=f'Turtle cleared, spawing turtle{nturtle+1}'), spawner]
            )
          )
          ld.add_action(event)          
       previous_event = spawner
       ld.add_action(spawner)

    return ld