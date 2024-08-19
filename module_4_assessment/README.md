# Assignments for Module #4 : Sensor Data for Robot Control

### Assignment 4 - Part 1 - Restructure Line Follower Project Code

<p align="center">
<img src = "doc/line_following.gif?raw=true" center=true width="55%"/>
</p>

<p align="center">
<img src = "doc/line_follower_struct.PNG?raw=true" center=true width="55%"/>
</p>

### Assignment 4 - Part 2 - Design The Software Flow for Maze Solving
<p align="center">

We designed a workflow for maze solving with draw.io based on rqt_graph, simplified version.
<p align="center">
<img src = "doc/maze_solving_flow.PNG?raw=true" center=true width="70%"/>
</p>

### Assignment 4 - Part 3 - Implement Maze Solving in Gazebo

#### Part 3a - Maze Solving

<p align="center">
<img src = "doc/maze_solving.gif?raw=true" center=true width="55%"/>
</p>


#### Part 3b - Fake cmd_vel Calculation

Implemented a fake_vel to show via `sensor_msgs::msg::Imu` the faked `geometry_msgs::msg::Twist`.  Some things to consider is that is a calculated value and have some errors of measurements.

We implemented the rotations of the Z axis of the sensor by 135° CCW (right-hand-rule) due to follow the convention of the `base_link`.

<p align="center">
<img src = "doc/maze_solving_fake_vel.PNG?raw=true" center=true width="85%"/>
</p>

### Assignment 4 - Part 4 - Fix Errors in Launch Files

In part 4 of this assignment we recovered the state of the original file and modified the missing or disordered faults over it.

Errors are related of:
 - bad configuration of the node launches
 - typo of the world
 - bad configuration of the package_shares
 - environment variable for TURTLEBOT3_MODEL and so on.

In resume the sense of changes are documented in [this file](doc/robot_sensing_debug_diff.txt)

Below an image of the file structure.

A (-) signs tells us that this part of the code or launch was removed and a (+) sign tells us that this part of the code was added.

<p align="center">
<img src = "doc/robot_sensing_debug_git_diff.PNG?raw=true" center=true width="55%"/>
</p>

<p align="center">
<img src = "doc/robot_sensing_debug_maze_solver.gif?raw=true" center=true width="55%"/>
</p>


<details open>
<summary> <b>Brief Review<b></summary>

- In part 1 we are requested to bring up the line following robot with the core guidelines of C++
- For part 2 a structure of the code is required before coding to follow up the design.
- Part 3 of this taks involves launch the maze_world app again wit the structures and guidelines of C++, finally we are requested to show via the imu values the linear velocities and accelerations.
- part 4 again we bring up the complete project due to errors from a junior engineer that wants to fulfill the task of launch the application.

### <b>Project Tree</b>
<p align="center">
<img src = "doc/tree.PNG?raw=true" center=true width="25%"/>
</p>

### <b>rqt_graph's</b>

### Assignment 4, Part 1

<p align="center">
<img src = "doc/line_follower_rqt_graph.PNG?raw=true" center=true width="55%"/>
</p>

### Assignment 4, Part 3
<p align="center">
<img src = "doc/maze_solving_rqt_graph.PNG?raw=true" center=true width="55%"/>
</p>

</details>

<details close>
<summary> <b>Assignments<b></summary>


#### Objective: 
 - Assignments focus on effective control, code restructuring and software design flow. Even for ths tasks we are requiring to show code capabilities and error debugging in ROS2 and Gazebo.

#### Tasks:
#####  Task 1: Restructure Line Following Project Code
 - Refactor the line follower project to structure code raeadability based on principles of code in lectures.

#### Task 2: Design Software Flow for Maze Solving
- Create a software design for maze-solving.
- Draw the design using [draw.io](https://draw.io).
- Next, add the design to github.

#### Task 3: Implement Maze Solving in Gazebo
- Create a maze environment in Gazebo with a square shape.
- The robot will move for the right (depending on how you view the map).
- With a LIDAR sense the walls and stablish a rout.
- Show acceleration with IMU sensor.

#### Task 4: Fix Errors in Launch Files
- Remove errors to launch the solutions.
- Document the error debugging process and provide a summary of the errors and their resolutions.

<p align="center"> </p>
</details>

<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

- Create the workspace
```sh
    cd ~
    mkdir -p assignments_ws/src
    cd assignments_ws/src
```
- Fork (or clone) this repo in the `~/assignments_ws/src` folder by typing:
```sh 
    git clone https://github.com/Robotisim/robotics_software_engineer.git
```
- Next compile and source the repository
```sh
    cd ~/assignments_ws
    colcon build --packages-select module_4_assignment
    source install/setup.bash
```

- Run the demos for Assignment 4, Part 1
```sh
    ros2 launch module_4_assignment line_follower.launch.py
```

- Run the demos for Assignment 4, Part 2
```sh
    # no code structure, design flow only
```

- Run the demos for Assignment 4, Part 3
```sh
    ros2 launch module_4_assignment maze_solver.launch.py world:=square.world x_pose:=-3.878695 y_pose:=-5.033942
    ros2 launch module_4_assignment maze_solver.launch.py world:=maze.world # extra, not needed
```

- Run the demos for Assignment 4, Part 4
```sh
    ros2 launch robot_sensing_debug lidar_maze_solving.launch.py
```


</details>


<details open>
<summary> <b>Issues<b></summary>

#### Line Following

- General
  - You will need to install opencv in ros, try `ros-<ros_distro>-<package_name>`, i.e. if you want to install humble gazebo_ros `ros-humble-gazebo-ros`

- You must modify the `<uri>` tag to your path in this files:
  - `worlds/line_follower.world`
  - `meshes/line_track/model.sdf`
- On the line following, depending of the cpu compute speed and ram you will experiment sometimes the line_follower didn't finish.
- Depending on the settings of the thresholding and ROI you will experiment sometimes does not follow the curved line, but the image threshold.

#### Maze Solver

- Sometimes you could experienc issues with Gazebo, just kill all gazebo processes and start again
- Algorithm doesn't completes all the maze tasks due to limits and stochastic process of the simulator

</details>

<details open>
<summary> <b>Future Work<b></summary>

- No Future works.

</details>

<details open>
<summary> <b>Contributing<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>