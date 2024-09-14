# Assignments for Module #5 : Control Systems with ROS2 Control


### Assignment 5 - Task 1: Improve Camera-Based Line Following

A turtlebot3 starts following a path based on the camera detection of a black line. The robot is governed by a PI controller that is the responsible to return to the correct path based on the detection of the mid point of the line aided by the canny edge detector.

<p align="center">
  <img src="doc/task1/imgs/task1.gif?raw=true" alt="Linear Velocity" width="100%" style="display:inline-block;"/>
</p>

We improved the line following the path followed from the robot adding a PI controller, [results and details of our analysis can be found here](doc/task1/Task1%20Analysis%20-%20Camera%20PID%20Line%20Following.pdf).

<p align="center">
  <img src="doc/task1/imgs/linear_velocity.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task1/imgs/linear_velocity_hist.PNG?raw=true" alt="Another Graph" width="45%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task1/imgs/angular_velocity.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task1/imgs/angular_velocity_hist.PNG?raw=true" alt="Another Graph" width="45%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task1/imgs/robot_pixel.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task1/imgs/robot_pixel_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>

### Assignment 5 - Task 2: Optimize Goal Selection and Path Planning

We expanded the PI controller to be a PID controller for the path planning part.  Selection the lowest path with the lower consumption is a must in this part so we proposed 3 tracks to follow to the robot and selected the best based on eucledian distance.

<p align="center">
  <img src="doc/task2/imgs/task2.gif?raw=true" alt="Linear Velocity" width="100%" style="display:inline-block;"/>
</p>


Finally, the study of the position of the path planner of the implemented PID controller could be found  [here detailing our findings](doc/task2/Task%202%20-%20Low%20Energy%20Path%20-%20Position%20Analysis.pdf).


<p align="center">
  <img src="doc/task2/imgs/X_Goal.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task2/imgs/X_Goal_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task2/imgs/Y_Goal.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task2/imgs/Y_Goal_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task2/imgs/Goal.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task2/imgs/Goal_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>



### Assignment 5 - Task 3: Add a Position Controller to Robotic Arm URDF

We made a custom ROS2 controller based on position controllers to enable the follow joint trajectory command.  We also added controllers to enable the gripper command and operate simultaneously with the trajectory of other joints.

<p align="center">
  <img src="doc/task3/imgs/task3.gif?raw=true" alt="Linear Velocity" width="100%" style="display:inline-block;"/>
</p>

We also made in octave the FK (Forward Kinematics) simulation to prevent compiling each time and use also de IK (Inverse Kinematics) to port both codes to ROS 2.

Our approach was at follows here: 
- prepare a simulation in octave using FK
- graph and get the values of the figure 8's trajectories
- compute and validate the IK
- pass the FK and IK to C++ in ROS2

<p align="center">
  <img src="doc/task3/imgs/fk.PNG?raw=true" alt="Linear Velocity" width="50%" style="display:inline-block;"/>
</p>

### Assignment 5 - Task 4: Implement and Visualize LQR for Multi-Goal Following

We improve the LQR controller and added waypoints and path followed visualization to our simulation.

<p align="center">
  <img src="doc/task4/imgs/task4.gif?raw=true" alt="Linear Velocity" width="100%" style="display:inline-block;"/>
</p>


The LQR controller is performing well, [our results on the findings](doc/task4/Task%204%20-%20LQR%20Analysis%20-%20State%20Analysis.pdf) can tell you that for High Q and low R values the regulator does a decent job for all variations, but if you want to perform weighted you can use Q = R = 0.5 on diag and leaving element Q(3,3) = R(3,3) = 0.1

<p align="center">
  <img src="doc/task4/imgs/goal_pos_x.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task4/imgs/goal_pos_x_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task4/imgs/goal_pos_y.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task4/imgs/goal_pos_y_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task4/imgs/goal_pos_theta.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task4/imgs/goal_pos_theta_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>

<p align="center">
  <img src="doc/task4/imgs/goal_pos.PNG?raw=true" alt="Linear Velocity" width="46.5%" style="display:inline-block;"/>
  <img src="doc/task4/imgs/goal_pos_hist.PNG?raw=true" alt="Another Graph" width="47.5%" style="display:inline-block;"/>
</p>


<details open>
<summary> <b>Brief Review<b></summary>

- In part 1 we added a PI controlller to the robot camera line following project previously done, the structure is encapsulated in a PID Library and reused in the code.


### <b>Project Tree</b>
<p align="center">
  <img src="doc/tree/tree1.PNG?raw=true" alt="Linear Velocity" width="33%" style="display:inline-block;"/>
  <img src="doc/tree/tree2.PNG?raw=true" alt="Another Graph" width="33%" style="display:inline-block;"/>
  <img src="doc/tree/tree3.PNG?raw=true" alt="Another Graph" width="33%" style="display:inline-block;"/>  
</p>


</details>

<details close>
<summary> <b>Assignments<b></summary>


#### Objective: 
 - Assignments focus on effective control, code restructuring and software design flow. Even for ths tasks we are requiring to show code capabilities and error debugging in ROS2 and Gazebo.

#### Tasks:
#####  Task 1: Improve Camera-Based Line Following

- You have to enhace the line-following algorithm previously done improving the speed and smoothness by introducing a PI (Proportional-Integral) controller.  

The improvements of the robot parameters an behaviour must be documented.


#### Task 2: Optimize Goal Selection and Path Planning

- Select the goal of the robot (Turtlebot3) based on the criterion of minimal consumption.
- Energy consumption must be evaluated based on the distance to the goal.
- The shortest path to the goal is the path the Turtlebot3 will follow.
- The ROS 2 node will plan and executes the TurtleBot3’s movement to the selected goal with minimal energy consumption.

#### Task 3: Add a Position Controller to Robotic Arm URDF

- Improve the model of the robotic arm of the previous module:
- Add a position controller to manage the arm’s movements accurately.
- Ensure the URDF file is correctly configured to simulate the position controller. - You need to send multiple positions to joints of the Robotic Arm.

#### Task 4: Implement and Visualize LQR for Multi-Goal Following
#### Objective: 
- Enhance the multi-goal following behavior of TurtleBot3 using LQR and analyze the impact of different Q and R matrix values on the robot’s path and performance.

- Visualize Goals and Path in RViz:
  - a. Implement visualizations in RViz to display the positions of the goals and the robot’s path.
  - b. Add markers for each goal and a trail of points to represent the robot’s odometry during the path following.
- Experiment with Different Q and R Values:
  - c. Test the LQR controller with three different sets of Q and R matrices.
  - d. Record and compare how these values affect the robot’s behavior, particularly in terms of stability, speed, and smoothness when transitioning between goals and document and Analyze the Results:
  - e. Provide a detailed report on the performance of the LQR controller with different Q and R settings.
  - f. Include visualizations from RViz to illustrate the robot’s path and goal positions for each scenario.
Discuss which set of Q and R values provided the best balance between responsiveness and smoothness in the robot’s motion.

<p align="center"> </p>
</details>

<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

- Install eigen, yaml-cpp and ros2 controllers using your ros distribution if you dont have it installed
```sh
    sudo apt-get update
    sudo apt-get install libeigen3-dev
    sudo apt-get install libyaml-cpp-dev
    sudo apt-get install ros-<ros2_distro>-ros2-control 
    sudo apt-get install ros-<ros2_distro>-joint-trajectory-controller 
    sudo apt-get install ros-<ros2_distro>-joint-state-broadcaster
```

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
- Next compile first the module_4_assignmet repository
```sh
    cd ~/assignments_ws
    colcon build --packages-select module_4_assignment
```

- Later compile the module_5_assignmet repository and source it
```sh
    cd ~/assignments_ws
    colcon build --packages-select module_5_assignment
    source install/setup.bash
```

- Run the demos for Assignment 5, for each task, possibilities are task1, task2, task3 and task4
```sh
    ros2 launch module_5_assignment main.launch.py task:=task1
```

- Or Run the demos manually...

- For Assignment 5, Part 1
```sh
    ros2 launch module_5_assignment main_tb3_line_world.launch.py
```

- For Assignment 5, Part 2
```sh
    ros2 launch module_5_assignment main_tb3_goal_planner_world.launch.py
```

- For Assignment 5, Part 3
```sh
    ros2 launch module_5_assignment main_robotic_arm.launch.py
```
- You can find also how to send commands via the terminal using [this bash script file for the gripper](docs/task3/act_gripper.sh) and [this other one for the joints](docs/task3/act_joints.sh).

- For Assignment 5, Part 4
```sh
    ros2 launch module_5_assignment main_tb3_lqr.launch.py
```

</details>



<details open>
<summary> <b>Results <b></summary>

#### Task 1: Improve Camera-Based Line Following

[<img src= "https://img.youtube.com/vi/O12xLjQX3pM/0.jpg" />](https://youtu.be/O12xLjQX3pM)


#### Task 2: Optimize Goal Selection and Path Planning

[<img src= "https://img.youtube.com/vi/GjTvCYY4tQA/0.jpg" />](https://youtu.be/GjTvCYY4tQA)

#### Task 3: Add a Position Controller to Robotic Arm URDF

[<img src= "https://img.youtube.com/vi/lOt_NSsD95k/0.jpg" />](https://youtu.be/lOt_NSsD95k)

#### Task 4: Implement and Visualize LQR for Multi-Goal Following

[<img src= "https://img.youtube.com/vi/vzfLyHSFSoo/0.jpg" />](https://youtu.be/vzfLyHSFSoo)

</details>





<details open>
<summary> <b>Issues<b></summary>

#### Line Following

- PI controller of line follower is very unstable, you can achieve better results if you point the camera looking down to the line.

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