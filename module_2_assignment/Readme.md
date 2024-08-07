# Assignments for Module #2 : ROS2 2D Turlesim Simulation

### Assignment 2 - Part 1 - Turtlesim Nodes for 2D Graph
<p align="center">
<img src = "doc/spiral.gif?raw=true" center=true width="55%"/>
</p>

### Assignment 2 - Part 2 - Turtlesim Nodes for 2D Graph
<p align="center">
<img src = "doc/multiple.gif?raw=true" center=true width="55%"/>
</p>



<details open>
<summary> <b>Brief Review<b></summary>

- The assignment 2, part 1, has several ways to interact with a publisher and a subscriber in ROS 2, it will draw 2D shapes like a square, triangle and a star that are fixed. The other parts, circle and logarithmic spiral has some controllable parameters.

- Assigment 2, part 2, has a different approach due to you have to manage the nodes and sync launch files, if you deliver each part as chunk of an application, gradually you will achieve the same result here

Below you will find this project tree

### <b>Project Tree</b>
<p align="center">
<img src = "doc/tree.PNG?raw=true" center=true width="55%"/>
</p>

### <b>rqt_graph's</b>
### Assignment 2, Part 1
<p align="center">
<img src = "doc/rqt_graph_1.PNG?raw=true" center=true width="55%"/>
</p>

### Assignment 2, Part 2
<p align="center">
<img src = "doc/rqt_graph_2.PNG?raw=true" center=true width="55%"/>
</p>


</details>

<details close>
<summary> <b>Assignments<b></summary>


### Assignment 1: Custom Nodes and Launch Files
- **Objective**: 
  - Develop ability to write custom ROS2 nodes and use launch files for running multiple nodes. 
  - Develop a launch file to run the Turtlesim simulation and the custom node simultaneously.
  - A the end of this you will develop understanding and execution of custom nodes and the utility of launch files in ROS2.

- **Tasks**:
   - Create a custom ROS2 node that makes Turtlesim follow a unique pattern:
      - Move in user input radius of circle
      - Move the turtle in logrithmic spiral

### Assignment 2: Exploring Turtlesim Services and Parameters
- **Objective**: Improve ROS2 knowledge of services and parameters in ROS2 with turtlesim and simulated environments.

- **Tasks**:
    - Use existing Turtlesim services (e.g., spawn, clear) to modify the simulation environment.
    - Spawn 5 Turtlebots with 1 single launch file in Diagonal.
    - Drive all turtles for continues movements changing its behaviours

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
    colcon build --packages-select module_2_assigment
    source install/setup.bash
```

- Run the demos for Assignment 2, Part 1
```sh
    ros2 launch module_2_assignment command_turtle shape:=square
    ros2 launch module_2_assignment command_turtle shape:=triangle
    ros2 launch module_2_assignment command_turtle shape:=star
    ros2 launch module_2_assignment command_turtle shape:=circle linear:=0.3
    ros2 launch module_2_assignment command_turtle shape:=spiral a:=0.3 b:=0.45    
```

- Run the full demo of Assignment 2, Part 2
```sh
    # launch turtlesim, clear the current turtle, spawns 5 turtles and drive them
    ros2 launch module_2_assignment drive_all_turtles.launch.py
```

- Run the demos for Assignment 2, Part 2 by bare hands
```sh
    # launches and clear turtlesim window
    ros2 launch module_2_assignment clear_turtlesim.launch.py
    # spawns 5 turtles
    ros2 launch module_2_assignment spawn_turtles.launch.py
    # drive a turtle, NOTE: Not all inputs are needed
    ros2 launch module_2_assignment drive_turtle.launch.py shape:=spiral a:=0.3 b:=0.4 cmd_vel_topic:=/turtle2/cmd_vel pose_topic:=/turtle2/pose node_name:=turtle2_node linear:=0.4 turtlebot3_enabled:=false
```


</details>


<details open>
<summary> <b>Issues<b></summary>

- No issues found
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