# IMU and GPS Sensor Fusion for TurtleBot3

### Assignment 8 - : Path Planning with A* and RRT

<p align="center">
  <img src="doc/img/astar.PNG?raw=true" alt="A*" width="40%" style="display:inline-block;"/>
  <img src="doc/img/rrt.PNG?raw=true" alt="RRT" width="40%" style="display:inline-block;"/>

 
</p>

 <p align="center">Figure 1. | A* (left) | RRT (right) |</p>

We compare here different path planning algorithms, specifically A* and RRT (Rapidly-exploring Random Tree).  At first, we create a map and we will measure the performance of the algorithms. 

After the analysis we present a summary of pros and cons of the algorithm given a fixed start to goal points.

## Time Analysis

| Algorithm | Computation Time (ms) | Goal |
|---|---|---|
| RRT |  11-15 |  (7,7) |
| A* |  4-9 | (7,7) | 


## Efficienci Analysis

| Feature         | A*  | RRT  |
|-----------------|---------------------------------------------|-------------------------------------------------|
| **Path Length** | Shorter (more direct path to the goal)        | Longer (more roundabout path)                 |
| **Smoothness** | Less smooth (sharp turns, grid-like path)    | Smoother (fewer sharp turns, more curved path) |
| **Efficiency** | More efficient (fewer nodes explored)      | Less efficient (explores more of the space)     |


## Analysis of Path Planners (A* and RRT)


Analysis of A vs. RRT:

A*

- Path Length	Shorter (more direct path to the goal)	Longer (more roundabout path)

- Smoothness	Less smooth (sharp turns, grid-like path)	Smoother (fewer sharp turns, more curved path)
Efficiency	More efficient (fewer nodes explored)	Less efficient (explores more of the space)

Explanation:

- Path Length: A* aims to find the shortest path, leading to a more direct route in this case. RRT, on the other hand, focuses on exploring the space and may find a longer path that still reaches the goal.

- Smoothness: A* typically produces paths that follow the grid structure, resulting in sharp turns. RRT's exploration strategy often leads to more curved paths, which can be smoother.

- Efficiency: A* is generally more efficient in finding a solution quickly, especially in well-defined spaces. RRT explores more of the space to find a path, which can be less efficient in terms of the number of nodes explored.

Important Notes:

- The efficiency of RRT can be improved with variations like RRT* which we are exploring next.
The smoothness of A* paths can be improved too with other techniques.


<details open>
<summary> <b>Brief Review<b></summary>

- We make an occupancy grid caller to make the algorithms in the occupancy grid callback solve the path.

- Started exploring an A* algorithm for different positions on the grid.

- Later we switched to RRT to make it search and explore the grid leading sometimes to a path.

- Finally we measure the performance of the algorithms noting that A* is more efficient in some cases that RTT, due to its nature of been random, RTT sometimes leads to a optimal solution, mostly are not optimal.

- A* gives always an optimal solution to the problem.


### <b>Project Tree</b>

```sh
├── CMakeLists.txt
├── config
│   └── search_path_planning.rviz
├── include
│   ├── algo
│   │   ├── a_star.h
│   │   └── rrt.hpp
│   ├── node
│   │   └── node_rrt.hpp
│   ├── occupancy_grid
│   │   └── occupancy_grid.hpp
│   └── path_planning
│       └── path_planning.hpp
├── launch
│   └── path_plan_rviz.launch.py
├── package.xml
├── Readme.md
└── src
    ├── algo
    │   ├── a_star.cpp
    │   └── rrt.cpp
    ├── node
    │   └── node_rrt.cpp
    ├── occupancy_grid
    │   └── occupancy_grid.cpp
    ├── occupancy_grid_node.cpp
    ├── path_planning
    │   └── path_planning.cpp
    └── path_planning_node.cpp
```


</details>



<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

```sh
    sudo apt-get update
    sudo apt-get install libeigen3-dev
    sudo apt install ros-<ros-distro>-imu-tools
    sudo apt-get install ros-<ros-distro>-ros2-control
    sudo apt-get install ros-<ros-distro>-joint-trajectory-controller
    sudo apt-get install ros-<ros-distro>-joint-state-broadcaster
    sudo apt-get install ros-<ros-distro>-tf2-ros
    sudo apt-get install ros-<ros-distro>-geometry-msgs
    sudo apt-get install ros-<ros-distro>-sensor-msgs
    sudo apt-get install ros-<ros-distro>-nav-msgs
    sudo apt-get install ros-<ros-distro>-geographic-msgs
    sudo apt-get install geographiclib-tools
    sudo apt-get install libgeographic-dev
    sudo geographiclib-get-data geoid egm96-5
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

- Later compile the module_6_assignmet repository and source it
```sh
    cd ~/assignments_ws
    colcon build --packages-select module_6_assignment
    source install/setup.bash
```

- Run the demos for Assignment 8
- On terminal 1
```sh
    ros2 launch module_8_assignment path_plan_rviz.launch.py
```

START NOTES:
- Running A*
  - For running A* you must comment lines between 29-53 that have been involved with RRT

  - After that uncomment near line 27
```sh
   path_msg.poses = a_star(grid);
```
- Running RTT*
  - For running RTT* you must uncomment lines between 29-53 that have been involved with RRT

  - After that comment near line 27

END NOTES

- On terminal 2
```sh
    ros2 run module_8_assigment path_planning_node
```

</details>



<details open>
<summary> <b>Results <b></summary>

#### A* Path Planning Video

[<img src= "https://img.youtube.com/vi/JL2UkT6qXOs/0.jpg" />](https://youtu.be/JL2UkT6qXOs)


#### RRT Path Planning Video

[<img src= "https://img.youtube.com/vi/DkVbFYuNVf8/0.jpg" />](https://youtu.be/DkVbFYuNVf8)


</details>


<details open>
<summary> <b>Issues<b></summary>

#### Extended Kalman Filter

- If you want to use the robot_localization package will not work of-the-shelf on this demo, you have to feed the odom instead of the gps.
- Only tested for waffle_gps model.

</details>

<details open>
<summary> <b>Future Work<b></summary>

- Test more examples.

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