# IMU and GPS Sensor Fusion for TurtleBot3

### Assignment 8 - : Path Planning with A* and RRT

<p align="center">
  <img src="doc/img/astar.PNG?raw=true" alt="A*" width="40%" style="display:inline-block;"/>
  <img src="doc/img/rrt.PNG?raw=true" alt="RRT" width="40%" style="display:inline-block;"/>

 
</p>

 <p align="center">Figure 1. | A* (left) | RRT (right) |</p>

### Task 1

We compare here different path planning algorithms, specifically A* and RRT (Rapidly-exploring Random Tree).  At first, we create a map and we will measure the performance of the algorithms. 

After the analysis we present a summary of pros and cons of the algorithm given a fixed start to goal points.

## Time Analysis

| Algorithm | Computation Time (ms) | Goal | Notes |
|---|---|---|---|
| RRT |  11-15 |  (7,7) | RRT quickly explores the space but does not refine existing connections. It is good for finding any solution quickly in high-dimensional or complex spaces.
| A* |  4-9 | (7,7) | A* is a graph-based search. For grid-based maps, it is often the quickest way to get an optimal path, but it scales poorly in very high-dimensional problems.

## Efficienci Analysis

| Feature         | A*  | RRT  |
|-----------------|---------------------------------------------|-------------------------------------------------|
| **Path Length** | Shorter (more direct path to the goal)        | Longer (more roundabout path)                 |
| **Smoothness** | Less smooth (sharp turns, grid-like path)    | Smoother (fewer sharp turns, more curved path) |
| **Efficiency** | More efficient (fewer nodes explored)      | Less efficient (explores more of the space)     |


## Analysis of Path Planners (A* and RRT)

Analysis of A vs. RRT:

A*

- Path Length: A* aims to find the shortest path, leading to a more direct route in this case. RRT, on the other hand, focuses on exploring the space and may find a longer path that still reaches the goal.

- Smoothness: A* typically produces paths that follow the grid structure, resulting in sharp turns. RRT's exploration strategy often leads to more curved paths, which can be smoother.

- Efficiency: A* is generally more efficient in finding a solution quickly, especially in well-defined spaces. RRT explores more of the space to find a path, which can be less efficient in terms of the number of nodes explored.

- The efficiency of RRT can be improved with variations like RRT* which we are exploring next.

- The smoothness of A* paths can be improved too with other techniques.

### Task 2

| Algorithm | Computation Time (ms) | Goal   | Notes |
|---|---|---|---|
| RRT | 11–15  | (7,7)  | Quickly explores the space but does not refine existing connections. Good for finding any solution quickly in high-dimensional or complex spaces.         |
| RRT* | 9–12   | (7,7)  | Builds on RRT with a **rewiring** step, iteratively improving the path quality. May take slightly longer than RRT but often produces shorter, better paths. |

<li>RRT* produces shorter paths than standard RRT thanks to its rewiring step.</li>
<li>Computation Time: May be higher than RRT, but often still competitive with A* for many maze configurations.</li>
<li>Overall Performance: Provides a better balance of exploration and exploitation, bringing performance closer to that of A*.</li>


### Implementation Documentation

| **Aspect**| **Original RRT Implementation**| **Hybrid RRT-A\* (RRT\*) Implementation**|
|---|---|---|
| **Algorithm Structure** | Uses random sampling to quickly explore the configuration space and connect nodes without further refinement. | **Two-Step Process:**<br>1. **RRT Phase:** Quickly finds an initial feasible path.<br>2. **A\* Phase:** Refines the initial path to improve optimality. | **Path Refinement** | No built-in mechanism to optimize the path once a feasible solution is found.| Incorporates an A\* refinement step after RRT to smooth and shorten the initial path. | | **Domain Setup** | Directly works with the occupancy grid to generate nodes.| Converts occupancy grid data into a domain for the RRT planner, then leverages both RRT (for initial connectivity) and A\* (for refinement). |
| **Output Path Quality**   | Returns the first feasible path generated, which may be suboptimal and contain unnecessary detours.  | Returns a refined path that typically has fewer nodes and is shorter, approaching the optimality provided by grid-based A\*. |
| **Logging and Debugging** | Uses basic ROS 2 logging for status messages. | Uses detailed ROS 2 logging (via `RCLCPP_INFO` and `RCLCPP_WARN`) to provide step-by-step information about the planning process, including domain setup, node counts, and final path details. |

### Explanation of Improvements and Competitiveness with A\*

| **Improvement Aspect** | **Explanation** |
|---|---| 
| **Path Optimality** | The addition of an A\* refinement phase significantly improves overall path quality. While RRT quickly finds a path, it may be non-optimal; the A\* step refines the path by reducing unnecessary detours, producing a route that better approximates the ideal solution. |
| **Computation Time Trade-off**  | Although the A\* refinement introduces additional computation, the hybrid approach balances the rapid exploration of RRT with the path optimality of A\*. This trade-off is beneficial in scenarios requiring both fast responses and high-quality paths.                   |
| **Overall Performance**         | By combining the strengths of RRT and A\*, the hybrid method yields a robust solution for continuous spaces. It achieves near-optimal paths, making it competitive with pure A\*, especially in complex or high-dimensional environments.                                   |
| **Competitiveness with A\***     | The dual approach leverages RRT's quick exploration and A\*'s optimization to handle the challenges of continuous space exploration, where pure A\* may struggle. This results in improved path quality and makes the hybrid method a competitive alternative to A\*.              |

### Task 3

#### Test Robutstness
- NarrowCorridor Test: Simulates with obstacles everywhere except a one-cell-wide corridor.

- DeadEnd Test:  Verifies robust failure detection when a valid route is completely obstructed.

- OpenSpace Test: Confirms that the planner successfully finds a valid path in an ideal, open environment.

- ValidPath Test: Ensures the planner meets basic expectations for path generation in a standard scenario.

- NoPathFound Test: Validates the robustness of the algorithm in handling unsolvable conditions.

<br>

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
├── doc
│   └── img
│       ├── astar.PNG
│       └── rrt.PNG
├── include
│   ├── algo
│   │   ├── a_star.h
│   │   ├── rrt_a_star.hpp
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
├── src
│   ├── algo
│   │   ├── a_star.cpp
│   │   ├── rrt_a_star.cpp
│   │   └── rrt.cpp
│   ├── node
│   │   └── node_rrt.cpp
│   ├── occupancy_grid
│   │   └── occupancy_grid.cpp
│   ├── occupancy_grid_node.cpp
│   ├── path_planning
│   │   └── path_planning.cpp
│   └── path_planning_node.cpp
└── test
    └── test_rrt_a_star.cpp
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
  - For running RTT* you replace the a_star for rrt
```sh
   path_msg.poses = rrt(grid);
```

- Running RTT*
  - For running RTT* you replace the a_star for rrt
```sh
   path_msg.poses = rrt_a_star(grid);
```

- On terminal 2
```sh
    ros2 run module_8_assigment path_planning_node
```

- Test with gtest
```sh
    sudo apt-get install libgtest-dev
    colcon build --packages-select module_8_assignment
    ctest --test-dir ./build/module_8_assignment/ --output-on-failure 
```

- You should see:
```sh
Internal ctest changing into directory: /home/issaiass/assignments_ws/src/robotics_software_engineer/build/module_8_assignment
Test project /home/issaiass/assignments_ws/src/robotics_software_engineer/build/module_8_assignment
    Start 1: test_rrt_a_star
1/1 Test #1: test_rrt_a_star ..................   Passed    0.77 sec

100% tests passed, 0 tests failed out of 1

Label Time Summary:
gtest    =   0.77 sec*proc (1 test)

Total Test time (real) =   0.78 sec
```
NOTE: If test fails you should see some part of the code that fails:
```sh
/home/issaiass/assignments_ws/src/robotics_software_engineer/module_8_assignment/test/test_rrt_a_star.cpp:98: Failure
Value of: path.empty()
  Actual: true
Expected: false
[  FAILED  ] RRTAStarTest.NoPathFound (332 ms)
[----------] 2 tests from RRTAStarTest (334 ms total)

[----------] Global test environment tear-down
[==========] 5 tests from 2 test suites ran. (667 ms total)
[  PASSED  ] 4 tests.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] RRTAStarTest.NoPathFound
```

- Other ways to make tests
``` sh
    colcon build --packages-select module_8_assignment
    colcon test --packages-select module_8_assignment
    colcon test-result --verbose
```

</details>



<details open>
<summary> <b>Results <b></summary>

#### A* Path Planning Video

[<img src= "https://img.youtube.com/vi/JL2UkT6qXOs/0.jpg" />](https://youtu.be/JL2UkT6qXOs)


#### RRT Path Planning Video

[<img src= "https://img.youtube.com/vi/Z1K7yUc4w-U/0.jpg" />](https://youtu.be/Z1K7yUc4w-U)


</details>


<details open>
<summary> <b>Issues<b></summary>

- Still no issues found

</details>

<details open>
<summary> <b>Future Work<b></summary>

- No

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