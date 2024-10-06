# IMU and GPS Sensor Fusion for TurtleBot3

### Assignment 6 - : Custom Extended Kalman Filter

<p align="center">
  <img src="doc/vids/demo.gif?raw=true" alt="Extended Kalman Filter" width="90%" style="display:inline-block;"/>
</p>


We present the creation and results of a custom EKF (no robot_localization package used).

The robot uses IMU and GPS data to fuse with a custom EKF (Extended Kalman Filter) to improve localization.  A turtlebot3 in a default worl is used to simulate the environment.  

We also explore the effects of high or low covariance in the process and measurement matrices and follow our results.

## Outcomes of the EKF Project

<p align="center">
  Low Q (Process Covariance) and Low R (Measurement Covariance)
</p>
<p align="center">
  <img src="doc/vids/lowqlowr.gif?raw=true" alt="Low Q and Low R" width="85%" style="display:inline-block;"/>
</p>

<p align="center">
  Low Q (Process Covariance) and High R (Measurement Covariance)
</>
<p align="center">
  <img src="doc/vids/lowqhighr.gif?raw=true" alt="Low Q and High R" width="85%" style="display:inline-block;"/>
</p>

<p align="center">
  High Q (Process Covariance) and Low R (Measurement Covariance)
</p>
<p align="center">
  <img src="doc/vids/highqlowr.gif?raw=true" alt="High Q and Low R" width="85%" style="display:inline-block;"/>
</p>

<p align="center">
  High Q (Process Covariance) and High R (Measurement Covariance)
</p>
<p align="center">
  <img src="doc/vids/highqhighr.gif?raw=true" alt="High Q and High R" width="85%" style="display:inline-block;"/>
</p>



## Covariance Effects in EKF

The effects of the covariace values of the process noise covariance and the measurement noise covariance \(Q\) and \(R\) respectively on the Extended Kalman Filter (EKF) are listed below. The table shows the extreme values of o ocurrences of \(Q\) and \(R\) with higher and lower value combinations.

| **Covariance Combination** | **Q (Process Noise Covariance)** | **R (Measurement Noise Covariance)** | **Effect on EKF**                                                                                                                                                  |
|----------------------------|-----------------------------------|---------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Low Q - Low R**           | Low                              | Low                                  | - Filter becomes very rigid.<br> - Trusts both the model and sensor measurements equally but might react slowly to sudden changes.<br> - Less noise in the state estimate. |
| **Low Q - High R**          | Low                              | High                                 | - Trusts the model more than the sensor measurements.<br> - Sensor data is considered noisy, so the EKF relies more on predictions.<br> - Can lead to drift if the model is inaccurate. |
| **High Q - Low R**          | High                             | Low                                  | - Trusts sensor measurements more than the model.<br> - Reacts quickly to measurement changes, even small fluctuations.<br> - More noise in the state estimate, leading to instability. |
| **High Q - High R**         | High                             | High                                 | - Filter becomes unstable.<br> - Both the model and measurements are considered unreliable.<br> - Large errors and noise in the state estimate, leading to frequent and unnecessary corrections. |

### A Quicker Explanation
- **low Q**: filter trusts the process model (predictive model) more.
- **high Q**: filter assumes more uncertainty (noise) in the process model, this allows more flexibility for unexpected changes.
- **low R**: filter trusts more on sensor measurement.
- **high R**: filter assumes sensor measurements are noisy, trust more on the prediction.




<details open>
<summary> <b>Brief Review<b></summary>

- We subscribe to topics of imu, gps, and odom (not used here)
- Initialize the EKF Matrices and the state estimation to default values
- Get the information of the imu, gps and odom in the topic messages callbacks
- Finally on the timer callback we make the state estimation of our turtlebot3 and publish marker and the odom filtered message
- The process continues until you stop the simulation.


### <b>Project Tree</b>

```sh
.
├── CMakeLists.txt
├── config
│   └── ekf.yaml
├── include
│   └── extended_kalman_filter
│       ├── ekf_node.hpp
│       └── extended_kalman_filter.hpp
├── launch
│   ├── ekf_turtlebot3.launch.py
│   ├── empty_world.launch.py
│   ├── robot_state_publisher.launch.py
│   ├── spawn_turtlebot3.launch.py
│   ├── turtlebot3_house.launch.py
│   └── turtlebot3_world.launch.py
├── models
│   ├── turtlebot3_common
│   │   ├── meshes
│   │   │   ├── burger_base.dae
│   │   │   ├── lds.dae
│   │   │   ├── r200.dae
│   │   │   ├── tire.dae
│   │   │   ├── waffle_base.dae
│   │   │   └── waffle_pi_base.dae
│   │   └── model.config
│   ├── turtlebot3_house
│   │   ├── model.config
│   │   └── model.sdf
│   ├── turtlebot3_waffle_gps
│   │   ├── model.config
│   │   └── model.sdf
│   ├── turtlebot3_waffle_pi
│   │   ├── model-1_4_.sdf
│   │   ├── model.config
│   │   └── model.sdf
│   └── turtlebot3_world
│       ├── meshes
│       │   ├── hexagon.dae
│       │   └── wall.dae
│       ├── model-1_4.sdf
│       ├── model.config
│       └── model.sdf
├── package.xml
├── README.md
├── rviz
│   └── robot_localization.rviz
├── src
│   └── extended_kalman_filter
│       ├── ekf_node.cpp
│       └── extended_kalman_filter.cpp
├── urdf
│   ├── common_properties.urdf
│   ├── turtlebot3_waffle_gps.urdf
│   └── turtlebot3_waffle_pi.urdf
└── worlds
    ├── empty_world.world
    ├── turtlebot3_house.world
    └── turtlebot3_world.world

17 directories, 40 files
```


</details>



<details open>
<summary> <b>Brief Review<b></summary>

### RQT Graph

<p align="center">
  <img src="doc/imgs/ekf_project.PNG?raw=true" alt="EKF rqt_graph" width="100%" style="display:inline-block;"/>
</p>


</details>


<details close>
<summary> <b>Assignments<b></summary>


#### Task Details

##### Set Up the Sensor Fusion Node:

- Utilize the provided EKF implementation to fuse data from TurtleBot3’s IMU and GPS sensors.
Ensure that the EKF node processes the sensor data and outputs an accurate estimation of the robot’s position and orientation.
Create a Custom Launch File:

- Develop a ROS 2 launch file to start the TurtleBot3 simulation along with the EKF sensor fusion node.
The launch file should be configured to include all necessary parameters and topics for IMU and GPS data inputs.
Visualize the Fused Data:

- Use RViz to visualize the robot’s estimated position and orientation as calculated by the EKF.
Display the raw IMU and GPS data alongside the fused output to demonstrate the improvement in localization accuracy.
Experiment with Different Q and R Values:

- Test the EKF with three different sets of Q (process noise covariance) and R (measurement noise covariance) matrices.
- Document the behavior of the robot’s state estimation under each set of values, focusing on how the changes in Q and R affect the accuracy and stability of the EKF.
- Analyze and Document the Results:

- Provide a detailed analysis of the impact of different Q and R values on the EKF’s performance.
- Include screenshots or recordings from RViz showing the robot’s path and the fused sensor data for each set of parameters.
- Discuss which set of Q and R values provided the best balance between accuracy and stability for TurtleBot3’s localization.

<p align="center"> </p>
</details>

<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

- Install eigen, ros2 controllers and other libraries using your ros distribution if you dont have it installed
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

- Run the demos for Assignment 6, currently working only on custom ekf node
- On terminal 1
```sh
    ros2 launch module_5_assignment main.launch.py model:=waffle_gps use_custom_ekf:=true
```

- On terminal 2
```sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

</details>



<details open>
<summary> <b>Results <b></summary>

#### Extended Kalman Filter Video

[<img src= "https://img.youtube.com/vi/vFNr7DC0EDw/0.jpg" />](https://youtu.be/vFNr7DC0EDw)


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