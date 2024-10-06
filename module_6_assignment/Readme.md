# IMU and GPS Sensor Fusion for TurtleBot3

## Objective
This assignment focuses on implementing and analyzing an Extended Kalman Filter (EKF) for fusing IMU and GPS data to improve the localization accuracy of TurtleBot3 in a simulated environment. You will create a sensor fusion node, visualize the results, and experiment with different noise covariance matrices to understand their impact on the robot’s state estimation.

### Task Details
####  1. Set Up the Sensor Fusion Node
- Utilize the provided EKF implementation to fuse data from TurtleBot3’s IMU and GPS sensors.
Ensure that the EKF node processes the sensor data and outputs an accurate estimation of the robot’s position and orientation.
### 2. Create a Custom Launch File
- Develop a ROS 2 launch file to start the TurtleBot3 simulation along with the EKF sensor fusion node.
The launch file should be configured to include all necessary parameters and topics for IMU and GPS data inputs.
### 3. Visualize the Fused Data
- Use RViz to visualize the robot’s estimated position and orientation as calculated by the EKF.
Display the raw IMU and GPS data alongside the fused output to demonstrate the improvement in localization accuracy.
### 4. Experiment with Different Q and R Values
- Test the EKF with three different sets of Q (process noise covariance) and R (measurement noise covariance) matrices.
Document the behavior of the robot’s state estimation under each set of values, focusing on how changes in Q and R affect the accuracy and stability of the EKF.
### 5. Analyze and Document the Results
- Provide a detailed analysis of the impact of different Q and R values on the EKF’s performance.
Include screenshots or recordings from RViz showing the robot’s path and the fused sensor data for each set of parameters.
Discuss which set of Q and R values provided the best balance between accuracy and stability for TurtleBot3’s localization.
This markdown format will display the content in a cle