#!/bin/bash

iterations=1

for ((i=0; i<$iterations; i++))
do
  echo "Sending trajectory"


  ros2 topic pub -t 1 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  \"joint_names\": [\"base_joint\", \"finger_joint\", \"gripper_extension_joint\", \"ee_joint\"],
  \"points\": [
    {
      \"positions\": [0.0, 0.0, 0.0, 0.0],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 0, \"nanosec\": 0}
    },
    {
      \"positions\": [0.2, 0.1, 0.5, 0.1],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 5, \"nanosec\": 0}
    },
    {
      \"positions\": [0.4, 0.0, 1, 0.0],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 10, \"nanosec\": 0}
    },
    {
      \"positions\": [0.9, -0.1, 0.5, -0.1],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 15, \"nanosec\": 0}
    },
    {
      \"positions\": [-3.14, 0.0, 0.3, 0.0],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 20, \"nanosec\": 0}
    },
    {
      \"positions\": [1.57, 0.2, 0.8, 0.2],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 25, \"nanosec\": 0}
    },
    {
      \"positions\": [2.14, 0.4, 0.1, 0.4],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 30, \"nanosec\": 0}
    },
    {
      \"positions\": [-1.2, 0.2, 0.5, 0.2],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 35, \"nanosec\": 0}
    },
    {
      \"positions\": [0.0, 0.0, 0.0, 0.0],
      \"velocities\": [0.0, 0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 40, \"nanosec\": 0}
    }
  ]
}"

  sleep 42
done
echo "Finished sending all trajectories."