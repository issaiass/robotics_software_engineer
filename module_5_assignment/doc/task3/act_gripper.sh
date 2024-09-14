#!/bin/bash

iterations=5

for ((i=0; i<$iterations; i++))
do
  echo "Sending trajectory $((i+1)) of $iterations"

  # Send the trajectory to open and close the gripper
  ros2 topic pub -t 1 /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  \"joint_names\": [\"gripper_tip_ee_right_joint\", \"gripper_tip_ee_center_joint\", \"gripper_tip_ee_left_joint\"],
  \"points\": [
    {
      \"positions\": [0.0, 0.0, 0.0],
      \"velocities\": [0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 0, \"nanosec\": 0}
    },
    {
      \"positions\": [0.7, 0.7, 0.7],
      \"velocities\": [0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 5, \"nanosec\": 0}
    },
    {
      \"positions\": [0.3, 0.3, 0.3],
      \"velocities\": [0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 10, \"nanosec\": 0}
    },
    {
      \"positions\": [-0.1, -0.1, -0.1],
      \"velocities\": [0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 15, \"nanosec\": 0}
    },
    {
      \"positions\": [0.0, 0.0, 0.0],
      \"velocities\": [0.0, 0.0, 0.0],
      \"time_from_start\": {\"sec\": 20, \"nanosec\": 0}
    },
  ]
}"

  sleep 22
done

echo "Finished sending all trajectories."
