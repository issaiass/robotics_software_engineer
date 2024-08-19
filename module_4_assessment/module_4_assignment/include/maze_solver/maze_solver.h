// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"


enum class RobotStates {
    MOVE_STRAIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    OUT_OF_MAZE
};

class MazeSolver : public rclcpp::Node {
  public:
    MazeSolver();
    ~MazeSolver();

  private:
    void timer_callback(); 
    void scan_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg);
    void imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg);
    void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_msg);


    const float THRESHOLD;
    const float ANGULAR_VEL;
    const float LINEAR_VEL;
    float COS_THETA;
    float SIN_THETA;
    RobotStates _state;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _last_time;
    geometry_msgs::msg::Vector3 _cmd_vel;
    sensor_msgs::msg::Imu _imu_msg;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher_cmd_vel;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscriber_scan;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subscriber_imu; 
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscriber_cmd_vel; 
    
    
};