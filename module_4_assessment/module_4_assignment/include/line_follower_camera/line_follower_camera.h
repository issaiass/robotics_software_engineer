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
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class LineFollowerCamera : public rclcpp::Node {
  public:
    LineFollowerCamera();
    ~LineFollowerCamera();

  private:
    void timer_callback(); 
    void image_callback(const std::shared_ptr<sensor_msgs::msg::Image> camera_msg);

    int _low_threshold;
    int _high_threshold;
    int _ctr1, _ctr2, _ctr3, _ctr4;
    float _angular_velocity;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher_cmd_vel;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscriber_camera_image_raw;    
};