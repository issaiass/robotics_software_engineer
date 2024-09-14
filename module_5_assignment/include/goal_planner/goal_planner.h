#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pid_lib/pid_lib.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <waypoints_lib/waypoints_lib.hpp>

#include <iostream>
#include <fstream>

class GoalPlanner : public rclcpp::Node {
  public:
    GoalPlanner();
    ~GoalPlanner();

  private:
    void timer_callback(); 
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg);
    tf2::Matrix3x3 get_euler_from_quaternion(const geometry_msgs::msg::Quaternion &quat_msg);
    tf2::Quaternion *get_quaternion_from_euler(double roll, double pitch, double yaw);
    
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher_cmd_vel;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_odom;
    bool _update_pid;
    std::unique_ptr<PID> _angular_pid;
    std::unique_ptr<PID> _linear_pid;
    WayPointLib::Path _path;
    std::ofstream _logFile;
};