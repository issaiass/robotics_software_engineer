#ifndef LQR_NODE_HPP
#define LQR_NODE_HPP

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <lqr_lib/lqr_lib.hpp>
#include <vector>


#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>
#include <fstream>

using namespace std::chrono_literals;
using namespace std;

struct State {

    double x;
    double y;
    double theta;

    State() = default;
    State(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

};

struct input {

    double v;
    double w;

    input() = default;
    input(double v_, double w_) : v(v_), w(w_) {}

};

class LqrNode : public rclcpp::Node {
public:
    LqrNode();
    ~LqrNode();    

private:

    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoopCallback();
    void publishVelocity(double v, double w);
    void optimiseHeading(std::vector<State>& waypoints);
    void angleNormalisation(double& angle);
    void timer_path_callback();
    void publishGoals(const std::vector<State> waypoints);
    void waitForMarkerSubscription(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_input_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr _timer_path;    
    visualization_msgs::msg::Marker line_strip;
    std::vector<geometry_msgs::msg::Point> _recent_points;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _publisher_path_marker;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_goals_marker;    
    nav_msgs::msg::Odometry::SharedPtr _tb3;
    std::ofstream _logFile;


    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;
    Eigen::Vector3d state_error_;

    double dt_;
    double tolerance;
    bool end_controller;
    double max_linear_velocity;
    double max_angular_velocity;
    State actual_state_;
    State desired_state_;
    input control_input_;

    std::vector<State> waypoints_;
    int current_waypoint;
    bool odom_received_;

    std::unique_ptr<LQR> lqr_;
};

#endif