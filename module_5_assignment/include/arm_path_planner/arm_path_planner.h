#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <yaml-cpp/yaml.h>


//#include "matplotlibcpp.h"

using namespace std;
using namespace std::chrono_literals;

class ArmPathPlanner : public rclcpp::Node
{
public:
    ArmPathPlanner();

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_marker;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subscriber_joint_states;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _publisher_joint_trajectory;
    Eigen::Vector4d inverse_kinematics(double x, double y, double z, double theta);
    Eigen::Matrix4d forward_kinematics(double theta_base, double theta_finger, double d_prismatic, double theta_ee);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _path_marker;

    //void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg);
    void timer_callback();
    void publishTrajectory();

    vector<vector<float>> _waypoints;

    void publishTrajectoryMarker();
    rclcpp::TimerBase::SharedPtr _timer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    visualization_msgs::msg::Marker line_strip;
    std::vector<geometry_msgs::msg::Point> _recent_points;

};