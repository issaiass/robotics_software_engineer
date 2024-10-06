#ifndef EKF_NODE_HPP
#define EKF_NODE_HPP

#include <extended_kalman_filter/extended_kalman_filter.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 4, 5> Matrix45d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 2, 5> Matrix25d;

class ExtendedKalmanFilter_Node : public rclcpp::Node {
  public:
    ExtendedKalmanFilter_Node();
  private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();    
    void estimation();
    void setMatrices();
    void publish_odom_marker(double x, double y);
    void publish_odom_filtered(double x, double y, double vx, double vy, double angular_z);    
    Vector5d x_in;
    Matrix5d P_in;
    Matrix5d F_in;
    Matrix45d H_in;
    Matrix4d R_in;
    Matrix5d Q_in;

    Vector4d measurements;

    // The following values are the covariances for the GPS and IMU sensors taken from the KITTI data.
    double cov_imu = 0.012727922061358;
    double cov_gps = 0.028452340807104;

    ExtendedKalmanFilter ekf;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;



    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markerArraySub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic estimation
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;    

};

#endif