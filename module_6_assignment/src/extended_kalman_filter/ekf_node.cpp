#include "extended_kalman_filter/ekf_node.hpp"


// IMU Readings: [omega, acceleration (z)]
// GPS Readings: [x_gps, y_gps]
// State Vector: [x, y, vx, vy, theta]
// Measurements: [GPS_readings, IMU_readings]
// The object being measured is a car

ExtendedKalmanFilter_Node::ExtendedKalmanFilter_Node() : Node("ekf_node")
{
    setMatrices();

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
    
    imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&ExtendedKalmanFilter_Node::imuCallback, this, std::placeholders::_1));

    odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&ExtendedKalmanFilter_Node::odomCallback, this, std::placeholders::_1));

    // Create a timer to periodically call estimation()
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(ekf.dt * 1000)),
        std::bind(&ExtendedKalmanFilter_Node::timerCallback, this));
}


void ExtendedKalmanFilter_Node::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_msg = msg;
}

void ExtendedKalmanFilter_Node::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_msg = msg;
}


void ExtendedKalmanFilter_Node::timerCallback()
{
    if (!imu_msg || !odom_msg)
    {
        RCLCPP_WARN(this->get_logger(), "IMU message not received yet.");
        return;
    }

    estimation();

    double filtered_x   = ekf.x_pred_[0];
    double filtered_y   = ekf.x_pred_[1];
    double filtered_vx  = ekf.x_pred_[2];
    double filtered_vy  = ekf.x_pred_[3];
    double filtered_yaw = ekf.x_pred_[4];

    publish_odom_marker(filtered_x, filtered_y);
    publish_odom_filtered(filtered_x, filtered_y, filtered_vx, filtered_vy, filtered_yaw);
}

void ExtendedKalmanFilter_Node::publish_odom_marker(double x, double y) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "tb3odom";
    marker.id = 0.0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_publisher_->publish(marker);
}


void ExtendedKalmanFilter_Node::publish_odom_filtered(double x, double y, double vx, double vy, double angular_z)  {
  // Create and publish the odometry message
    nav_msgs::msg::Odometry odom_msg_filtered;
    odom_msg_filtered.header.stamp = this->get_clock()->now();
    odom_msg_filtered.header.frame_id = "odom";  // Set your appropriate frame
    odom_msg_filtered.child_frame_id = "base_link";  // Set your appropriate child frame

    // Set the position
    odom_msg_filtered.pose.pose.position.x = x;
    odom_msg_filtered.pose.pose.position.y = y;
    odom_msg_filtered.pose.pose.position.z = 0.0;

    // Convert yaw angle to quaternion for odometry
    tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // // Convert yaw to quaternion for odometry
    // tf2::Quaternion q;
    // double yaw;
    // q.setRPY(0, 0, angular_);  // Set the yaw in RPY format
    // q.normalize();

    odom_msg_filtered.pose.pose.orientation.x = q.x();
    odom_msg_filtered.pose.pose.orientation.y = q.y();
    odom_msg_filtered.pose.pose.orientation.z = q.z();
    odom_msg_filtered.pose.pose.orientation.w = q.w();

    // Set the velocities
    odom_msg_filtered.twist.twist.linear.x = vx;
    odom_msg_filtered.twist.twist.linear.y = vy;
    odom_msg_filtered.twist.twist.linear.z = 0.0;
    odom_msg_filtered.twist.twist.angular.z = angular_z;

    // Publish the odometry message
    odom_publisher_->publish(odom_msg_filtered);    
}

void ExtendedKalmanFilter_Node::estimation()
{
    RCLCPP_INFO(this->get_logger(), "--------> Starting EKF Iterations ");

    ekf.predict();
    RCLCPP_INFO(this->get_logger(), "Prediction step completed. State and covariance matrices have been predicted.");

    measurements << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, imu_msg->linear_acceleration.x, imu_msg->angular_velocity.z;
    RCLCPP_INFO(this->get_logger(), "Measurements obtained from sensors: x = %f, y = %f, ax = %f, az = %f",
                measurements[0], measurements[1], measurements[2], measurements[3]);

    std::vector<double> cov_values = {cov_gps, cov_gps, cov_imu, cov_imu};
    ekf.updateR(cov_values);
    ekf.update(measurements);
    RCLCPP_INFO(this->get_logger(), "Update step completed. State and covariance matrices have been corrected.");
}

void ExtendedKalmanFilter_Node::setMatrices()
{

    x_in << 0.0, 0.0, 0.0, 0.0, 0.0; // [x, y, vx, vy, yaw]

    P_in << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
    // State Transition Matrix
    F_in << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
    // Measurement Matrix
    H_in.setZero();
    // Measurement Noise Covariance Matrix
    R_in.setZero();
    // Process Noise Covariance Matrix
    Q_in << 0.1, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0,
        0, 0, 0.1, 0, 0,
        0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0.1;

    ekf.dt = 0.1;

    ekf.x_pred_ = x_in;
    ekf.z_pred_ = Vector4d::Zero();

    ekf.initialize(x_in, P_in, F_in, H_in, R_in, Q_in);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtendedKalmanFilter_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}