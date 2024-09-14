#include "arm_path_planner/arm_path_planner.h"

ArmPathPlanner::ArmPathPlanner() : Node("arm_path_planner")
{

    _publisher_joint_trajectory = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
    _publisher_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("/arm_markers", 10);


    YAML::Node config = YAML::LoadFile("module_5_assignment/config/arm_waypoints.yaml");

    if (config["waypoints"])
    {
        for (const auto &wp : config["waypoints"])
        {
            vector<float> p = wp.as<vector<float>>();
            _waypoints.push_back(p);
        }
    }


    // Wait for the trajectory marker subscription
    while (_publisher_marker->get_subscription_count() < 1)
    {
        if (!rclcpp::ok())
            return;
        RCLCPP_INFO(this->get_logger(), "Please create a subscriber to the marker");
        rclcpp::sleep_for(1s);
    }
    // wait for the arm controller to be ready
    while (_publisher_joint_trajectory->get_subscription_count() < 1) {
        if (!rclcpp::ok()) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for subscribers to /arm_controller/joint_trajectory");
        rclcpp::sleep_for(500ms);
    }

    publishTrajectoryMarker();
    _timer = this->create_wall_timer(200ms, std::bind(&ArmPathPlanner::timer_callback, this));
    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    _path_marker = this->create_publisher<visualization_msgs::msg::Marker>("/arm_followed_path", 10);
    publishTrajectory();
}


void ArmPathPlanner::timer_callback() {
    geometry_msgs::msg::TransformStamped t;
    try {
        t = _tf_buffer->lookupTransform("base_footprint", "gripper_extension", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "gripper_extension", "base_footprint", ex.what());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Joint (X,Y,Z) (%.4f, %.4f, %.4f)", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);

    line_strip.header.frame_id = "base_footprint";
    line_strip.header.stamp = this->now();;
    line_strip.ns = "/followed_path";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.id = 1;
    
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01;
    line_strip.scale.y = 0.01;
    line_strip.scale.z = 0.01;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.g = 165/255.0;   
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    // Set the marker lifetime.
    line_strip.lifetime = rclcpp::Duration(5, 0);

    // Create the vertices for the points and lines
    geometry_msgs::msg::Point p;
    p.x = t.transform.translation.x;
    p.y = t.transform.translation.y;
    p.z = t.transform.translation.z;
    //line_strip.points.push_back(p);
    _recent_points.push_back(p);

    if (_recent_points.size() > 50) {
        _recent_points.erase(_recent_points.begin());
    }

    line_strip.points.clear();
    for (const auto& point : _recent_points) {
        line_strip.points.push_back(point);
    }

    _path_marker->publish(line_strip);

}



void ArmPathPlanner::publishTrajectoryMarker()
{

    visualization_msgs::msg::Marker line_strip;

    string frame_id = "base_footprint";
    string ns = "/trajectory";

    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = this->now();
    line_strip.ns = ns;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.02;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    visualization_msgs::msg::MarkerArray text_array;
    for (size_t index = 0; index < _waypoints.size(); ++index)
    {
        geometry_msgs::msg::Point p;

        vector<float> wp = _waypoints[index];
        p.x = wp[0];
        p.y = wp[1];
        p.z = wp[2];
        line_strip.points.push_back(p);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "base_footprint";
        text_marker.header.stamp = this->now();
        text_marker.ns = "/labels";
        text_marker.id = index;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = wp[0];
        text_marker.pose.position.y = wp[1];
        text_marker.pose.position.z = wp[2] + 0.1;
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "Goal" + std::to_string(index + 1);
        text_array.markers.push_back(text_marker);
    }

    visualization_msgs::msg::MarkerArray line_array;
    line_array.markers.push_back(line_strip);
    _publisher_marker->publish(line_array);
    
    _publisher_marker->publish(text_array);
}

// Function to get the transformation matrix
Eigen::Matrix4d ArmPathPlanner::forward_kinematics(double theta_base, double theta_finger, double d_prismatic, double theta_ee) {
    Eigen::Matrix4d T_base, T_finger, T_prismatic, T_ee;
    // Base joint (rotation around Z, continuous joint)
    T_base << cos(theta_base), -sin(theta_base), 0, 0,
              sin(theta_base), cos(theta_base),  0, 0,
              0,               0,                1, 0.0375, // Offset by base height (0.075 from URDF)
              0,               0,                0, 1;

    // Finger joint (rotation around Y, revolute joint)
    // The finger is 0.5 meters long according to the URDF.
    T_finger << cos(theta_finger), 0, sin(theta_finger),   0,
                0,                 1,                 0,   0,
               -sin(theta_finger), 0, cos(theta_finger), 0.5, // Length of the finger link (0.5 meters)
                0,                 0,                 0,   1;

    // Prismatic joint (translation along Z, between gripper_base and gripper_extension)
    T_prismatic << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, d_prismatic,  // Translation along the Z-axis by d_prismatic
                   0, 0, 0, 1;

    // End-effector joint (rotation around Z, revolute joint)
    T_ee << cos(theta_ee), -sin(theta_ee), 0, 0,
            sin(theta_ee), cos(theta_ee),  0, 0,
            0,             0,              1, 0.005,  // Offset for the end-effector (0.005 meters)
            0,             0,              0, 1;

    // Multiply the transformations to get the total transformation
    Eigen::Matrix4d T_total = T_base * T_finger * T_prismatic * T_ee;

    return T_total;
}


// Inverse Kinematics: Compute joint states from desired end-effector position and orientation
Eigen::Vector4d ArmPathPlanner::inverse_kinematics(double x, double y, double z, double theta_ee) {
    Eigen::Vector4d joint_states;  // [theta_base, theta_finger, d_prismatic, theta_ee]

    // Base joint (theta_base): Compute the base rotation angle from (x, y)
    double theta = atan2(y, x);
    joint_states(0) = theta; //atan2(sin(theta), cos(theta));

    // Compute the distance in the XY plane from the base (r = sqrt(x^2 + y^2))
    double r = sqrt(x * x + y * y);

    // Compute the finger joint angle using the Z height and link length
    double base_height = 0.0375;  // Base height from the URDF
    double finger_length = 0.5;  // Length of the finger link

    // Compute the effective height (Z position) from the base height
    double z_eff = z - base_height;

    // Angle for the finger joint (rotation around Y-axis), offset by pi/2
    double distance_3d = sqrt(r * r + z_eff * z_eff);  // 3D distance to the target
    if (distance_3d > finger_length) {
        distance_3d = finger_length;  // Limit to max reach of the finger link
    }
        
    double theta_finger = atan2(z_eff, r) - M_PI/2;  // Updated finger angle calculation
    joint_states(1) = atan2(sin(theta_finger), cos(theta_finger));


    // Prismatic joint (d_prismatic): Compute the prismatic joint extension
    joint_states(2) = std::max(0.0, distance_3d - 0.2); // The remaining Z distance after extending the prismatic length

    // End-effector joint (theta_ee): End-effector rotation is given directly
    joint_states(3) = theta_ee;

    return joint_states;
}

void ArmPathPlanner::publishTrajectory() {
    trajectory_msgs::msg::JointTrajectory joint_traj;


    joint_traj.header.stamp = rclcpp::Time(0,0);
    joint_traj.header.frame_id = "";
    joint_traj.joint_names = {"base_joint", "finger_joint", "gripper_extension_joint", "ee_joint"};

    for (size_t i = 0; i < _waypoints.size(); i++) {
        auto wp_ = _waypoints[i];
        Eigen::Vector4d joint_states = inverse_kinematics(wp_[0], wp_[1], wp_[2], 0.0);
        trajectory_msgs::msg::JointTrajectoryPoint traj_point;
        traj_point.positions = {joint_states(0), joint_states(1), joint_states(2), joint_states(3)};
        RCLCPP_INFO(this->get_logger(), "Published trajectory: %.3f, %.3f, %.3f, %.3f", 
        joint_states(0),joint_states(1),joint_states(2),joint_states(3));
        traj_point.time_from_start = rclcpp::Duration::from_seconds(1.0*(i+1));
        joint_traj.points.push_back(traj_point);
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory with %zu points", joint_traj.points.size());
    _publisher_joint_trajectory->publish(joint_traj);
    RCLCPP_INFO(this->get_logger(), "Trajectory published");

}