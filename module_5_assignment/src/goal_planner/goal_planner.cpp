#include "goal_planner/goal_planner.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

GoalPlanner::GoalPlanner() : Node("goal_planner")
{
    const double D_THRESHOLD = 0.1;
    const double YAW_THRESHOLD = 0.1;

    const double LINEAR_KP = 0.3;
    const double LINEAR_KI = 0.003;
    const double LINEAR_KD = 0.05;
    const double LINEAR_CMD_MAX = 1.0;
    const double LINEAR_CMD_MIN = -1.0;

    const double ANGULAR_KP = 0.7;
    const double ANGULAR_KI = 0.02;
    const double ANGULAR_KD = 0.3;
    const double ANGULAR_CMD_MAX = 1.0;
    const double ANGULAR_CMD_MIN = -1.0;

    this->declare_parameter<double>("d_threshold", D_THRESHOLD);
    this->declare_parameter<double>("yaw_threshold", YAW_THRESHOLD);
    this->declare_parameter<double>("linear_kp", LINEAR_KP);
    this->declare_parameter<double>("linear_ki", LINEAR_KI);
    this->declare_parameter<double>("linear_kd", LINEAR_KD);
    this->declare_parameter<double>("angular_kp", ANGULAR_KP);
    this->declare_parameter<double>("angular_ki", ANGULAR_KI);
    this->declare_parameter<double>("angular_kd", ANGULAR_KD);

    YAML::Node config = YAML::LoadFile("module_5_assignment/config/routes.yaml");

    // Extract start, goal, and waypoints from the YAML file
    WayPointLib::XYTheta start = config["start"].as<vector<float>>();
    WayPointLib::XYTheta goal = config["goal"].as<vector<float>>();
    WayPointLib::Path midpoints1 = config["midpoints1"].as<std::vector<std::vector<float>>>();
    WayPointLib::Path midpoints2 = config["midpoints2"].as<std::vector<std::vector<float>>>();
    WayPointLib::Path midpoints3 = config["midpoints3"].as<std::vector<std::vector<float>>>();

    // call the waypoints constructor
    auto wpl = std::make_shared<WayPointLib>();

    // make paths
    WayPointLib::Path path1 = wpl->make_path(start, goal, midpoints1);
    WayPointLib::Path path2 = wpl->make_path(start, goal, midpoints2);
    WayPointLib::Path path3 = wpl->make_path(start, goal, midpoints3);

    // compute and get the waypoints
    vector<WayPointLib::Path> pathv{path1, path2, path3};
    _path = wpl->get_lower_energy_path(pathv);

    this->declare_parameter<double>("goal_x", static_cast<double>(_path[0][0]));
    this->declare_parameter<double>("goal_y", static_cast<double>(_path[0][1]));

    _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&GoalPlanner::odom_callback, this, _1));
    _timer = this->create_wall_timer(100ms, std::bind(&GoalPlanner::timer_callback, this));
    _linear_pid = std::make_unique<PID>(LINEAR_KP, LINEAR_KI, LINEAR_KD, LINEAR_CMD_MIN, LINEAR_CMD_MAX);
    _angular_pid = std::make_unique<PID>(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, ANGULAR_CMD_MIN, ANGULAR_CMD_MAX);

    _logFile.open("log_task2.csv", ios::out | ios::app); // Open in append mode
    _logFile << "X_Goal" << "," << "Y_Goal" << "," << "X_Pos" << "," << "Y_Pos" << std::endl;
}

void GoalPlanner::timer_callback()
{
    _update_pid = true;
}

void GoalPlanner::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg)
{
    if (!_path.empty())
    {
        double current_goal_x = static_cast<double>(_path[0][0]);
        double current_goal_y = static_cast<double>(_path[0][1]);
        this->set_parameter(rclcpp::Parameter("goal_x", current_goal_x));
        this->set_parameter(rclcpp::Parameter("goal_y", current_goal_y));
    }

    double goal_x_ = this->get_parameter("goal_x").as_double();
    double goal_y_ = this->get_parameter("goal_y").as_double();
    double d_threshold_ = this->get_parameter("d_threshold").as_double();
    double yaw_threshold_ = this->get_parameter("yaw_threshold").as_double();

    double linear_kp_ = this->get_parameter("linear_kp").as_double();
    double linear_ki_ = this->get_parameter("linear_ki").as_double();
    double linear_kd_ = this->get_parameter("linear_kd").as_double();

    double angular_kp_ = this->get_parameter("angular_kp").as_double();
    double angular_ki_ = this->get_parameter("angular_ki").as_double();
    double angular_kd_ = this->get_parameter("angular_kd").as_double();

    double current_x_ = odom_msg->pose.pose.position.x;
    double current_y_ = odom_msg->pose.pose.position.y;

    tf2::Matrix3x3 current_angles_ = this->get_euler_from_quaternion(odom_msg->pose.pose.orientation);
    double roll_, pitch_, current_yaw_;
    current_angles_.getRPY(roll_, pitch_, current_yaw_);

    float error_x_ = goal_x_ - current_x_;
    float error_y_ = goal_y_ - current_y_;

    double d_error_ = sqrt(pow(error_x_, 2) + pow(error_y_, 2));
    double yaw_error_ = atan2(error_y_, error_x_) - current_yaw_;
    yaw_error_ = atan2(sin(yaw_error_), cos(yaw_error_));

    if (_update_pid)
    {
        geometry_msgs::msg::Twist tw_;
        tw_.linear.x = d_error_;
        if (d_error_ > d_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "Movement");
            tw_.linear.x = -_linear_pid->stepPID(d_error_, d_threshold_, 0.1);
            tw_.angular.z = -_angular_pid->stepPID(yaw_error_, yaw_threshold_, 0.1);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Stop");
            tw_.linear.x = 0.0;
            tw_.angular.z = 0.0;
            if (!_path.empty())
            {
                _path.erase(_path.begin());
            }
        }

        _publisher_cmd_vel->publish(tw_);
        _update_pid = false;
    }
    _logFile << goal_x_ << "," << goal_y_ << "," << current_x_ << "," << current_y_ << std::endl;
    RCLCPP_INFO(this->get_logger(), "Goal: ('%.3f', '%.3f') / Position: ('%.3f', '%.3f') / Pose Error: '%.3f' / Yaw Error: '%.3f'",
                goal_x_, goal_y_, current_x_, current_y_, d_error_, yaw_error_);
}

tf2::Matrix3x3 GoalPlanner::get_euler_from_quaternion(const geometry_msgs::msg::Quaternion &quat_msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat_msg, q);
    tf2::Matrix3x3 m(q);
    return m;
}

tf2::Quaternion *GoalPlanner::get_quaternion_from_euler(double roll, double pitch, double yaw)
{

    tf2::Quaternion *q;
    q->setRPY(roll, pitch, yaw);
    return q;
}

GoalPlanner::~GoalPlanner()
{
    _logFile.close();
    RCLCPP_INFO(this->get_logger(), "Bye! Bye! Bye!");
}