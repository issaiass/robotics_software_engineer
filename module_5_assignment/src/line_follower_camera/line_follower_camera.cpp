#include "line_follower_camera/line_follower_camera.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

LineFollowerCamera::LineFollowerCamera() : Node("line_follower_camera"), _ctr1(0), _ctr2(0), _ctr3(0), _ctr4(0), _angular_velocity(0.3)
{

    const double ANGULAR_Z = 0.5;    // 0.5
    const double KP_ANGULAR = 0.01;  // 0.01
    const double KI_ANGULAR = 0.01;  // 0.01
    const double KD_ANGULAR = 0.00; // 0.001

    const double LINEAR_X = 0.22;     // 0.3
    const double KP_LINEAR = 0.1;    // 0.1
    const double KI_LINEAR = 0.01;   // 0.01
    const double KD_LINEAR = 0.0;  // 0.01

    this->declare_parameter<int>("low_threshold", 200);
    this->declare_parameter<int>("high_threshold", 250);

    this->declare_parameter<double>("Kp_angular", KP_ANGULAR);
    this->declare_parameter<double>("Ki_angular", KI_ANGULAR);
    this->declare_parameter<double>("Kd_angular", KD_ANGULAR);


    this->declare_parameter<double>("linear_sp", LINEAR_X);
    this->declare_parameter<double>("Kp_linear", KP_LINEAR);
    this->declare_parameter<double>("Ki_linear", KI_LINEAR);
    this->declare_parameter<double>("Kd_linear", KD_LINEAR);

 

    _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscriber_camera_image_raw = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&LineFollowerCamera::image_callback, this, _1));
    _timer = this->create_wall_timer(100ms, std::bind(&LineFollowerCamera::timer_callback, this));
    _angular_pid = std::make_unique<PID>(KP_ANGULAR, KI_ANGULAR, KD_ANGULAR, -ANGULAR_Z, ANGULAR_Z); // Kp, Ki, Kd, dt, cmd_min, cmd_max
    _linear_pid = std::make_unique<PID>(KP_LINEAR, KI_LINEAR, KD_LINEAR, -LINEAR_X, LINEAR_X);

    
    _logFile.open("log_task1.csv", ios::out | ios::app); // Open in append mode   
    _logFile << "Linear_Setpoint"  << "," << "Linear_Velocity" << "," << "Robot_Pixel" << "," << "Robot_Pixel_Setpoint" << "," \
             << "Angular_Velocity" << std::endl;
}

void LineFollowerCamera::timer_callback()
{
    // static rclcpp::Time last_time = this->now();
    // rclcpp::Time  current_time = this->now();
    // _dt = (current_time - last_time).nanoseconds() / 1e9;     // Calculate delta time in seconds
    // RCLCPP_INFO(this->get_logger(), "Delta time: %.2fs", _dt);    // Output the delta time
    // last_time = current_time; // Update the last_time_ to the current time
    _update_pid = true;
}

void LineFollowerCamera::image_callback(const std::shared_ptr<sensor_msgs::msg::Image> camera_msg)
{
    static vector<int> prev_edge;
    int low_threshold_ = this->get_parameter("low_threshold").as_int();
    int high_threshold_ = this->get_parameter("high_threshold").as_int();

    _angular_pid->_Kp = this->get_parameter("Kp_angular").as_double();
    _angular_pid->_Ki = this->get_parameter("Ki_angular").as_double();
    _linear_pid->_Kd = this->get_parameter("Kd_linear").as_double();

    double linear_sp = this->get_parameter("linear_sp").as_double();
    _linear_pid->_Kp = this->get_parameter("Kp_linear").as_double();
    _linear_pid->_Ki = this->get_parameter("Ki_linear").as_double();
    _linear_pid->_Kd = this->get_parameter("Kd_linear").as_double();




    //   RCLCPP_INFO(this->get_logger(), "Get Image");

    // get the camera pointer
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(camera_msg, "bgr8");

    cv::Mat gray_img, canny_img;

    cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_img, canny_img, low_threshold_, high_threshold_);

    int row = 150;
    int column = 0;
    cv::Mat roi = canny_img(cv::Range(row, row + 240), cv::Range(column, column + 640));

    vector<int> edge;

    for (int i = 0; i < 640; i++)
    {
        if (roi.at<uchar>(160, i) == 255)
        {
            edge.push_back(i);
        }
    }

    switch (edge.size())
    {
    case 1:
        _ctr1 += 1;
        break;
    case 2:
        _ctr2 += 1;
        break;
    case 3:
        _ctr3 += 1;
        break;
    case 4:
        _ctr4 += 1;
        break;
    default:
        break;
    }

    // RCLCPP_INFO(this->get_logger(), "ctr1 '%i'", _ctr1);
    // RCLCPP_INFO(this->get_logger(), "ctr2 '%i'", _ctr2);
    // RCLCPP_INFO(this->get_logger(), "ctr3 '%i'", _ctr3);
    // RCLCPP_INFO(this->get_logger(), "ctr4 '%i'", _ctr4);
    if (edge.empty()) {
        edge = prev_edge;
    }

    int mid_area = abs(edge[1] - edge[0]);
    int mid_point = edge[0] + mid_area / 2;
    int robot_midpoint = 640 / 2;

    cv::circle(roi, cv::Point(mid_point, 160), 2, cv::Scalar(255, 255, 255), -1);
    cv::circle(roi, cv::Point(robot_midpoint, 160), 5, cv::Scalar(255, 255, 255), -1);

    cv::imshow("Canny Img", roi);
    cv::waitKey(1);

    // float error = robot_midpoint - mid_point;
    geometry_msgs::msg::Twist velocity_msg_;
    if (_update_pid)
    {
        double angular_speed = -_angular_pid->stepPID(static_cast<double>(robot_midpoint), static_cast<double>(mid_point), 0.1);
        double linear_speed  = -_linear_pid->stepPID(velocity_msg_.linear.x, linear_sp, _dt)/(1+velocity_msg_.angular.z);
        velocity_msg_.angular.z = angular_speed/(1+1.5*abs(linear_speed)); // penalize curvee
        velocity_msg_.linear.x  = linear_speed/(1+1.5*abs(velocity_msg_.angular.z)); // penalize curves
        RCLCPP_INFO(this->get_logger(), "Linear: %.3f / Angular: %.3f", velocity_msg_.linear.x, velocity_msg_.angular.z); // Output the delta time
        _publisher_cmd_vel->publish(velocity_msg_);
        _update_pid = false;

        _logFile << linear_sp << "," << velocity_msg_.linear.x << "," << mid_point << "," << robot_midpoint << "," \
                 << velocity_msg_.angular.z << std::endl;
    }
    
    if (!edge.empty()) {
        prev_edge = edge;
    }
}


LineFollowerCamera::~LineFollowerCamera()
{
    _logFile.close();
    RCLCPP_INFO(this->get_logger(), "Bye bye bye!");
}