#include "line_follower_camera/line_follower_camera.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

LineFollowerCamera::LineFollowerCamera() : Node("line_follower_camera"), _ctr1(0), _ctr2(0), _ctr3(0), _ctr4(0), _angular_velocity(0.3)
{
    this->declare_parameter<int>("low_threshold", 200);
    this->declare_parameter<int>("high_threshold", 250);

    _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscriber_camera_image_raw = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&LineFollowerCamera::image_callback, this, _1));
    _timer = this->create_wall_timer(500ms, std::bind(&LineFollowerCamera::timer_callback, this));
}

void LineFollowerCamera::timer_callback()
{
    ;
}

void LineFollowerCamera::image_callback(const std::shared_ptr<sensor_msgs::msg::Image> camera_msg)
{
    int low_threshold_ = this->get_parameter("low_threshold").as_int();
    int high_threshold_ = this->get_parameter("high_threshold").as_int();

    RCLCPP_INFO(this->get_logger(), "Get Image");

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
    }

    RCLCPP_INFO(this->get_logger(), "ctr1 '%i'", _ctr1);
    RCLCPP_INFO(this->get_logger(), "ctr2 '%i'", _ctr2);
    RCLCPP_INFO(this->get_logger(), "ctr3 '%i'", _ctr3);
    RCLCPP_INFO(this->get_logger(), "ctr4 '%i'", _ctr4);

    int mid_area = abs(edge[1] - edge[0]);
    int mid_point = edge[0] + mid_area/2;
    int robot_midpoint = 640/2;

    cv::circle(roi, cv::Point(mid_point, 160), 2, cv::Scalar(255, 255, 255), -1);
    cv::circle(roi, cv::Point(robot_midpoint, 160), 5, cv::Scalar(255, 255, 255), -1);    

    cv::imshow("Canny Img", roi);
    cv::waitKey(1);

      float error = robot_midpoint - mid_point;
      geometry_msgs::msg::Twist velocity_msg_;
      velocity_msg_.linear.x = 0.1;
      if (error < 0) {
        velocity_msg_.angular.z = -_angular_velocity;
      } else {
        velocity_msg_.angular.z = _angular_velocity;
      }

      _publisher_cmd_vel->publish(velocity_msg_);

}

LineFollowerCamera::~LineFollowerCamera()
{
    RCLCPP_INFO(this->get_logger(), "Bye bye bye!");
}