#include "maze_solver/maze_solver.h"
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

MazeSolver::MazeSolver() : 
  Node("maze_solver"), 
  THRESHOLD(3.0f), 
  ANGULAR_VEL(1.3f), 
  LINEAR_VEL(0.5f), 
  _state(RobotStates::MOVE_STRAIGHT), 
  _last_time(this->now()),
  _cmd_vel()
{
    _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscriber_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&MazeSolver::scan_callback, this, _1));
    _subscriber_imu = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&MazeSolver::imu_callback, this, _1));
    _timer = this->create_wall_timer(500ms, std::bind(&MazeSolver::timer_callback, this));
    float theta_ = 135.0 * M_PI / 180.0;
    COS_THETA = cos(theta_);
    SIN_THETA = sin(theta_);
}

void MazeSolver::timer_callback()
{
    auto current_time = this->now();
    float dt_ = (current_time - _last_time).nanoseconds() / 1e9; // 
    _last_time = current_time;

    geometry_msgs::msg::Twist cmd_vel_;
    geometry_msgs::msg::Vector3 acc_;
    acc_ = _imu_msg.linear_acceleration;

    // Apply rotation matrix to the linear acceleration vector   
    geometry_msgs::msg::Vector3 acc_rot;
    acc_rot.x = acc_.x * COS_THETA - acc_.y * SIN_THETA;
    acc_rot.y = acc_.x * SIN_THETA + acc_.y * COS_THETA;
    acc_rot.z = acc_.z;

    // Get the angular velocity
    cmd_vel_.angular = _imu_msg.angular_velocity;
    
    // Integrate acceleration to get velocity
    const float Kp = 27.5;
    cmd_vel_.linear.x += abs(cmd_vel_.linear.x) > LINEAR_VEL ? LINEAR_VEL : Kp*acc_rot.x*dt_;
    cmd_vel_.linear.y += abs(cmd_vel_.linear.y) > LINEAR_VEL ? LINEAR_VEL : Kp*acc_rot.y*dt_;
    cmd_vel_.linear.z += abs(cmd_vel_.linear.z) > LINEAR_VEL ? LINEAR_VEL : Kp*acc_rot.z*dt_;

    RCLCPP_INFO(this->get_logger(), "delta_t ('%.3f') / cmd_vel.linear.x ('%.3f') / cmd_vel.angular.z ('%.3f')", dt_, cmd_vel_.linear.x, cmd_vel_.angular.z);
}


void MazeSolver::imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) {
    _imu_msg = *imu_msg;
}

/*
void MazeSolver::cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_msg) {
    //_prev_vel = cmd_vel_msg->linear;
}
*/


void MazeSolver::scan_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg)
{
    float max_ = std::numeric_limits<float>::max();
    static bool status_ = false;
    geometry_msgs::msg::Twist tw_;

    float right_obstacle_ = *min_element(scan_msg->ranges.begin() + 260, scan_msg->ranges.begin() + 280);
    float front_obstacle_ = *min_element(scan_msg->ranges.begin() + 340, scan_msg->ranges.begin() + 360);
    float left_obstacle_ = *min_element(scan_msg->ranges.begin() + 80, scan_msg->ranges.begin() + 100);

  //  RCLCPP_INFO(this->get_logger(), "scan: Left ('%f') / Front('%f') / Right('%f')", left_obstacle_, front_obstacle_, right_obstacle_);

    switch (_state)
    {
    case RobotStates::MOVE_STRAIGHT:
        if (front_obstacle_ < THRESHOLD)
        {
            if (left_obstacle_ < THRESHOLD)
            {
                _state = RobotStates::TURN_RIGHT;
            }
            if (right_obstacle_ < THRESHOLD)
            {
                _state = RobotStates::TURN_LEFT;
            }
        }
        if (left_obstacle_ > max_ && front_obstacle_ > max_ && right_obstacle_ > max_ && status_)
        {
            _state = RobotStates::OUT_OF_MAZE;
        }
        break;
    case RobotStates::TURN_LEFT:
        if (front_obstacle_ > THRESHOLD)
        {
            _state = RobotStates::MOVE_STRAIGHT;
            status_ = true;
        }
        break;
    case RobotStates::TURN_RIGHT:
        if (front_obstacle_ > THRESHOLD)
        {
            _state = RobotStates::MOVE_STRAIGHT;
            status_ = true;
        }
        break;
    case RobotStates::OUT_OF_MAZE:
    default:
        break;
    }

    switch (_state)
    {
    case RobotStates::MOVE_STRAIGHT:
  //      RCLCPP_INFO(this->get_logger(), "Moving Straight");
        tw_.linear.x = LINEAR_VEL;
        break;
    case RobotStates::TURN_LEFT:
        tw_.linear.x = 0.0;
        tw_.angular.z = ANGULAR_VEL;
        break;
    case RobotStates::TURN_RIGHT:
   //     RCLCPP_INFO(this->get_logger(), "Turning Right");
        tw_.linear.x = 0.0;
        tw_.angular.z = -ANGULAR_VEL;
        break;
    case RobotStates::OUT_OF_MAZE:
 //       RCLCPP_INFO(this->get_logger(), "Out of Maze");
        tw_.linear.x = 0.0;
        tw_.angular.z = 0.0;
        break;
    }
    _publisher_cmd_vel->publish(tw_);
}

MazeSolver::~MazeSolver()
{
    RCLCPP_INFO(this->get_logger(), "Bye bye bye!");
}