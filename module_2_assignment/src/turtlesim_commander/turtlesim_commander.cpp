#include "turtlesim_commander/turtlesim_commander.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

TurtlesimCommander::TurtlesimCommander() : Node("turtlesime_commander"), _dt_ticks(0), _target_x(0), _target_theta(0), 
_enable_linear(false), _enable_angular(false), _enable_spiral(false),  _angular_threshold(0.01), _timer_period(10ms),
initial_pose_received(false)
{
    // Declare the parameter with a default value
    this->declare_parameter<string>("shape", "square");
    this->declare_parameter<float>("linear", 1.0);    
    this->declare_parameter<float>("a", 0.4);
    this->declare_parameter<float>("b", 0.3);
    _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    _subscriber_pose = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtlesimCommander::pose_callback, this, _1));
    _timer = this->create_wall_timer(_timer_period, std::bind(&TurtlesimCommander::timer_callback, this));
}

void TurtlesimCommander::move_linear(float linear_speed, float linear_time) {
  _tw.linear.x = linear_speed;
  _dt_ticks = linear_time/(_timer_period.count()/1000.0);
  _target_x += this->compute_distance(_pose, linear_speed*linear_time);
  RCLCPP_INFO(this->get_logger(), "next target x: '%.3f'", _target_x);  
  _enable_linear  = true;
  while (_enable_linear) {
    rclcpp::spin_some(this->get_node_base_interface());
  }

}

float TurtlesimCommander::fix_angle(float angle) {
    angle *=M_PI/180;
    float decision_angle = fmod(angle, 2*M_PI);
    decision_angle = abs(decision_angle) > M_PI ? -2*M_PI + abs(decision_angle) : decision_angle;    
    cout << "Angle corrected " << decision_angle << endl;
    return decision_angle;
}

void TurtlesimCommander::move_angular(float angle, float angular_speed) {
  _target_theta = fix_angle(angle);
  _tw.angular.z = _target_theta < 0 ? angular_speed : -angular_speed;  
  cout << _pose.theta << ", " << _target_theta << endl;
  RCLCPP_INFO(this->get_logger(), "next target theta: '%.3f'", _target_theta);    
  _enable_angular = true;
  while (_enable_angular) {  
    rclcpp::spin_some(this->get_node_base_interface());    
  }
}

float TurtlesimCommander::compute_distance(turtlesim::msg::Pose pose, float x) {
  return sqrt(pow(pose.x + x, 2) + pow(pose.y, 2));
}

void TurtlesimCommander::timer_callback() {
  if (_dt_ticks) {
    _dt_ticks -= 1;  
  }
  if (_dt_ticks <= 0 && _enable_linear) {
    _tw.linear.x = 0.0;
    _enable_linear = false;
    RCLCPP_INFO(this->get_logger(), "Stop Linear");
  }
  if (abs(_pose.theta - _target_theta) < _angular_threshold && _enable_angular) {
    _tw.angular.z = 0.0;
    _enable_angular = false;
    RCLCPP_INFO(this->get_logger(), "Stop Angular");
  }
  if (_enable_spiral) {
    static int i = 0;
    i++;
    if (fmod(i, 100)==0) {
      static float vel;
      static float angle = 0;
      vel = _a*exp(_b*angle);
      RCLCPP_INFO(this->get_logger(), "spiral growing vel '%.3f'", vel); 
      _tw.linear.x  = vel * 0.1;
      _tw.angular.z = 0.3 ;
      angle += 0.1;
      if (_pose.x >= 10.5 || _pose.y >= 10.5 || _pose.x <= 0.1 || _pose.y <= 0.1) {
        _tw.linear.x = 0;
        _tw.angular.z = 0;
      }
    }
  }    
  _publisher_cmd_vel->publish(_tw);
}

void TurtlesimCommander::pose_callback(const std::shared_ptr<turtlesim::msg::Pose> msg) {
  _pose = *msg;
  initial_pose_received = true;
}

void TurtlesimCommander::draw_n_point(int n, float linear_speed, float linear_time, float angular_speed, int repetitions) {
  while (repetitions--) {
    float step = n? 360/n : 90;
    float angle = step;
    while(n--) {
      move_angular(angle, angular_speed=angular_speed);
      move_linear(linear_speed=linear_speed, linear_time=linear_time);
      angle += step;
    }
  }
}

void TurtlesimCommander::draw_5_point_star(float linear_speed, float linear_time, float angular_speed, int repetitions) {
  while(repetitions--) {
    std::vector<int> angles = {144, 288, 432, 576, 720};
    for (int angle : angles) {
      move_angular(angle=angle, angular_speed=angular_speed);
      move_linear(linear_speed=linear_speed, linear_time=linear_time);
    }
  }

}

void TurtlesimCommander::draw_circle(float linear_speed, float angular_speed) {
    _tw.linear.x = linear_speed;
    _tw.angular.z = angular_speed;
    _publisher_cmd_vel->publish(_tw);
}

void TurtlesimCommander::draw_log_spiral(float a, float b) {
  _enable_spiral = true;
  _a = a;
  _b = b;
  RCLCPP_INFO(this->get_logger(), "a '%.3f'", _a); 
  RCLCPP_INFO(this->get_logger(), "b '%.3f'", _b);   
}