#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TurtlesimCommander : public rclcpp::Node {
  public:
    TurtlesimCommander();
    bool initial_pose_received;
    void draw_n_point(int n, float linear_speed = 1.0, float linear_time = 2.0, float angular_speed = 0.2, int repetitions = 1);
    void draw_5_point_star(float linear_speed = 1.0, float linear_time = 2.0, float angular_speed = 0.2, int repetitions = 1);
    void draw_circle(float linear_speed = 1.0, float angular_speed = 0.2);
    void draw_log_spiral(float a = 0.4, float b = 0.3);

  private:
    void timer_callback();
    void pose_callback(const std::shared_ptr<turtlesim::msg::Pose> msg);
    void move_linear(float linear_speed = 1.0, float linear_time = 1);
    void move_angular(float angle = 0.0, float angular_speed = 0.1);
    
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher_cmd_vel;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _subscriber_pose;  
    turtlesim::msg::Pose _pose;
    geometry_msgs::msg::Twist _tw;    
    std::chrono::milliseconds _timer_period;
    std::string shape;
    float linear;    
    float _a;
    float _b;

    float _dt_ticks;
    float _target_x;
    float _target_theta;
    bool _enable_linear;
    bool _enable_angular;
    bool _enable_spiral;
    float _angular_threshold;

  protected:
    float fix_angle(float angle);
    float compute_distance(turtlesim::msg::Pose pose, float x = 0);
};