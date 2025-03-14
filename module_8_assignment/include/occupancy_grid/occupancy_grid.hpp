#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;
class OccupancyGrid_Publisher : public rclcpp::Node
{
public:
  OccupancyGrid_Publisher();

private:
  void og_callback();
  std::function<void(int)> add_line;
  std::function<void(int)> add_diag;  
  rclcpp::TimerBase::SharedPtr og_timer;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;
};

#endif // OCCUPANCY_GRID_HPP