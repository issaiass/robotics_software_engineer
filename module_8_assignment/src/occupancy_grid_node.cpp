#include "rclcpp/rclcpp.hpp"
#include "occupancy_grid/occupancy_grid.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}