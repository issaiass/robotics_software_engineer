#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPlanning : public rclcpp::Node {
public:
  PathPlanning();

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid);
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Time start_time_;
};