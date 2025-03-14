#include "rclcpp/rclcpp.hpp"
#include "path_planning/path_planning.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanning>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}