#include "path_planning/path_planning.hpp"
#include "node/node_rrt.hpp"
#include "algo/a_star.h"
#include "algo/rrt.hpp"
#include "algo/rrt_a_star.hpp"


PathPlanning::PathPlanning() : Node("path_planning_node")
{ // Set your algorithm flags here

  RCLCPP_INFO(this->get_logger(), "Performing Path Planning Search");
  occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&PathPlanning::occupancyGridCallback, this, std::placeholders::_1));

  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
}

void PathPlanning::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid)
{
  RCLCPP_INFO(this->get_logger(), "Received occupancy grid with dimensions: %d x %d", grid.info.width, grid.info.height);

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = grid.header.frame_id;
  path_msg.header.stamp = grid.header.stamp;

  start_time_ = this->get_clock()->now();
  path_msg.poses = rrt_a_star(grid); // a_star(grid); rrt(grid); rrt_a_star(grid);
  rclcpp::Duration elapsed_time = this->get_clock()->now() - start_time_;
  RCLCPP_INFO(this->get_logger(), "Elapsed time: %.9f seconds", elapsed_time.seconds());
  path_publisher_->publish(path_msg);
}