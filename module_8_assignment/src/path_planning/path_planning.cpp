#include "path_planning/path_planning.hpp"
#include "node/node_rrt.hpp"
//#include "algo/grid_sweep.h"
#include "algo/a_star.h"
#include "algo/rrt.hpp"


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
  // path_msg.poses = grid_sweep(grid);
  path_msg.poses = a_star(grid);

  // RCLCPP_DEBUG(this->get_logger(), "Initializing RRT Planner");

  // RRT_Planner planner;
  // std::vector<int> domain(grid.data.begin(), grid.data.end());
  // planner.setDomain(domain);

  // Node_RRT start(0, 0);
  // Node_RRT goal(7, 7);
  // std::vector<Node_RRT> path = planner.planPath(start, goal);

  // RCLCPP_INFO(this->get_logger(), "Path Planning from (%d, %d) to (%d, %d)", start.getX(), start.getY(), goal.getX(), goal.getY());
  // if (path.empty())
  // {
  //   RCLCPP_WARN(this->get_logger(), "No path found");
  // } else {
  //   for (auto &node : path)
  //   {
  //     RCLCPP_INFO(this->get_logger(), "Path node: (%d, %d)", node.getX(), node.getY());
  //     geometry_msgs::msg::PoseStamped pose;
  //     pose.pose.position.x = node.getX() - 5.0;
  //     pose.pose.position.y = node.getY() - 5.0;
  //     pose.pose.position.z = 0.0;
  //     path_msg.poses.push_back(pose);
  //   }
  // }

  rclcpp::Duration elapsed_time = this->get_clock()->now() - start_time_;

  // Print elapsed time in seconds
  RCLCPP_INFO(this->get_logger(), "Elapsed time: %.9f seconds", elapsed_time.seconds());
    

  // call the planner
  path_publisher_->publish(path_msg);
}