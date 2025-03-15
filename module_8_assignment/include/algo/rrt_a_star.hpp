#ifndef RRT_A_STAR_HPP
#define RRT_A_STAR_HPP

#include "algo/a_star.h"
#include "algo/rrt.hpp"
#include "node/node_rrt.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

class RRT_AStar {
public:
    RRT_AStar();
    std::vector<geometry_msgs::msg::PoseStamped> planPath(const nav_msgs::msg::OccupancyGrid &grid);

private:
    RRT_Planner rrt_planner;
};

std::vector<geometry_msgs::msg::PoseStamped> rrt_a_star(const nav_msgs::msg::OccupancyGrid &grid);

#endif // RRT_A_STAR_HPP
