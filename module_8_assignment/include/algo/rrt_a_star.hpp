// rrt_a_star.hpp
#ifndef RRT_ASTAR_HPP
#define RRT_ASTAR_HPP

#include "algo/a_star.h"
#include "algo/rrt.hpp"
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class RRT_AStar : public RRT_Planner  // Only inherit from RRT_Planner
{
public:
    RRT_AStar();
    
    // Use RRT to find a coarse path
    std::vector<Node_RRT> planRRTPath(const Node_RRT &start, const Node_RRT &goal);
    
    // Refine the path using A*
    std::vector<geometry_msgs::msg::PoseStamped> refinePathWithAStar(
        const nav_msgs::msg::OccupancyGrid &grid,
        const std::vector<Node_RRT> &rrt_path);
    
    // Set domain from occupancy grid
    void setDomain(const std::vector<int> &domain);
};

// Function to be called from PathPlanning::occupancyGridCallback
std::vector<geometry_msgs::msg::PoseStamped> rrt_a_star(const nav_msgs::msg::OccupancyGrid& grid);

#endif // RRT_ASTAR_HPP