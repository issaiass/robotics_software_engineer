#include "algo/rrt_a_star.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

RRT_AStar::RRT_AStar() {}

void RRT_AStar::setDomain(const std::vector<int> &domain)
{
    RRT_Planner::setDomain(domain);
    AStar::setDomain(domain);
}

std::vector<Node_RRT> RRT_AStar::planPath(const Node_RRT &start, const Node_RRT &goal)
{
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Planning initial path using RRT...");
    return RRT_Planner::planPath(start, goal);
}

std::vector<geometry_msgs::msg::PoseStamped> RRT_AStar::refinePathWithAStar(const nav_msgs::msg::OccupancyGrid &grid)
{
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Refining path with A*...");
    std::vector<geometry_msgs::msg::PoseStamped> refined_path;
    std::vector<Node_RRT> rrt_path = RRT_Planner::planPath(Node_RRT(0, 0), Node_RRT(grid.info.width - 1, grid.info.height - 1));
    if (rrt_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("RRT_AStar"), "No initial path from RRT, skipping A* refinement.");
        return refined_path;
    }

    // Convert RRT path to A* nodes
    std::vector<AStarNode> a_star_path;
    for (const auto &node : rrt_path)
    {
        a_star_path.emplace_back(node.getX(), node.getY());
    }

    // refine
    std::vector<AStarNode> optimized_path = AStar::findPath(grid, a_star_path.front(), a_star_path.back());

    // message conversion
    for (const auto &astar_node : optimized_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = astar_node.x;
        pose.pose.position.y = astar_node.y;
        pose.pose.position.z = 0.0;
        refined_path.push_back(pose);
    }

    return refined_path;
}