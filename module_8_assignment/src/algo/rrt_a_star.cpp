#include "algo/rrt_a_star.hpp"
#include <algorithm>

RRT_AStar::RRT_AStar() {}

std::vector<geometry_msgs::msg::PoseStamped> RRT_AStar::planPath(const nav_msgs::msg::OccupancyGrid &grid)
{
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Starting RRT-A* Hybrid Planning...");

    std::vector<int> domain(grid.data.begin(), grid.data.end()); // set the grid
    rrt_planner.setDomain(domain);

    Node_RRT start(0, 0);
    Node_RRT goal(7, 7);
    std::vector<Node_RRT> rrt_path = rrt_planner.planPath(start, goal); // run rrt

    if (rrt_path.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("RRT_AStar"), "RRT failed to find a path!");
        return {};
    }

    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "RRT found an initial path with %lu nodes", rrt_path.size());

    // Step 2: Run A* to refine the path
    std::vector<geometry_msgs::msg::PoseStamped> refined_path = a_star(grid);

    if (refined_path.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("RRT_AStar"), "A* failed to refine the RRT path!");
        return refined_path;
    }

    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "A* refined the path with %lu nodes", refined_path.size());

    // Step 3: Print the final refined path
    for (auto &node : refined_path)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("RRT_AStar"), "Path node: (%i, %i)", 
            static_cast<int>(node.pose.position.x + 5), 
            static_cast<int>(node.pose.position.y + 5)
        );
    }

    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Finished RRT-A* Hybrid Planning.");
    return refined_path;
}

std::vector<geometry_msgs::msg::PoseStamped> rrt_a_star(const nav_msgs::msg::OccupancyGrid &grid)
{
    RRT_AStar hybrid_planner;
    return hybrid_planner.planPath(grid);
}
