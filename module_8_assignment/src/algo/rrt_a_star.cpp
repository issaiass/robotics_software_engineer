// rrt_a_star.cpp
#include "algo/rrt_a_star.hpp"

RRT_AStar::RRT_AStar() {}

void RRT_AStar::setDomain(const std::vector<int> &domain)
{
    // Convert const vector to non-const for RRT_Planner
    std::vector<int> mutable_domain(domain.begin(), domain.end());
    RRT_Planner::setDomain(mutable_domain);
}

std::vector<Node_RRT> RRT_AStar::planRRTPath(const Node_RRT &start, const Node_RRT &goal)
{
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Planning initial path using RRT...");
    return RRT_Planner::planPath(start, goal);
}

std::vector<geometry_msgs::msg::PoseStamped> RRT_AStar::refinePathWithAStar(
    const nav_msgs::msg::OccupancyGrid &grid,
    const std::vector<Node_RRT> &rrt_path)
{
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Refining path with A*...");
    std::vector<geometry_msgs::msg::PoseStamped> refined_path;
    
    if (rrt_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("RRT_AStar"), "Empty RRT path provided, nothing to refine.");
        return refined_path;
    }
    
    // Create start and goal points for A*
    const Node_RRT &start_node = rrt_path.front();
    const Node_RRT &goal_node = rrt_path.back();
    
    // Extract waypoints from RRT path for A* to connect
    std::vector<std::pair<int, int>> waypoints;
    for (const auto &node : rrt_path) {
        waypoints.emplace_back(node.getX(), node.getY());
    }
    
    // Connect consecutive waypoints using A*
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        auto start_pair = waypoints[i];
        auto goal_pair = waypoints[i + 1];
        
        // Set up start and end for A* segment
        auto start_index = start_pair.second * grid.info.width + start_pair.first;
        auto goal_index = goal_pair.second * grid.info.width + goal_pair.first;
        
        // Create a subgrid for A* to work with (optional optimization)
        nav_msgs::msg::OccupancyGrid subgrid = grid;
        
        // Run A* for this segment
        std::priority_queue<std::shared_ptr<NodeAstar>, std::vector<std::shared_ptr<NodeAstar>>, compare_node> open_list;
        float MAX = std::numeric_limits<float>::max();
        std::vector<std::vector<float>> cost_so_far(grid.info.height, std::vector<float>(grid.info.width, MAX));
        std::vector<std::vector<bool>> closed_list(grid.info.height, std::vector<bool>(grid.info.width, false));

        auto start_a_node = std::make_shared<NodeAstar>(start_pair.first, start_pair.second);
        start_a_node->set_gcost(0);
        start_a_node->set_hcost(heuristic(start_pair.first, start_pair.second, goal_pair.first, goal_pair.second));

        open_list.push(start_a_node);
        cost_so_far[start_pair.second][start_pair.first] = 0;

        std::vector<std::pair<int, int>> directions = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1},
            {3, 0}, {0, 3}, {-3, 0}, {0, -3}, {3, 3}, {3, -3}, {-3, 3}, {1, 1}, {1, 1}
        };
        
        bool found_path = false;
        std::shared_ptr<NodeAstar> goal_a_node = nullptr;

        while (!open_list.empty()) {
            auto current_node = open_list.top();
            open_list.pop();

            if (current_node->x == goal_pair.first && current_node->y == goal_pair.second) {
                found_path = true;
                goal_a_node = current_node;
                break;
            }

            closed_list[current_node->y][current_node->x] = true;

            // Explore neighbors
            for (auto dir: directions) {
                int new_x = current_node->x + dir.first;
                int new_y = current_node->y + dir.second;

                
                if (new_x >= 0 && new_x < grid.info.width && new_y >= 0 && new_y < grid.info.height) { // Check if new node is within the grid
                    int new_index = new_y * grid.info.width + new_x;
                    if (grid.data[new_index] == 100) {  // Check if the cell is an obstacle
                        continue;
                    }

                    float new_cost = current_node->g_cost + (dir.first == 0 || dir.second == 0 ? 1 : std::sqrt(2));

                    if (!closed_list[new_y][new_x]) {
                        auto neighbor = std::make_shared<NodeAstar>(new_x, new_y, current_node);

                        if (new_cost < cost_so_far[new_y][new_x]) {
                            cost_so_far[new_y][new_x] = new_cost;
                            neighbor->set_gcost(new_cost);
                            neighbor->set_hcost(heuristic(new_x, new_y, goal_pair.first, goal_pair.second));
                            open_list.push(neighbor);
                        }
                    }
                }
            }
        }
        
        if (found_path && goal_a_node) { // add the refined path for the found path
            std::vector<geometry_msgs::msg::PoseStamped> segment_path;
            auto path_node = goal_a_node;
            
            while (path_node != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = grid.header.frame_id;
                pose.header.stamp = grid.header.stamp;
                pose.pose.position.x = path_node->x - 5.0; 
                pose.pose.position.y = path_node->y - 5.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                
                segment_path.push_back(pose);
                path_node = path_node->parent;
            }            

            std::reverse(segment_path.begin(), segment_path.end()); // Reverse the path
            
            size_t start_idx = (i > 0) ? 1 : 0;             // Add segment path to refined path, skip the first point except for the first segment to avoid duplicates
            for (size_t j = start_idx; j < segment_path.size(); ++j) {
                refined_path.push_back(segment_path[j]); 
            }
        } else { // If A* fails use straight line (RRT path as fallback)
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = grid.header.frame_id;
            pose.header.stamp = grid.header.stamp;
            pose.pose.position.x = goal_pair.first - 5.0;
            pose.pose.position.y = goal_pair.second - 5.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            if (i > 0 || refined_path.empty()) {
                refined_path.push_back(pose);
            }
        }
    }
    
    return refined_path;
}

std::vector<geometry_msgs::msg::PoseStamped> rrt_a_star(const nav_msgs::msg::OccupancyGrid& grid)
{
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "Starting RRT-A* path planning...");
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = grid.header.frame_id;
    path_msg.header.stamp = grid.header.stamp;


    RRT_AStar planner;
    planner.setDomain(std::vector<int>(grid.data.begin(), grid.data.end()));
    
    // Set start and goal positions
    Node_RRT start(0, 0);
    Node_RRT goal(7, 7);
    
    // First, find a coarse path using RRT
    std::vector<Node_RRT> rrt_path = planner.planRRTPath(start, goal);
    
    if (rrt_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("RRT_AStar"), "No path found using RRT");
        return path_msg.poses;
    }
    
    // Then refine the path using A*
    std::vector<geometry_msgs::msg::PoseStamped> refined_path = planner.refinePathWithAStar(grid, rrt_path);
    
    if (refined_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("RRT_AStar"), "Failed to refine path with A*");
        
        // Fallback to converting RRT path directly to poses
        for (const auto &node : rrt_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = grid.header.frame_id;
            pose.header.stamp = grid.header.stamp;
            pose.pose.position.x = node.getX() - 5.0;
            pose.pose.position.y = node.getY() - 5.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            refined_path.push_back(pose);
        }
    }    

    for (auto &node : refined_path) {
        RCLCPP_INFO(
            rclcpp::get_logger("RRT_AStar"), "Path node: (%i, %i)", 
            static_cast<int>(node.pose.position.x+5), 
            static_cast<int>(node.pose.position.y+5)
        );
    }

    
    RCLCPP_INFO(rclcpp::get_logger("RRT_AStar"), "RRT-A* planning complete. Path size: %zu", refined_path.size());
    return refined_path;
}