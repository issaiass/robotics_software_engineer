#ifndef RRT_HPP
#define RRT_HPP

#include <array>
#include <memory>
#include <vector>
#include <random>
#include "node/node_rrt.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


class RRT_Planner {
  public:
    RRT_Planner();
    Node_RRT START = Node_RRT(0, 0);
    Node_RRT GOAL = Node_RRT(10, 10);

    void setDomain(std::vector<int> &domain);
    static Node_RRT indexToCoordinate(int index);

    Node_RRT generateRandomNode(std::mt19937 &gen);
    bool isObstacle(Node_RRT &new_node, Node_RRT const &goal);
    bool isGoalFound(Node_RRT &newNode, Node_RRT const &goal);
    Node_RRT findNearestNode(std::vector<Node_RRT> const &nodes, Node_RRT const &randdom_node);
    Node_RRT findNewConfig(Node_RRT const &nearest_node, Node_RRT const &random_node);

    std::vector<Node_RRT> planPath(Node_RRT const &start, Node_RRT const &goal);

  private:
    void Occupancy_Callback();
    void publishPath(std::vector<Node_RRT> const &path);
    
    int const MAX_ITERATIONS = 1000000;
    int const SEED = 0;

    static constexpr int GRID_WIDTH = 10;
    static constexpr int GRID_HEIGHT = 10;
    static constexpr float STEP_SIZE = 1.0f;
    
    std::array<int, GRID_WIDTH * GRID_HEIGHT> DOMAIN{};
    std::vector<Node_RRT> nodes;
    std::vector<Node_RRT> path;

};

std::vector<geometry_msgs::msg::PoseStamped> rrt(const nav_msgs::msg::OccupancyGrid& grid);

#endif // RRT_HPP