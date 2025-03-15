#ifndef RRT_ASTAR_HPP
#define RRT_ASTAR_HPP

#include "algo/a_star.h"
#include "algo/rrt.hpp"
#include <vector>

class RRT_AStar : public AStar, public RRT_Planner  // Multiple Inheritance
{
  public:
    RRT_AStar();
    std::vector<Node_RRT> planPath(const Node_RRT &start, const Node_RRT &goal); // seek the path
    std::vector<geometry_msgs::msg::PoseStamped> refinePathWithAStar(const nav_msgs::msg::OccupancyGrid &grid); // refine the path
    void setDomain(const std::vector<int> &domain); // set domain
};

#endif // RRT_ASTAR_HPP
