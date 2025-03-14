#ifndef A_STAR_H
#define A_STAR_H


#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <vector>
#include <iostream>


#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>



class NodeAstar {
  public:
    NodeAstar(int x, int y, std::shared_ptr<NodeAstar> parent = nullptr);

    void set_gcost(float g_cost);
    void set_hcost(float h_cost);

    int x, y;
    float g_cost, h_cost, f_cost;

    std::shared_ptr<NodeAstar> parent;
};

struct compare_node {
    bool operator()(const std::shared_ptr<NodeAstar> &a, const std::shared_ptr<NodeAstar> &b) const;
};

float heuristic(int x1, int y1, int x2, int y2);


std::vector<geometry_msgs::msg::PoseStamped> a_star(const nav_msgs::msg::OccupancyGrid& grid);
std::pair<int,int> indexToCoordinate(int index, int width);



#endif // A_STAR_H