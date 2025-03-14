#include <node/node_rrt.hpp>
#include <cmath>
#include <memory>
#include <utility>


Node_RRT::Node_RRT(int x, int y) : x(x), y(y) {}


Node_RRT::Node_RRT(int x, int y, std::shared_ptr<Node_RRT> parent, float cost)
    : x(x), y(y), parent(std::move(parent)), cost(cost) {}

    

void Node_RRT::setCost(float cost)
{
    this->cost = cost;
}

void Node_RRT::setParent(std::shared_ptr<Node_RRT> parent)
{
    this->parent = std::move(parent);
}

auto Node_RRT::getParent() -> std::shared_ptr<Node_RRT>
{
    return parent;
}

auto Node_RRT::getCost() const -> float
{
    return cost;
}

auto Node_RRT::getX() const -> int
{
    return x;
}

auto Node_RRT::getY() const -> int
{
    return y;
}

auto Node_RRT::operator==(Node_RRT const& node) const -> bool
{
    return x == node.x && y == node.y;
}

auto Node_RRT::heuristics(Node_RRT const& node1, Node_RRT const& node2) -> float
{
    float dx = static_cast<float>(node1.getX() - node2.getX());
    float dy = static_cast<float>(node1.getY() - node2.getY());

    return std::abs(dx) + std::abs(dy);
}

float Node_RRT::heuristicsEuclidean(Node_RRT const& node1, Node_RRT const& node2)
{
    float dx = static_cast<float>(node1.getX() - node2.getX());
    float dy = static_cast<float>(node1.getY() - node2.getY());
    
    float dx2 = std::pow(dx, 2);
    float dy2 = std::pow(dy, 2);

    return std::sqrt(dx2 + dy2);
}