#ifndef NODE_RRT_HPP
#define NODE_RRT_HPP

#include <memory>

class Node_RRT
{
public:
    Node_RRT();
    Node_RRT(int x, int y);
    Node_RRT(int x, int y, std::shared_ptr<Node_RRT> parent, float cost);

    void setParent(std::shared_ptr<Node_RRT> parent);
    void setCost(float cost);

    auto getParent() -> std::shared_ptr<Node_RRT>;
    auto getCost() const -> float;
    auto getX() const -> int;
    auto getY() const -> int;
    auto operator==(Node_RRT const &node) const -> bool;

    static auto heuristics(Node_RRT const &node1, Node_RRT const &node2) -> float;
    static float heuristicsEuclidean(Node_RRT const &node1, Node_RRT const &node2);

private:
    int x, y;
    std::shared_ptr<Node_RRT> parent = nullptr;
    float cost = 0.0f;
};

#endif // NODE_HPP