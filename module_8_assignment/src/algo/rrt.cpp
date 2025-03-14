#include "algo/rrt.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

RRT_Planner::RRT_Planner() {}

void RRT_Planner::setDomain(std::vector<int> &domain)
{
    std::fill(DOMAIN.begin(), DOMAIN.end(), 0);
    std::copy(domain.begin(), domain.end(), DOMAIN.begin());
}

Node_RRT RRT_Planner::generateRandomNode(std::mt19937 &gen)
{
    std::uniform_int_distribution<int> distribution(0, GRID_WIDTH * GRID_HEIGHT - 1);
    int random_index = distribution(gen);

    return indexToCoordinate(random_index);
}

Node_RRT RRT_Planner::findNearestNode(std::vector<Node_RRT> const &nodes, Node_RRT const &randomNode)
{
    Node_RRT nearestNode = nodes[0];
    double minDistance = std::numeric_limits<double>::max();

    for (auto const& node : nodes) {
        double distance = Node_RRT::heuristics(node, randomNode);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNode = node;
        }
    }

    return nearestNode;
}

Node_RRT RRT_Planner::findNewConfig(Node_RRT const &nearestNode, Node_RRT const &randomNode)
{
    int dx = randomNode.getX() - nearestNode.getX();
    int dy = randomNode.getY() - nearestNode.getY();

    float distance = Node_RRT::heuristicsEuclidean(nearestNode, randomNode);

    float scaledDx = (distance > 0) ? (dx * STEP_SIZE) / distance : dx;
    float scaledDy = (distance > 0) ? (dy * STEP_SIZE) / distance : dy;

    int x = nearestNode.getX() + static_cast<int>(scaledDx);
    int y = nearestNode.getY() + static_cast<int>(scaledDy);

    return Node_RRT(x, y);
}

bool RRT_Planner::isObstacle(Node_RRT &nearestNode, Node_RRT const &newNode)
{
    if (newNode.getX() < 0 || newNode.getX() >= GRID_WIDTH || newNode.getY() < 0 || newNode.getY() >= GRID_HEIGHT) {
        return true;
    }

    if (DOMAIN[newNode.getY() * GRID_WIDTH + newNode.getX()] == 100) {
        return true;
    }

    int dx = newNode.getX() - nearestNode.getX();
    int dy = newNode.getY() - nearestNode.getY();
    int steps = std::max(std::abs(dx), std::abs(dy));

    for (int i = 0; i < steps; i++) {
        float x = nearestNode.getX() + (i * dx) / steps;
        float y = nearestNode.getY() + (i * dy) / steps;

        if (DOMAIN[static_cast<int>(y) * GRID_WIDTH + static_cast<int>(x)] == 100) {
            return true;
        }
    }

    return false;
}

bool RRT_Planner::isGoalFound(Node_RRT &newNode, Node_RRT const &goal)
{
    return newNode == goal;
}

Node_RRT RRT_Planner::indexToCoordinate(int index)
{
    int x = index % GRID_WIDTH;
    int y = index / GRID_WIDTH;

    return Node_RRT(x, y);
}

std::vector<Node_RRT> RRT_Planner::planPath(Node_RRT const &start, Node_RRT const &goal)
{
    nodes.push_back(start);
    std::mt19937 gen(SEED);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        Node_RRT randomNode = generateRandomNode(gen);
        Node_RRT nearestNode = findNearestNode(nodes, randomNode);
        if (nearestNode == randomNode) {
            continue;
        }

        Node_RRT newNode = findNewConfig(nearestNode, randomNode);

        if (isObstacle(nearestNode, newNode)) {
            continue;
        }

        newNode.setParent(std::make_shared<Node_RRT>(nearestNode));
        nodes.push_back(newNode);

        if (isGoalFound(newNode, goal)) {
            path.push_back(newNode);
            auto parent = *newNode.getParent();

            while (&parent != &start) {
                path.push_back(parent);
                if (parent.getParent() == nullptr) {
                    break;
                }

                parent = *parent.getParent();
            }
            std::reverse(path.begin(), path.end());
            break;
        }
    }

    return path;
}