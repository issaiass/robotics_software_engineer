#include "algo/a_star.h"

NodeAstar::NodeAstar(int x, int y, std::shared_ptr<NodeAstar> parent)
    : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(parent) {}

void NodeAstar::set_gcost(float cost)
{
    g_cost = cost;
    f_cost = g_cost + h_cost;
}

void NodeAstar::set_hcost(float cost)
{
    f_cost = cost;
    f_cost = g_cost + h_cost;
}

bool compare_node::operator()(const std::shared_ptr<NodeAstar> &a, const std::shared_ptr<NodeAstar> &b) const
{
    return a->f_cost > b->f_cost;
}

float heuristic(int x1, int y1, int x2, int y2)
{
    float dx = std::pow(x2 - x1, 2);
    float dy = std::pow(y2 - y1, 2);
    float d = std::sqrt(dx + dy);
    return d;
}

std::vector<geometry_msgs::msg::PoseStamped> a_star(const nav_msgs::msg::OccupancyGrid &grid)
{
    std::cout << "Started A* Planning Algorithm" << std::endl;
    std::vector<geometry_msgs::msg::PoseStamped> path;
    int GRID_HEGHT = grid.info.height;
    int GRID_WIDTH = grid.info.width;

    const auto &grid_data = grid.data;

    int occupied = 0;
    int free = 0;

    for (int i = 0; i < GRID_HEGHT * GRID_WIDTH; ++i)
    {
        if (grid_data[i] == 100)
        {
            occupied++;
        }
        if (grid_data[i] == 0)
        {
            free++;
        }
    }

    std::cout << "Free cells: " << free << " | Occupied cells: " << occupied << std::endl;

    geometry_msgs::msg::Point start_point;
    geometry_msgs::msg::Point goal_point;

    std::pair<int, int> start = indexToCoordinate(0, GRID_WIDTH);
    std::pair<int, int> goal = indexToCoordinate(77, GRID_WIDTH);

    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose;

    start_point.x = static_cast<float>(start.first);
    start_point.y = static_cast<float>(start.second);

    goal_point.x = static_cast<float>(goal.first);
    goal_point.y = static_cast<float>(goal.second);


    std::cout << "Start: " << start_point.x << " " << start_point.y << " | Goal: " << goal_point.x << " " << goal_point.y << std::endl;
    std::cout << "Path size: " << path.size() << std::endl;
    std::cout << "Map size: " << GRID_HEGHT << "x" << GRID_WIDTH << std::endl;
    std::cout << "Finished A* Planning Algorithm" << std::endl;

    std::priority_queue<std::shared_ptr<NodeAstar>, std::vector<std::shared_ptr<NodeAstar>>, compare_node> open_list;
    float MAX = std::numeric_limits<float>::max();
    std::vector<std::vector<float>> cost_so_far(GRID_HEGHT, std::vector<float>(GRID_WIDTH, MAX));
    std::vector<std::vector<bool>> closed_list(GRID_HEGHT, std::vector<bool>(GRID_WIDTH, false));

    auto start_node = std::make_shared<NodeAstar>(static_cast<int>(start_point.x), static_cast<int>(start_point.y));

    start_node->set_gcost(0);
    start_node->set_hcost(heuristic(start_node->x, start_node->y,
                                    static_cast<int>(goal_point.x), static_cast<int>(goal_point.y)));

    open_list.push(start_node);
    cost_so_far[start_point.x][start_point.y] = 0;

    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
    };

    while (!open_list.empty())
    {
        auto current_node = open_list.top();
        open_list.pop();

        std::cout << "Checking note ad {" << current_node->x << ", " << current_node->y << "}" << " against the goal {" << goal_point.x << ", " << goal_point.y << "}" << std::endl;

        if (current_node->x == goal_point.x && current_node->y == goal_point.y)
        {
            std::vector<geometry_msgs::msg::PoseStamped> path_reversed;

            auto path_node = current_node;
            int node_count = 0;

            while (path_node != nullptr)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = grid.header.frame_id;
                pose.header.stamp = grid.header.stamp;
                pose.pose.position.x = path_node->x - 5.0;
                pose.pose.position.y = path_node->y - 5.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;

                path_reversed.push_back(pose);
                path_node = path_node->parent;
                node_count++;
            }

            std::cout << "Found goal path. Total numer of nodes: " << node_count << std::endl;

            for (int i = path_reversed.size() - 1; i >= 0; --i)
            {
                path.push_back(path_reversed[i]);
            }
            return path;
            break;
        }

        closed_list[current_node->x][current_node->y] = true;

        // Explore neighbors
        for (auto dir: directions) {
            int new_x = current_node->x + dir.first;
            int new_y = current_node->y + dir.second;

            std::cout << "Considering neighbor at {" << new_x << ", " << new_y << "}" << std::endl;

            // Check if new node is within the grid and not an obstacle
            if (new_x >= 0 && new_x < GRID_WIDTH && new_y >= 0 && new_y < GRID_HEGHT) {
                int new_index = new_y * GRID_WIDTH + new_x;
        
                // Check if the cell is an obstacle
                if (grid_data[new_index] == 100) { 
                    std::cout << "Obstacle found at {" << new_x << ", " << new_y << "}" << std::endl;
                  continue;
                }

                float new_cost = current_node->g_cost + (dir.first == 0 || dir.second == 0 ? 1 : std::sqrt(2));

                if (!closed_list[new_x][new_y])
                {
                    auto neighbor = std::make_shared<NodeAstar>(new_x, new_y, current_node);

                    if (new_cost < cost_so_far[new_x][new_y])
                    {
                        cost_so_far[new_x][new_y] = new_cost;
                        neighbor->set_gcost(new_cost);
                        neighbor->set_hcost(heuristic(new_x, new_y, goal_point.x, goal_point.y));
                        open_list.push(neighbor);
                    }
                }
            }
        }

    }

    return path;
}

std::pair<int, int> indexToCoordinate(int index, int width)
{
    if (index < 0 || index >= width * width)
    {
        std::cout << "Index out of bounds" << std::endl;
        return std::make_pair(-1, -1);
    }
    int x = index % width;
    int y = index / width;
    return std::make_pair(x, y);
}