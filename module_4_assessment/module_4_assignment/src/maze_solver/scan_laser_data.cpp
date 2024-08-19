#include "rclcpp/rclcpp.hpp"
#include "maze_solver/maze_solver.h"

using namespace std;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto maze_solver = std::make_shared<MazeSolver>();
    rclcpp::spin(maze_solver);
    rclcpp::shutdown();
    return 0;
}