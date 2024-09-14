#include "rclcpp/rclcpp.hpp"
#include "goal_planner/goal_planner.h"

using namespace std;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto goal_planner = std::make_shared<GoalPlanner>();
    rclcpp::spin(goal_planner);
    rclcpp::shutdown();
    return 0;
}