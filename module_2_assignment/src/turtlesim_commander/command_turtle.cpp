#include "rclcpp/rclcpp.hpp"
#include "turtlesim_commander/turtlesim_commander.h"

using namespace std;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto turtle = std::make_shared<TurtlesimCommander>();
    
    while (!turtle->initial_pose_received && rclcpp::ok()) {
        rclcpp::spin_some(turtle);
    }

    string draw;
    turtle->get_parameter("shape", draw);
    if (draw == "square") {
        turtle->draw_n_point(4, 1.0, 2.0, 0.2, 1);
    } 
    if (draw == "triangle") {
        turtle->draw_n_point(3, 1.0, 2.0, 0.2, 1);
    } 
    if (draw == "star") {
        turtle->draw_5_point_star(1.0, 2.0, 0.2, 1);
    }
    if (draw == "circle") {
        float linear;
        turtle->get_parameter("linear", linear);
        turtle->draw_circle(linear, 0.6);
    }
    if (draw == "spiral") {
        float a, b;
        turtle->get_parameter("a", a);
        turtle->get_parameter("b", b);        
        turtle->draw_log_spiral(a, b);
    }
    rclcpp::spin(turtle);
    rclcpp::shutdown();
    return 0;
}