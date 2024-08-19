#include "rclcpp/rclcpp.hpp"
#include "line_follower_camera/line_follower_camera.h"

using namespace std;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto line_follower = std::make_shared<LineFollowerCamera>();
    rclcpp::spin(line_follower);
    rclcpp::shutdown();
    return 0;
}