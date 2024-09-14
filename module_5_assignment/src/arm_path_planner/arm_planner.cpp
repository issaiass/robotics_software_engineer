#include "arm_path_planner/arm_path_planner.h"

using namespace std;

// base_joint (revolute) -pi, pi
// finger_joint (continuous) -pi/4, pi/4
// ee_joint (revolute) -pi/2, pi/2
// gripper_extension_joint prismatic (0, 0.5)
// gripper_tip_ee_right_joint  (-0.08, 0.785)
// gripper_tip_ee_center_joint (-0.08, 0.785)
// gripper_tip_ee_left_joint   (-0.08, 0.785)

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto arm_planner = std::make_shared<ArmPathPlanner>();
    rclcpp::spin(arm_planner);
    rclcpp::shutdown();
    return 0;
}