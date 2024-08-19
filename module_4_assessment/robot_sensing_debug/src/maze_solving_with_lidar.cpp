#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <limits>

enum class RobotState {
  MOVING_STRAIGHT,
  TURNING_LEFT,
  TURNING_RIGHT,
  OUT_OF_MAZE
};

class MazeSolving : public rclcpp::Node {
public:
  MazeSolving() : Node("maze_solver"), state_(RobotState::MOVING_STRAIGHT) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&MazeSolving::lidarCallback, this, std::placeholders::_1));
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidarMsg) {
    float max_ = std::numeric_limits<float>::max();
    static bool status_ = false;

    float rightObstacle = *std::max_element(lidarMsg->ranges.begin() + 260,
                                            lidarMsg->ranges.begin() + 280);
    float frontObstacle = *std::max_element(lidarMsg->ranges.begin() + 340,
                                            lidarMsg->ranges.begin() + 360);
    float leftObstacle = *std::max_element(lidarMsg->ranges.begin() + 80,
                                           lidarMsg->ranges.begin() + 100);

    RCLCPP_INFO(this->get_logger(), "Left: %.3f, Front: %.3f, Right: %.3f",
                leftObstacle, frontObstacle, rightObstacle);

    switch (state_) {
    case RobotState::MOVING_STRAIGHT:
      if (frontObstacle < frontThreshold_) {
        if (leftObstacle < rightObstacle) {
          state_ = RobotState::TURNING_RIGHT;
        } 
        if (rightObstacle < leftObstacle) {
          state_ = RobotState::TURNING_LEFT;
        }
      }
      break;
    case RobotState::TURNING_LEFT:
      if (frontObstacle > frontThreshold_) {
        state_ = RobotState::MOVING_STRAIGHT;
        status_ = true;
      }
      break;
    case RobotState::TURNING_RIGHT:
      if (frontObstacle > frontThreshold_) {
        state_ = RobotState::MOVING_STRAIGHT;
        status_ = true;
      }
      break;
    case RobotState::OUT_OF_MAZE:
        ;
      break;
    }

    if (frontObstacle > max_ && leftObstacle >  max_  && rightObstacle > max_ && status_) {
      state_ = RobotState::OUT_OF_MAZE;
    }

    geometry_msgs::msg::Twist command;
    switch (state_) {
    case RobotState::MOVING_STRAIGHT:
      command.linear.x = linearVel_;
      break;
    case RobotState::TURNING_LEFT:
      command.linear.x = 0.0;
      command.angular.z = angularVel_;
      break;
    case RobotState::TURNING_RIGHT:
      command.linear.x = 0.0;
      command.angular.z = -angularVel_;
      break;
    case RobotState::OUT_OF_MAZE:
      command.linear.x = 0.0;
      command.angular.z = 0.0;
      break;
    }

    publisher_->publish(command);
  }

  float frontThreshold_ = 1.5f;
  float angularVel_ = 1.3f;
  float linearVel_  = 0.5f;
  RobotState state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeSolving>());
  rclcpp::shutdown();
  return 0;
}

