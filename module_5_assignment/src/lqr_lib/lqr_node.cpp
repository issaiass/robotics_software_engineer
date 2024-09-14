
#include <angles/angles.h>
#include <cmath>
#include <lqr_node/lqr_node.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>

input input_old = input(0, 0);

LqrNode::LqrNode()
    : Node("LqrNode"), dt_(0.2), tolerance(0.2), end_controller(false),
      max_linear_velocity(0.8), max_angular_velocity(M_PI / 2),
      current_waypoint(0), odom_received_(false) {

  robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
      std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));

  control_input_pub_ =this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                        std::bind(&LqrNode::controlLoopCallback, this));
  _publisher_path_marker = this->create_publisher<visualization_msgs::msg::Marker>("/path_marker", 100);
  _publisher_goals_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("/goals_marker", 20);


  waitForMarkerSubscription(_publisher_path_marker);

  _timer_path = this->create_wall_timer(std::chrono::milliseconds(int(1000*dt_)),
                        std::bind(&LqrNode::timer_path_callback, this));

// BEST VALUES
// q_diag = 0.85, r_diag = 0.3, horizon = 100,  tol = 0.2, dt_ = 0.2


// HIGH Q, LOW R FINISHED AND GET THE GOALS VERY FAST
// HIGH R, LOW Q ONLY STAYS ON GOAL 3
// HIGH R, HIGH Q ACHIEVES ALL GOALS VERY CONSERVATIVE
// LOW R, LOW Q ONLY STAYS ON GOAL 1
// MID Q, MID R A WELL BALANCE BETWEEN GOAL AND RESPONSE INPUT

  const float q_diag = 0.85;
  const float r_diag = 0.25; 
  const uint8_t horizon_ = 100;
  Q_ << q_diag, 0, 0, 0, q_diag, 0, 0, 0, 0.1;
  R_ << r_diag, 0, 0, 0.1;

  lqr_ = std::make_unique<LQR>(Q_, R_, horizon_);

  waypoints_ = {State(1, 1, M_PI / 4), State(2, 2, M_PI / 2),
                State(3, 3, M_PI),     State(0, 2.5, M_PI / 2),
                State(-1, 4, M_PI),    State(-2, 3, -M_PI / 2),
                State(-3, 2, M_PI),    State(-3, 1, M_PI / 2),
                State(0, 0, 0)};
  actual_state_ = State(0, 0, 0);
  publishGoals(waypoints_);
  optimiseHeading(waypoints_);

  _logFile.open("log.csv", ios::out | ios::app); // Open in append mode

  _logFile << "q_diag" << "," << "r_diag" << ","  << "horizon"            << "," << "tolerance" << "," << "dt_" << std::endl;  
  _logFile << q_diag   << "," << r_diag   << ","  << to_string(horizon_)  << "," << tolerance   << "," <<  dt_  << std::endl;  
  _logFile << "X_Goal" << "," << "Y_Goal" << "," << "X_Pos" << "," << "Y_Pos" << "," << "Theta_Goal" << "," << "Theta" << std::endl;  

}

void LqrNode::waitForMarkerSubscription(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker) {
  while (marker->get_subscription_count() < 1)
  {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Please create a subscriber to the marker and marker array...");
    rclcpp::sleep_for(1s);
  }
}

void LqrNode::publishGoals(const std::vector<State> waypoints) {
  uint8_t index = 0;
  visualization_msgs::msg::MarkerArray text_array;
  visualization_msgs::msg::MarkerArray shape_array;  
  for (State wp : waypoints) {
    geometry_msgs::msg::Point p;
    p.x = wp.x;
    p.y = wp.y;
    p.z = 0.3;

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "odom";
    text_marker.header.stamp = this->now();
    text_marker.ns = "/goal_labels";
    text_marker.id = index;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position = p;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "Goal" + std::to_string(++index);
    text_array.markers.push_back(text_marker);

    visualization_msgs::msg::Marker shape_marker;    
    shape_marker = text_marker;
    shape_marker.ns = "/goal_shape";    
    shape_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    shape_marker.pose.position.z = 0.0;
    shape_marker.scale.x = 0.5;
    shape_marker.scale.y = 0.5;
    shape_marker.scale.z = 0.03;

    shape_marker.color.r = 0.1*(index+1);
    shape_marker.color.g = 0.45;
    shape_marker.color.b = 1.0*(waypoints.size()-index);
    shape_marker.color.a = 1.0;

    shape_array.markers.push_back(shape_marker);
  }
  _publisher_goals_marker->publish(text_array);
  _publisher_goals_marker->publish(shape_array);  
}


void LqrNode::timer_path_callback() {
    if (!odom_received_) {
      RCLCPP_INFO(this->get_logger(), "timer path callback didn't received first odometry msg...");
      return;
    }

    line_strip.header.frame_id = "odom";
    line_strip.header.stamp = this->now();;
    line_strip.ns = "/path";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.id = 1;
    
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01;
    line_strip.scale.y = 0.01;
    line_strip.scale.z = 0.01;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.g = 165/255.0;   
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    // Set the marker lifetime.
    line_strip.lifetime = rclcpp::Duration(5, 0);

    // Create the vertices for the points and lines
    geometry_msgs::msg::Point p;
    p = _tb3->pose.pose.position;
    line_strip.points.push_back(p);
    _recent_points.push_back(p);

    if (_recent_points.size() > 50) {
        _recent_points.erase(_recent_points.begin());
    }

    line_strip.points.clear();
    for (const auto& point : _recent_points) {
        line_strip.points.push_back(point);
    }
    _publisher_path_marker->publish(line_strip);

  _logFile << desired_state_.x << "," << desired_state_.y << "," << actual_state_.x << "," << actual_state_.y << "," << desired_state_.theta << "," << actual_state_.theta << std::endl;
}


void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
 tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  actual_state_ =
      State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  odom_received_ = true;
 _tb3 = msg;
}

void LqrNode::publishVelocity(double v, double w) {

  geometry_msgs::msg::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Publishing control input: v=%f, w=%f",
              v, w);
  control_input_ = input(v, w);
  input_old = input(v, w);
  control_input_pub_->publish(msg);
}

void LqrNode::optimiseHeading(std::vector<State> &waypoints) {

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    double dx = waypoints[i + 1].x - waypoints[i].x;
    double dy = waypoints[i + 1].y - waypoints[i].y;
    waypoints[i].theta = std::atan2(dy, dx);
  }
  waypoints.back().theta = waypoints[waypoints.size() - 2].theta;

}

void LqrNode::controlLoopCallback() {

 if (!odom_received_) {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waiting for odometry message...");
    return;
  }
  if (end_controller) {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Goal reached!");
    control_loop_timer_->cancel();
    return;
  }

  desired_state_ = waypoints_[current_waypoint];

  Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y,
                           actual_state_.theta);
  Eigen::Vector3d x_desired(desired_state_.x, desired_state_.y,
                            desired_state_.theta);
  state_error_ = x_actual - x_desired;

  // if (current_waypoint == 2) {
  //   waypoints_[current_waypoint + 1] = State(-1, 3, M_PI);
  // }

  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current Waypoint:=%d ",current_waypoint);
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Actual state: x=%f, y=%f, theta=%f",x_actual(0), x_actual(1), x_actual(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Desired state: x=%f, y=%f, theta=%f",x_desired(0), x_desired(1), x_desired(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "State error: x=%f, y=%f, theta=%f",state_error_(0), state_error_(1), state_error_(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current goal: x=%f, y=%f, theta=%f",
                waypoints_[current_waypoint].x, waypoints_[current_waypoint].y,waypoints_[current_waypoint].theta);

  auto A = lqr_->getA(actual_state_.theta, control_input_.v, dt_);
  auto B = lqr_->getB(actual_state_.theta, dt_);
  lqr_->updateMatrices(A, B);
  lqr_->computeRiccati(B, A);

  auto u = lqr_->computeOptimalInput(state_error_);

  Eigen::EigenSolver<Eigen::MatrixXd> solver(B * lqr_->K_ + A);
  auto eigenValues = solver.eigenvalues().real();
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Eigenvalues: %f, %f, %f",
              eigenValues(0), eigenValues(1), eigenValues(2));

  publishVelocity(
      std::clamp(u(0), -max_linear_velocity, max_linear_velocity),
      std::clamp(u(1), -max_angular_velocity, max_angular_velocity));

  if (state_error_.norm() < tolerance) {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waypoint reached!");
    current_waypoint++;
    if (static_cast<long unsigned int>(current_waypoint) >= waypoints_.size()) {
      end_controller = true;
      publishVelocity(0.0, 0.0);
    }
  }

}

LqrNode::~LqrNode()
{
    _logFile.close();
    RCLCPP_INFO(this->get_logger(), "Bye! Bye! Bye!");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LqrNode>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
