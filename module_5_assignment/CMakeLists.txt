cmake_minimum_required(VERSION 3.8)
project(module_5_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EIGEN_INCLUDE_DIR $(CMAKE_SOURCE_DIR)/../..extern/eigen)
find_package(Eigen3 REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)



# LQR Library
add_library(lqr_lib STATIC src/lqr_lib/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

# PID library
add_library(pid_lib STATIC src/pid_lib/pid_lib.cpp)
target_link_libraries(pid_lib)
ament_target_dependencies(pid_lib rclcpp)
target_include_directories(pid_lib PRIVATE include)

# WAYPOINT library
add_library(waypoints_lib STATIC src/waypoints_lib/waypoints_lib.cpp)
ament_target_dependencies(waypoints_lib yaml-cpp)
target_include_directories(waypoints_lib PRIVATE include)

include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# task 1 - line follower with camera 
add_executable(line_follower_pid src/line_follower_camera/line_follower.cpp src/line_follower_camera/line_follower_camera.cpp )
ament_target_dependencies(line_follower_pid rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(line_follower_pid ${OpenCV_LIBS} pid_lib)

# task 2 - goal planner (low consumption path)
add_executable(goal_planner_node src/goal_planner/goal_planner_node.cpp src/goal_planner/goal_planner.cpp)
ament_target_dependencies(goal_planner_node rclcpp geometry_msgs nav_msgs tf2 sensor_msgs tf2_geometry_msgs)
target_link_libraries(goal_planner_node pid_lib yaml-cpp waypoints_lib)

# task 3 - arm path planner
add_executable(arm_path_planner src/arm_path_planner/arm_path_planner.cpp src/arm_path_planner/arm_planner.cpp)
target_link_libraries(arm_path_planner Eigen3::Eigen yaml-cpp)
ament_target_dependencies(arm_path_planner rclcpp sensor_msgs 
trajectory_msgs visualization_msgs geometry_msgs tf2 tf2_ros)

# task 4 - lqr controller
add_executable(lqr_node src/lqr_lib/lqr_node.cpp)
target_link_libraries(lqr_node lqr_lib Eigen3::Eigen)
ament_target_dependencies(lqr_node rclcpp nav_msgs geometry_msgs tf2 angles visualization_msgs tf2_ros)
target_include_directories(lqr_node PRIVATE ${EIGEN_INCLUDE_DIR} include)


# add the executable to lib
install(TARGETS
  pid_lib
  lqr_lib
  lqr_node
  line_follower_pid
  waypoints_lib
  goal_planner_node
  arm_path_planner
  DESTINATION lib/${PROJECT_NAME}
)

# add folders to share
install(DIRECTORY
  config
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
