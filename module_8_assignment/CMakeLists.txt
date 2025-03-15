cmake_minimum_required(VERSION 3.8)
project(module_8_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)


# include directories
include_directories(include)


# Node Library
add_library(node_lib src/node/node_rrt.cpp)

# A* Library
add_library(a_star STATIC src/algo/a_star.cpp)
ament_target_dependencies(a_star rclcpp geometry_msgs nav_msgs)

# RRT Library
add_library(rrt STATIC src/algo/rrt.cpp)
ament_target_dependencies(rrt rclcpp nav_msgs geometry_msgs)
target_link_libraries(rrt node_lib)

# RRTA* Library
add_library(rrt_a_star STATIC src/algo/rrt_a_star.cpp)
ament_target_dependencies(rrt_a_star rclcpp nav_msgs geometry_msgs)
target_link_libraries(rrt_a_star node_lib a_star rrt)

# Occupancy Grid Node
add_executable(occupancy_grid_node src/occupancy_grid/occupancy_grid.cpp src/occupancy_grid_node.cpp)
ament_target_dependencies(occupancy_grid_node rclcpp nav_msgs geometry_msgs)

# Path Planner Node
add_executable(path_planning_node src/path_planning/path_planning.cpp src/path_planning_node.cpp)
ament_target_dependencies(path_planning_node rclcpp nav_msgs geometry_msgs)
target_link_libraries(path_planning_node node_lib a_star rrt rrt_a_star)

install(TARGETS 
  occupancy_grid_node
  path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
