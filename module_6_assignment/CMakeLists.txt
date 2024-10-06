cmake_minimum_required(VERSION 3.8)
project(module_6_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# add Eigen
set(EIGEN_INCLUDE_DIR $(CMAKE_SOURCE_DIR)/../..extern/eigen)
find_package(Eigen3 REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)


include_directories(include)

# Extended Kalman Filter
add_library(extended_kalman_filter_lib STATIC src/extended_kalman_filter/extended_kalman_filter.cpp)
target_include_directories(extended_kalman_filter_lib PRIVATE ${EIGEN3_INCLUDE_DIR} include)
target_link_libraries(extended_kalman_filter_lib Eigen3::Eigen)


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

# Extended Kalman Filter
add_executable(ekf_node src/extended_kalman_filter/ekf_node.cpp)
target_include_directories(ekf_node PRIVATE include)
target_link_libraries(ekf_node extended_kalman_filter_lib)
ament_target_dependencies(ekf_node rclcpp sensor_msgs nav_msgs visualization_msgs geometry_msgs tf2)


install(TARGETS 
  extended_kalman_filter_lib
  ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

# add folders to share
install(DIRECTORY
  launch
  rviz
  config
  models
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
