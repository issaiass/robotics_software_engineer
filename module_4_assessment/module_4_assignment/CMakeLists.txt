cmake_minimum_required(VERSION 3.8)
project(module_4_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
#find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


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


# https://stackoverflow.com/questions/65935364/looking-for-cv-bridge-example-in-c-for-ros2
add_executable(line_follower src/line_follower_camera/line_follower.cpp src/line_follower_camera/line_follower_camera.cpp )
ament_target_dependencies(line_follower rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(line_follower ${OpenCV_LIBS})

# add the executable and link the libraries
add_executable(maze_solver src/maze_solver/scan_laser_data.cpp src/maze_solver/maze_solver.cpp )
ament_target_dependencies(maze_solver rclcpp geometry_msgs sensor_msgs)
target_link_libraries(maze_solver)


install(TARGETS
  line_follower
  maze_solver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
