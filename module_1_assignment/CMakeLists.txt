cmake_minimum_required(VERSION 3.8)
project(module_1_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# add the executable
add_executable(assignment1 src/task1/assignment1.cpp src/task1/C3PO.cpp src/task1/R2D2.cpp)
add_executable(assignment2 src/task2/assignment2.cpp src/task2/RobotTask2.cpp)
add_executable(assignment3 src/task3/assignment3.cpp src/task3/RobotTask3.cpp)

# link the libraries
target_link_libraries(assignment1)
target_link_libraries(assignment2)
target_link_libraries(assignment3)

ament_target_dependencies(assignment1)
ament_target_dependencies(assignment2)
ament_target_dependencies(assignment3)

install(TARGETS
  assignment1
  assignment2
  assignment3
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
