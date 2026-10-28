cmake_minimum_required(VERSION 3.8)
project(lzh_sdk_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
add_executable(
  joint_state_test_node
  src/joint_state_test_node.cpp
)
add_executable(
  lowstate_bridge_node
  src/lowstate_bridge_node.cpp
)
add_executable(
  lzh_trajectory_action_server
  src/lzh_trajectory_action_server.cpp
)

ament_target_dependencies(
  lzh_trajectory_action_server
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  sensor_msgs
)

ament_target_dependencies(
  lowstate_bridge_node
  rclcpp
  sensor_msgs
)
target_link_libraries(
  lzh_trajectory_action_server
  unitree_sdk2
)

target_link_libraries(
  lowstate_bridge_node
  unitree_sdk2
)

ament_target_dependencies(
  joint_state_test_node
  rclcpp
  sensor_msgs
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
install(
  TARGETS
    joint_state_test_node
    lowstate_bridge_node
    lzh_trajectory_action_server
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
