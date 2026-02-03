cmake_minimum_required(VERSION 3.8)
project(twist_to_carla_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

# Add executable
add_executable(twist_to_carla_control_node src/twist_to_carla_control.cpp)

ament_target_dependencies(twist_to_carla_control_node
  rclcpp
  geometry_msgs
  carla_msgs
)

# Install targets
install(TARGETS
  twist_to_carla_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
