cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera src/sub.cpp)
target_include_directories(camera PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera ${OpenCV_LIBRARIES})

install(TARGETS
  camera
  DESTINATION lib/${PROJECT_NAME})

ament_package()