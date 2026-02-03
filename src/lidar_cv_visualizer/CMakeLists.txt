cmake_minimum_required(VERSION 3.8)
project(lidar_cv_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 실행 파일 생성
add_executable(cv_visualizer src/lidar_cv_node.cpp)

# 헤더 경로 및 라이브러리 연결
target_include_directories(cv_visualizer PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(cv_visualizer rclcpp sensor_msgs)
target_link_libraries(cv_visualizer ${OpenCV_LIBRARIES})

# 설치 경로 설정
install(TARGETS
  cv_visualizer
  DESTINATION lib/${PROJECT_NAME})

ament_package()