cmake_minimum_required(VERSION 3.8)
project(tcam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)


# 실행 파일 빌드
add_executable(mono_camera_publisher src/mono_camera_publisher.cpp)
ament_target_dependencies(mono_camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# 실행 파일 설치
install(TARGETS
  mono_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 설치할 launch 파일을 명시
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
