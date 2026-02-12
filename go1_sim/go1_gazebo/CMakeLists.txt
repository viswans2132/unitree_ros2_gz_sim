cmake_minimum_required(VERSION 3.8)
project(go1_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Gazebo Sim (gz Harmonic) integration
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gz_ros2_control REQUIRED)

# ros2_control tooling / broadcasters (used by your launch)
find_package(controller_manager REQUIRED)
find_package(imu_sensor_broadcaster REQUIRED)

# Messages (only needed if this package builds nodes; harmless to keep)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(robot_description_publisher src/robot_description_publisher.cpp)
ament_target_dependencies(robot_description_publisher rclcpp std_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS robot_description_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    worlds
#     models
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
