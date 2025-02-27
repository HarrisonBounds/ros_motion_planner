cmake_minimum_required(VERSION 3.8)
project(motion_planner)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(obstacle_marker_publisher src/obstacle_marker_publisher.cpp)
ament_target_dependencies(obstacle_marker_publisher rclcpp visualization_msgs geometry_msgs)

# Install the executable
install(TARGETS
  obstacle_marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
