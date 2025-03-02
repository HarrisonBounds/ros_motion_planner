cmake_minimum_required(VERSION 3.8)
project(spawn_map)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(generate_occupancy_grid src/generate_occupancy_grid.cpp)
add_executable(generate_custom_occupancy_grid src/generate_custom_occupancy_grid.cpp)

#Link dependencies
ament_target_dependencies(generate_occupancy_grid
  rclcpp
  nav_msgs
  OpenCV
)

ament_target_dependencies(generate_custom_occupancy_grid
  rclcpp
  nav_msgs
)

#Install the executable
install(TARGETS
  generate_occupancy_grid
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  generate_custom_occupancy_grid
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()