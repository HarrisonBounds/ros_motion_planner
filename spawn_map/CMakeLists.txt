cmake_minimum_required(VERSION 3.8)
project(spawn_map)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add the executable
# add_executable(display_map src/display_map.cpp)

# Link dependencies
# ament_target_dependencies(display_map
#   rclcpp
#   nav_msgs
# )

# Install the executable
# install(TARGETS
#   display_map
#   DESTINATION lib/${PROJECT_NAME}
# )

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