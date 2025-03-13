/**
 * @file generate_custom_occupnacy_grid.cpp
 * @brief Publishes a predefined occupancy grid map for navigation
 * @author Harrison Bounds
 * @date 2025-03-13
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

/**
 * @class OccupancyGrid_Publisher
 * @brief ROS2 node for publishing a predefined occupancy grid map
 *
 * This class creates and periodically publishes a static occupancy grid map
 * on the "custom_occupancy_grid" topic. The map is defined as a hardcoded
 * 25x20 grid representing a simple environment with walls (0) and free space (100).
 */
class OccupancyGrid_Publisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for OccupancyGrid_Publisher
   *
   * Initializes the node, creates a publisher for the occupancy grid map,
   * and sets up a timer to publish the map periodically.
   */
  OccupancyGrid_Publisher()
      : Node("occupancy_grid_publisher")
  {
    og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid", 10);
    og_timer = this->create_wall_timer(500ms, std::bind(&OccupancyGrid_Publisher::og_callback, this));
  }

private:
  /**
   * @brief Callback function to create and publish the occupancy grid
   *
   * This method:
   * 1. Creates a new occupancy grid message
   * 2. Defines a hardcoded 25x20 grid map with walls (0) and free space (100)
   * 3. Sets the necessary metadata (resolution, dimensions, origin)
   * 4. Publishes the map on the "custom_occupancy_grid" topic
   *
   * The map is published every 500ms as defined by the timer.
   */
  void og_callback()
  {
    auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
    std::vector<signed char> og_array(500);
    int grid[500] = {
        // Row 1-5 (indices 0-99)
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 100, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 100, 0, 0, 0, 100, 0, 0, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0,
        0, 100, 0, 0, 0, 100, 0, 0, 100, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0,
        // Row 6-10 (indices 100-199)
        0, 100, 100, 0, 100, 100, 0, 0, 100, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        // Row 11-15 (indices 200-299)
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
        // Row 16-20 (indices 300-399)
        0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        // Row 21-25 (indices 400-499)
        100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 100, 0,
        100, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
        100, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
        100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 100, 0,
        100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Convert int array to signed char vector required by OccupancyGrid message
    for (int i = 0; i < 500; i++)
    {
      og_array[i] = static_cast<signed char>(grid[i]);
    }

    // Set occupancy grid metadata
    occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
    occupancy_grid_msg.header.frame_id = "map";
    occupancy_grid_msg.info.resolution = 1; // 1 meter per cell
    occupancy_grid_msg.info.width = 25;     // 25 cells wide
    occupancy_grid_msg.info.height = 20;    // 20 cells tall
    occupancy_grid_msg.info.origin.position.x = 0.0;
    occupancy_grid_msg.info.origin.position.y = 0.0;
    occupancy_grid_msg.info.origin.position.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;

    // Assign data and publish
    occupancy_grid_msg.data = og_array;
    og_pub->publish(occupancy_grid_msg);
  }

  /** @brief Timer for periodic map publishing */
  rclcpp::TimerBase::SharedPtr og_timer;

  /** @brief Publisher for the occupancy grid map */
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;
};

/**
 * @brief Main function to initialize and run the OccupancyGrid_Publisher node
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit status
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}