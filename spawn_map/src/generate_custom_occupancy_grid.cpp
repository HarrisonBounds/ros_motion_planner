#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;
class OccupancyGrid_Publisher : public rclcpp::Node
{
  public:
    OccupancyGrid_Publisher()
    : Node("occupancy_grid_publisher")
    {
      og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid", 10);
      og_timer = this->create_wall_timer(500ms, std::bind(&OccupancyGrid_Publisher::og_callback, this));
    }
  private:

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
        100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
      };

      for (int i = 0; i < 500; i++) {
        og_array[i] = static_cast<signed char>(grid[i]);
      }

      occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
      occupancy_grid_msg.header.frame_id = "map";

      occupancy_grid_msg.info.resolution = 1;

      occupancy_grid_msg.info.width = 25;
      occupancy_grid_msg.info.height = 20;

      occupancy_grid_msg.info.origin.position.x = 0.0;
      occupancy_grid_msg.info.origin.position.y = 0.0;
      occupancy_grid_msg.info.origin.position.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.x = 0.0;
      occupancy_grid_msg.info.origin.orientation.y = 0.0;
      occupancy_grid_msg.info.origin.orientation.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.w = 1.0;
      occupancy_grid_msg.data = og_array;

      og_pub->publish(occupancy_grid_msg);
    }


    rclcpp::TimerBase::SharedPtr og_timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}