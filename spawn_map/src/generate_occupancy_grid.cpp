#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <opencv2/opencv.hpp>

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
        

        // Load the PGM file
        std::string pgm_file = "maps/test_map.pgm";
        cv::Mat map_image = cv::imread(pgm_file, cv::IMREAD_GRAYSCALE);

        if (map_image.empty()) {
            std::cerr << "Error: Could not load image file " << pgm_file << std::endl;
        }

        int width = map_image.cols;
        int height = map_image.rows;

        const int grid_size = width*height;

        std::vector<signed char> og_array(grid_size);

        std::cout << "Image loaded successfully!" << std::endl;
        std::cout << "Width: " << width << ", Height: " << height << std::endl;

         // Loop through the grid and assign values
         for (int i = 0; i < grid_size; i++)
         {
             // Calculate the row and column in the grid
             int row = i / width;
             int col = i % width;
 
             // Get the pixel value from the image
             uint8_t pixel_value = map_image.at<uint8_t>(row, col);
 
             // Map the pixel value to the occupancy grid format
             if (pixel_value == 0) {
                 og_array[i] = 100; // Occupied (wall)
             } else if (pixel_value == 255) {
                 og_array[i] = 0; // Free space
             } else {
                 og_array[i] = -1; // Unknown space
             }
         }

        occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
        occupancy_grid_msg.header.frame_id = "map_frame";

        occupancy_grid_msg.info.resolution = 1;

        occupancy_grid_msg.info.width = 12;
        occupancy_grid_msg.info.height = 10;

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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
    rclcpp::shutdown();
    return 0;
}