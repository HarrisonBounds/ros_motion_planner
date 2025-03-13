/**
 * @file generate_occupancy_grid.cpp
 * @brief Publishes an occupancy grid map from a PNG image for navigation
 * @author Harrison Bounds
 * @date 2025-03-13
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

/**
 * @class MapPublisher
 * @brief ROS2 node for publishing occupancy grid maps from PNG images
 *
 * This class loads a PNG image, converts it to an occupancy grid map,
 * and periodically publishes it on the "/custom_occupancy_grid" topic.
 * The image is processed to create a map suitable for robot navigation.
 */
class MapPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for MapPublisher
     *
     * Initializes the node, creates a publisher for the occupancy grid map,
     * loads the map from a PNG file, and sets up a timer to publish the map periodically.
     */
    MapPublisher()
        : Node("map_publisher")
    {
        // Publisher for the occupancy grid
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/custom_occupancy_grid", 10);

        // Load the map once
        load_map();

        // Create a timer to publish the map periodically
        timer_ = this->create_wall_timer(500ms, std::bind(&MapPublisher::publish_map, this));
    }

private:
    /**
     * @brief Loads a PNG image and converts it to an occupancy grid map
     *
     * This method:
     * 1. Loads a PNG image from the specified file path
     * 2. Downsamples the image to reduce resolution
     * 3. Converts the grayscale image to an occupancy grid representation
     * 4. Classifies pixels as free, occupied, or unknown based on thresholds
     *
     * @note The map is stored in the occupancy_grid_msg_ member variable
     */
    void load_map()
    {
        // Path to the PNG map file (update this path as needed)
        std::string image_file_path = "/home/harrison-bounds/ws/motionplanner/src/ros_motion_planner/spawn_map/maps/maze_map.png";

        // Hardcoded metadata
        double resolution = 0.02;                     // Increase resolution (reduce precision) to make the map smaller
        std::vector<double> origin = {0.0, 0.0, 0.0}; // x, y, z
        double free_thresh = 0.2;                     // Adjust thresholds to better classify free/occupied space
        double occupied_thresh = 0.6;
        bool negate = false;

        // Load the PNG image
        map_image_ = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", image_file_path.c_str());
            return;
        }

        // Downsample the map (reduce resolution)
        cv::Mat resized_map;
        cv::resize(map_image_, resized_map, cv::Size(), 0.5, 0.5, cv::INTER_AREA); // Reduce size by 50%

        // Initialize the OccupancyGrid message
        occupancy_grid_msg_.header.frame_id = "map";
        occupancy_grid_msg_.info.resolution = resolution;
        occupancy_grid_msg_.info.width = resized_map.cols;
        occupancy_grid_msg_.info.height = resized_map.rows;
        occupancy_grid_msg_.info.origin.position.x = origin[0];
        occupancy_grid_msg_.info.origin.position.y = origin[1];
        occupancy_grid_msg_.info.origin.position.z = origin[2];
        occupancy_grid_msg_.info.origin.orientation.x = 0.0;
        occupancy_grid_msg_.info.origin.orientation.y = 0.0;
        occupancy_grid_msg_.info.origin.orientation.z = 0.0;
        occupancy_grid_msg_.info.origin.orientation.w = 1.0;

        // Convert the image data to the occupancy grid data
        occupancy_grid_msg_.data.resize(resized_map.cols * resized_map.rows);
        for (int y = 0; y < resized_map.rows; y++)
        {
            for (int x = 0; x < resized_map.cols; x++)
            {
                int pixel_value = resized_map.at<uchar>(y, x);
                if (negate)
                {
                    pixel_value = 255 - pixel_value;
                }
                if (pixel_value < free_thresh * 255)
                {
                    occupancy_grid_msg_.data[y * resized_map.cols + x] = 100; // Free
                }
                else if (pixel_value > occupied_thresh * 255)
                {
                    occupancy_grid_msg_.data[y * resized_map.cols + x] = 0; // Occupied
                }
                else
                {
                    occupancy_grid_msg_.data[y * resized_map.cols + x] = 0; // Unknown
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Map loaded and downsampled successfully!");
    }

    /**
     * @brief Publishes the occupancy grid map
     *
     * This method is called periodically by the timer to publish the
     * pre-processed occupancy grid map to the "/custom_occupancy_grid" topic.
     * It updates the timestamp in the message header before publishing.
     */
    void publish_map()
    {
        // Update the timestamp
        occupancy_grid_msg_.header.stamp = rclcpp::Clock().now();

        // Publish the map
        map_pub->publish(occupancy_grid_msg_);
        RCLCPP_INFO(this->get_logger(), "Map published!");
    }

    /** @brief Publisher for the occupancy grid map */
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

    /** @brief Timer for periodic map publishing */
    rclcpp::TimerBase::SharedPtr timer_;

    /** @brief Original map image loaded from file */
    cv::Mat map_image_;

    /** @brief Processed occupancy grid message ready for publishing */
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg_;
};

/**
 * @brief Main function to initialize and run the MapPublisher node
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit status
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPublisher>());
    rclcpp::shutdown();
    return 0;
}