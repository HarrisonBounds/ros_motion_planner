#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using std::placeholders::_1;

class OccupancyGridSubscriber : public rclcpp::Node {
public:
    OccupancyGridSubscriber() : Node("occupancy_grid_subscriber") {
        // Create a subscriber for the custom topic "custom_occupancy_grid"
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "custom_occupancy_grid", 10, std::bind(&OccupancyGridSubscriber::occupancyGridCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: custom_occupancy_grid");
    }

private:
    // Callback function for the occupancy grid subscriber
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Print basic information about the occupancy grid
        RCLCPP_INFO(this->get_logger(), "Received occupancy grid:");
        RCLCPP_INFO(this->get_logger(), "Map dimensions: %d x %d", msg->info.width, msg->info.height);
        RCLCPP_INFO(this->get_logger(), "Map resolution: %f m/cell", msg->info.resolution);
        RCLCPP_INFO(this->get_logger(), "Map origin: (%f, %f, %f)",
                    msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z);

        // Print the occupancy data (optional: limit the output for large maps)
        RCLCPP_INFO(this->get_logger(), "Occupancy data (first 10x10 cells):");
        for (uint32_t y = 0; y < msg->info.height; ++y) {
            for (uint32_t x = 0; x < msg->info.width; ++x) {
                uint32_t index = y * msg->info.width + x;
                RCLCPP_INFO(this->get_logger(), "Cell (%d, %d): %d", x, y, msg->data[index]);
            }
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridSubscriber>());
    rclcpp::shutdown();
    return 0;
}