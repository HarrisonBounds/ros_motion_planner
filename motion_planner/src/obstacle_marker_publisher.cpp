#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class SpawnWall : public rclcpp::Node
{
public:
    SpawnWall()
        : Node("spawn_wall")
    {
        // Create a publisher for the marker array topic
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

        // Create a timer to periodically publish markers
        timer_ = this->create_wall_timer(500ms, std::bind(&SpawnWall::publish_wall, this));
    }

private:
    void publish_wall()
    {
        // Create a MarkerArray to hold the wall markers
        visualization_msgs::msg::MarkerArray marker_array;

        // Publish 20 square markers to form a wall
        for(int n = 0; n < 4; n++)
        {
            for (int i = 0; i < 50; ++i) {
                auto marker = create_square_marker(i);
                marker_array.markers.push_back(marker);
            }
        }
        

        // Publish the MarkerArray
        marker_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_square_marker(int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Frame ID
        marker.header.stamp = this->now();
        marker.ns = "wall";
        marker.id = id;  // Unique ID for each marker
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the square (build a wall along the x-axis)
        marker.pose.position.x = (static_cast<double>(id) * 0.5) - 5.0;  // Adjust so that the wall is centered
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the square (0.5x0.5x0.5)
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Set the color of the square (red)
        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // Fully opaque

        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpawnWall>());
    rclcpp::shutdown();
    return 0;
}
