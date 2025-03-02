#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : Node("path_planner") {
        // Subscriber for the occupancy grid
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "custom_occupancy_grid", 10, std::bind(&PathPlanner::occupancyGridCallback, this, _1));

        // Publisher for the planned path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(this->get_logger(), "Path planner node initialized.");
    }

private:
    // Callback for the occupancy grid
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received occupancy grid. Planning path...");

        // Store the grid info for path planning
        grid_info_ = msg->info;
        occupancy_data_ = msg->data;

        // Compute a simple straight-line path
        nav_msgs::msg::Path path = computeStraightLinePath();

        // Publish the path
        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Path published to /planned_path.");
    }

    // Function to compute a straight-line path
    nav_msgs::msg::Path computeStraightLinePath() {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map"; // Assume the path is in the map frame

        // Define start and goal positions (in meters)
        double start_x = 0.0, start_y = 0.0;
        double goal_x = 5.0, goal_y = 5.0;

        // Number of waypoints
        int num_waypoints = 10;

        // Generate waypoints along the straight line
        for (int i = 0; i <= num_waypoints; ++i) {
            double x = start_x + (goal_x - start_x) * i / num_waypoints;
            double y = start_y + (goal_y - start_y) * i / num_waypoints;

            // Create a PoseStamped message for the waypoint
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            // Set orientation (facing towards the goal)
            tf2::Quaternion q;
            q.setRPY(0, 0, atan2(goal_y - start_y, goal_x - start_x));
            pose.pose.orientation = tf2::toMsg(q);

            // Add the waypoint to the path
            path.poses.push_back(pose);
        }

        return path;
    }

    // Subscriber for the occupancy grid
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    // Publisher for the planned path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Occupancy grid data
    nav_msgs::msg::MapMetaData grid_info_;
    std::vector<int8_t> occupancy_data_;
};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}