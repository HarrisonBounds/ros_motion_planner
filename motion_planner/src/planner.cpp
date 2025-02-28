#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cstdlib>  // For rand() function
#include <ctime>    // For seeding random number generator
#include <vector>
#include <queue>
#include <unordered_set>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

// Struct to represent a point in 2D grid
struct Point2D {
    int x, y;
    
    bool operator==(const Point2D& other) const {
        return x == other.x && y == other.y;
    }
};

// Custom hash function for Point2D to use in unordered_set
namespace std {
    template <>
    struct hash<Point2D> {
        size_t operator()(const Point2D& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

class SimpleMotionPlanner : public rclcpp::Node
{
public:
    SimpleMotionPlanner()
        : Node("simple_motion_planner")
    {
        // Seed the random number generator
        std::srand(std::time(0));

        // Create a publisher for the marker array topic
        environment_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);
        trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/trajectory", 10);
        
        // Create interactive marker server
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "motion_planner_markers", 
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_topics_interface(),
            this->get_node_services_interface()
        );
        
        // Create service to trigger planning
        plan_service_ = this->create_service<std_srvs::srv::Trigger>(
            "plan_path",
            std::bind(&SimpleMotionPlanner::handle_plan_request, this, 
                      std::placeholders::_1, std::placeholders::_2)
        );
        
        // Initialize the environment and interactive markers
        init_environment();
        init_interactive_markers();
        
        // Create a timer to periodically publish markers
        timer_ = this->create_wall_timer(500ms, std::bind(&SimpleMotionPlanner::publish_environment, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple Motion Planner initialized");
    }

private:
    // Interactive marker feedback callback
    void process_marker_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        if (feedback->marker_name == "start_marker") {
            start_pos_.x = round(feedback->pose.position.x * 2.0) / 2.0;
            start_pos_.y = round(feedback->pose.position.y * 2.0) / 2.0;
            start_pos_.z = 0.25;

            // Create a new interactive marker with the updated position
            auto start_marker = create_interactive_marker("start_marker", start_pos_, make_color(0.0, 1.0, 0.0, 1.0));
            marker_server_->insert(start_marker);
            marker_server_->applyChanges();

            RCLCPP_INFO(this->get_logger(), "Start position updated: %.1f, %.1f", start_pos_.x, start_pos_.y);
        }
        else if (feedback->marker_name == "goal_marker") {
            goal_pos_.x = round(feedback->pose.position.x * 2.0) / 2.0;
            goal_pos_.y = round(feedback->pose.position.y * 2.0) / 2.0;
            goal_pos_.z = 0.25;

            // Create a new interactive marker with the updated position
            auto goal_marker = create_interactive_marker("goal_marker", goal_pos_, make_color(1.0, 0.0, 1.0, 1.0));
            marker_server_->insert(goal_marker);
            marker_server_->applyChanges();

            RCLCPP_INFO(this->get_logger(), "Goal position updated: %.1f, %.1f", goal_pos_.x, goal_pos_.y);
        }
    }
    
    // Handle planning service request
    void handle_plan_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Planning path from (%.1f, %.1f) to (%.1f, %.1f)",
                   start_pos_.x, start_pos_.y, goal_pos_.x, goal_pos_.y);
        
        bool success = plan_path();
        
        response->success = success;
        response->message = success ? "Path planned successfully" : "Failed to find a path";
        
        if (success) {
            visualize_trajectory();
        }
    }
    
    // Initialize the environment (walls and obstacles)
    void init_environment()
    {
        // Define wall length (number of cubes per wall)
        wall_length_ = 20;
        
        // Clear the current environment
        environment_markers_.markers.clear();
        obstacle_positions_.clear();
        
        // Create walls
        for (int wall = 0; wall < 4; ++wall) {
            for (int i = 0; i < wall_length_; ++i) {
                auto marker = create_square_marker(wall, i, wall_length_);
                environment_markers_.markers.push_back(marker);
                
                // Store wall positions for collision checking
                Point2D wall_pos;
                wall_pos.x = static_cast<int>(marker.pose.position.x * 2); // Scale to grid
                wall_pos.y = static_cast<int>(marker.pose.position.y * 2); // Scale to grid
                obstacle_positions_.insert(wall_pos);
            }
        }
        
        // Add random sphere obstacles within the walls
        add_random_obstacles();
        
        // Initialize start and goal positions
        start_pos_.x = -wall_length_ * 0.25 + 1.0;
        start_pos_.y = -wall_length_ * 0.25 + 1.0;
        start_pos_.z = 0.25;
        
        goal_pos_.x = wall_length_ * 0.25 - 1.0;
        goal_pos_.y = wall_length_ * 0.25 - 1.0;
        goal_pos_.z = 0.25;
    }
    
    // Create interactive markers for start and goal
    void init_interactive_markers()
    {
        // Create start marker
        auto start_marker = create_interactive_marker("start_marker", start_pos_, 
                                                     make_color(0.0, 1.0, 0.0, 1.0)); // Green
        marker_server_->insert(start_marker);
        marker_server_->setCallback(start_marker.name, 
                                  std::bind(&SimpleMotionPlanner::process_marker_feedback, this, 
                                           std::placeholders::_1));
        
        // Create goal marker
        auto goal_marker = create_interactive_marker("goal_marker", goal_pos_, 
                                                    make_color(1.0, 0.0, 1.0, 1.0)); // Purple
        marker_server_->insert(goal_marker);
        marker_server_->setCallback(goal_marker.name, 
                                  std::bind(&SimpleMotionPlanner::process_marker_feedback, this, 
                                           std::placeholders::_1));
        
        marker_server_->applyChanges();
    }
    
    // Create an interactive marker
    visualization_msgs::msg::InteractiveMarker create_interactive_marker(
        const std::string& name, 
        const geometry_msgs::msg::Point& position,
        const std_msgs::msg::ColorRGBA& color)
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = name;
        int_marker.description = name;
        int_marker.pose.position = position;
        int_marker.pose.orientation.w = 1.0;
        int_marker.scale = 1.0;
        
        // Create a marker inside the interactive marker
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color = color;
        
        // Create interactive marker control
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        control.orientation.w = 1.0;
        control.orientation.z = 1.0; // z-axis is up
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);
        
        return int_marker;
    }
    
    // Helper to create a color
    std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
    {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
    
    // Function to publish the environment
    void publish_environment()
    {
        environment_markers_.markers[0].header.stamp = this->now();
        environment_pub_->publish(environment_markers_);
    }
    
    // Function to create a square marker for walls
    visualization_msgs::msg::Marker create_square_marker(int wall, int id, int wall_length)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Frame ID
        marker.header.stamp = this->now();
        marker.ns = "wall";
        marker.id = wall * wall_length + id;  // Unique ID for each marker
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the square based on the wall index
        switch (wall) {
            case 0:  // Top wall (positive Y-axis)
                marker.pose.position.x = static_cast<double>(id) * 0.5;  // X position (spacing by 0.5)
                marker.pose.position.y = wall_length * 0.25;  // Y position is fixed
                break;
            case 1:  // Right wall (positive X-axis)
                marker.pose.position.x = wall_length * 0.25;  // X position is fixed
                marker.pose.position.y = static_cast<double>(id) * 0.5;  // Y position (spacing by 0.5)
                break;
            case 2:  // Bottom wall (negative Y-axis)
                marker.pose.position.x = static_cast<double>(id) * 0.5;  // X position (spacing by 0.5)
                marker.pose.position.y = -wall_length * 0.25;  // Y position is fixed
                break;
            case 3:  // Left wall (negative X-axis)
                marker.pose.position.x = -wall_length * 0.25;  // X position is fixed
                marker.pose.position.y = static_cast<double>(id) * 0.5;  // Y position (spacing by 0.5)
                break;
        }

        marker.pose.position.z = 0.25;
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

    // Function to add random obstacles (spheres) inside the walls
    void add_random_obstacles()
    {
        int num_obstacles = 10;  // Number of obstacles to add

        for (int i = 0; i < num_obstacles; ++i) {
            // Generate random position within the walls
            double rand_x = (std::rand() % (wall_length_)) * 0.5 - wall_length_ * 0.25 + 0.5;
            double rand_y = (std::rand() % (wall_length_)) * 0.5 - wall_length_ * 0.25 + 0.5;

            auto obstacle = create_sphere_marker(i, rand_x, rand_y);
            environment_markers_.markers.push_back(obstacle);
            
            // Store obstacle position for collision checking
            Point2D obs_pos;
            obs_pos.x = static_cast<int>(rand_x * 2); // Scale to grid
            obs_pos.y = static_cast<int>(rand_y * 2); // Scale to grid
            obstacle_positions_.insert(obs_pos);
        }
    }

    // Function to create a sphere marker for obstacles
    visualization_msgs::msg::Marker create_sphere_marker(int id, double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Frame ID
        marker.header.stamp = this->now();
        marker.ns = "obstacle";
        marker.id = id;  // Unique ID for each obstacle
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the sphere
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.25;  // Set a small height so the sphere is above the ground

        marker.pose.orientation.w = 1.0;

        // Set the scale of the sphere (radius = 0.25)
        marker.scale.x = 0.5;  // Radius along x-axis
        marker.scale.y = 0.5;  // Radius along y-axis
        marker.scale.z = 0.5;  // Radius along z-axis

        // Set the color of the sphere (blue)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // Blue
        marker.color.a = 1.0;  // Fully opaque

        return marker;
    }
    
    // Plan a path from start to goal using BFS
    bool plan_path()
    {
        // Convert start and goal to grid coordinates
        Point2D start = {static_cast<int>(start_pos_.x * 2), static_cast<int>(start_pos_.y * 2)};
        Point2D goal = {static_cast<int>(goal_pos_.x * 2), static_cast<int>(goal_pos_.y * 2)};
        
        // Store parent for each visited cell to reconstruct path
        std::unordered_map<Point2D, Point2D> came_from;
        
        // BFS queue
        std::queue<Point2D> queue;
        queue.push(start);
        came_from[start] = start; // Start has no parent
        
        // Possible moves (4-connected grid: up, right, down, left)
        const std::vector<std::pair<int, int>> directions = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}
        };
        
        bool found_path = false;
        
        while (!queue.empty() && !found_path) {
            Point2D current = queue.front();
            queue.pop();
            
            // Check if we've reached the goal
            if (current.x == goal.x && current.y == goal.y) {
                found_path = true;
                break;
            }
            
            // Try all possible moves
            for (const auto& dir : directions) {
                Point2D next = {current.x + dir.first, current.y + dir.second};
                
                // Check if already visited
                if (came_from.find(next) != came_from.end()) {
                    continue;
                }
                
                // Check if obstacle
                if (obstacle_positions_.find(next) != obstacle_positions_.end()) {
                    continue;
                }
                
                // Check if outside the bounds
                if (next.x < -wall_length_ || next.x > wall_length_ || 
                    next.y < -wall_length_ || next.y > wall_length_) {
                    continue;
                }
                
                // Add to queue and record parent
                queue.push(next);
                came_from[next] = current;
            }
        }
        
        // If no path found
        if (!found_path) {
            RCLCPP_ERROR(this->get_logger(), "No path found from start to goal");
            return false;
        }
        
        // Reconstruct path from goal to start
        path_.clear();
        Point2D current = goal;
        
        while (!(current.x == start.x && current.y == start.y)) {
            // Convert grid coordinates back to world coordinates
            geometry_msgs::msg::Point point;
            point.x = current.x / 2.0;
            point.y = current.y / 2.0;
            point.z = 0.25;
            path_.push_back(point);
            
            current = came_from[current];
        }
        
        // Add start position
        geometry_msgs::msg::Point start_point;
        start_point.x = start.x / 2.0;
        start_point.y = start.y / 2.0;
        start_point.z = 0.25;
        path_.push_back(start_point);
        
        // Reverse path to get start-to-goal order
        std::reverse(path_.begin(), path_.end());
        
        RCLCPP_INFO(this->get_logger(), "Path found with %zu points", path_.size());
        return true;
    }
    
    // Visualize the planned trajectory
    void visualize_trajectory()
    {
        if (path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path to visualize");
            return;
        }
        
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        
        line_strip.scale.x = 0.1;  // Line width
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        
        line_strip.points = path_;
        
        trajectory_pub_->publish(line_strip);
        RCLCPP_INFO(this->get_logger(), "Published trajectory visualization");
    }

    // Member variables
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environment_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
    
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    
    visualization_msgs::msg::MarkerArray environment_markers_;
    std::vector<geometry_msgs::msg::Point> path_;
    
    geometry_msgs::msg::Point start_pos_;
    geometry_msgs::msg::Point goal_pos_;
    
    int wall_length_;
    std::unordered_set<Point2D> obstacle_positions_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMotionPlanner>());
    rclcpp::shutdown();
    return 0;
}