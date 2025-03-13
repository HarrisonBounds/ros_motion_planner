#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <chrono>
#include <future>
#include <thread>
#include <mutex>

using std::placeholders::_1;

// Structure to represent a grid cell
struct GridCell
{
    int x;
    int y;

    bool operator==(const GridCell &other) const
    {
        return x == other.x && y == other.y;
    }
};

// Hash function for GridCell to use in unordered_map
namespace std
{
    template <>
    struct hash<GridCell>
    {
        size_t operator()(const GridCell &cell) const
        {
            return hash<int>()(cell.x) ^ (hash<int>()(cell.y) << 1);
        }
    };
}

// Structure to represent a node in the A* algorithm
struct AStarNode
{
    GridCell cell;
    double g_cost; // Cost from start to current node
    double h_cost; // Heuristic cost from current node to goal
    double f_cost; // Total cost (g + h)
    GridCell parent;

    // Compare nodes for priority queue (lower f_cost has higher priority)
    bool operator>(const AStarNode &other) const
    {
        return f_cost > other.f_cost;
    }
};

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : rclcpp::Node("path_planner")
    {
        // Subscriber for the occupancy grid
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "custom_occupancy_grid", 10, std::bind(&PathPlanner::occupancyGridCallback, this, _1));

        // Publisher for the planned path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        // Initialize start and goal positions
        start_x_ = 5.0;
        start_y_ = 0.0;
        goal_x_ = 18.0;
        goal_y_ = 15.0;

        // Create the interactive marker server
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "path_planner_markers", get_node_base_interface(), get_node_clock_interface(),
            get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());

        // Timer for debouncing path updates
        debounce_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms delay
            std::bind(&PathPlanner::debouncedUpdatePath, this));
        debounce_timer_->cancel(); // Start inactive

        RCLCPP_INFO(this->get_logger(), "A* Path planner node initialized.");
    }

    // Initialize interactive markers after receiving the occupancy grid
    void initializeInteractiveMarkers()
    {
        // Create start marker
        visualization_msgs::msg::InteractiveMarker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = this->now();
        start_marker.name = "start_position";
        start_marker.description = "Start Position - Drag to move";
        start_marker.pose.position.x = start_x_;
        start_marker.pose.position.y = start_y_;
        start_marker.pose.position.z = 0.0;

        // Create a control for the marker
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "move_plane";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        start_marker.controls.push_back(control);

        // Add a visual marker
        visualization_msgs::msg::Marker visual_marker;
        visual_marker.type = visualization_msgs::msg::Marker::SPHERE;
        visual_marker.scale.x = 0.5;
        visual_marker.scale.y = 0.5;
        visual_marker.scale.z = 0.5;
        visual_marker.color.r = 0.0;
        visual_marker.color.g = 1.0;
        visual_marker.color.b = 0.0;
        visual_marker.color.a = 1.0;

        // Add the visual marker to a control
        visualization_msgs::msg::InteractiveMarkerControl visual_control;
        visual_control.always_visible = true;
        visual_control.markers.push_back(visual_marker);
        start_marker.controls.push_back(visual_control);

        // Add the marker to the server
        server_->insert(start_marker,
                        std::bind(&PathPlanner::processStartFeedback, this, std::placeholders::_1));

        // Create goal marker (similar to start marker but with red color)
        visualization_msgs::msg::InteractiveMarker goal_marker = start_marker;
        goal_marker.name = "goal_position";
        goal_marker.description = "Goal Position - Drag to move";
        goal_marker.pose.position.x = goal_x_;
        goal_marker.pose.position.y = goal_y_;

        // Change the color to red for the goal marker
        goal_marker.controls[1].markers[0].color.r = 1.0;
        goal_marker.controls[1].markers[0].color.g = 0.0;
        goal_marker.controls[1].markers[0].color.b = 0.0;

        // Add the marker to the server
        server_->insert(goal_marker,
                        std::bind(&PathPlanner::processGoalFeedback, this, std::placeholders::_1));

        // Apply changes to the server
        server_->applyChanges();
    }

private:
    // Callback for the start position marker feedback
    void processStartFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
        {
            start_x_ = feedback->pose.position.x;
            start_y_ = feedback->pose.position.y;
            RCLCPP_INFO(this->get_logger(), "Start position updated to: (%.2f, %.2f)", start_x_, start_y_);

            // Trigger debounced path update
            debounce_timer_->reset();
        }
    }

    // Callback for the goal position marker feedback
    void processGoalFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
        {
            goal_x_ = feedback->pose.position.x;
            goal_y_ = feedback->pose.position.y;
            RCLCPP_INFO(this->get_logger(), "Goal position updated to: (%.2f, %.2f)", goal_x_, goal_y_);

            // Trigger debounced path update
            debounce_timer_->reset();
        }
    }

    // Debounced path update function
    void debouncedUpdatePath()
    {
        if (!grid_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot update path: occupancy grid not received yet");
            return;
        }

        // Run A* in a separate thread
        std::future<nav_msgs::msg::Path> future = std::async(
            std::launch::async,
            &PathPlanner::computeAStarPath, this, start_x_, start_y_, goal_x_, goal_y_);

        // Publish the path when the computation is done
        nav_msgs::msg::Path path = future.get();
        path_pub_->publish(path);
    }

    // Callback for the occupancy grid
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received occupancy grid. Planning path with A*...");

        // Store the grid info for path planning
        grid_info_ = msg->info;
        occupancy_data_ = msg->data;
        grid_initialized_ = true;

        // Initialize interactive markers after receiving the first grid
        if (!markers_initialized_)
        {
            initializeInteractiveMarkers();
            markers_initialized_ = true;
        }

        // Compute initial path
        debouncedUpdatePath();
    }

    // Convert world coordinates to grid cell
    GridCell worldToGrid(double x, double y)
    {
        int grid_x = static_cast<int>((x - grid_info_.origin.position.x) / grid_info_.resolution);
        int grid_y = static_cast<int>((y - grid_info_.origin.position.y) / grid_info_.resolution);
        return {grid_x, grid_y};
    }

    // Convert grid cell to world coordinates
    std::pair<double, double> gridToWorld(int grid_x, int grid_y)
    {
        double world_x = grid_x * grid_info_.resolution + grid_info_.origin.position.x;
        double world_y = grid_y * grid_info_.resolution + grid_info_.origin.position.y;
        return {world_x, world_y};
    }

    // Check if a grid cell is valid (within bounds and not occupied)
    bool isValidCell(const GridCell &cell)
    {
        // Check bounds
        if (cell.x < 0 || cell.y < 0 ||
            cell.x >= static_cast<int>(grid_info_.width) ||
            cell.y >= static_cast<int>(grid_info_.height))
        {
            return false;
        }

        // Get the index in the 1D array
        int index = cell.y * grid_info_.width + cell.x;

        // Check if the cell is free (not occupied)
        // Typically, in occupancy grids: -1 = unknown, 0 = free, 100 = occupied
        return (index < static_cast<int>(occupancy_data_.size()) &&
                occupancy_data_[index] < 50); // Threshold for occupancy
    }

    // Calculate the Euclidean distance heuristic
    double calculateHeuristic(const GridCell &current, const GridCell &goal)
    {
        return std::sqrt(std::pow(current.x - goal.x, 2) + std::pow(current.y - goal.y, 2));
    }

    // Function to compute a path using A* algorithm
    nav_msgs::msg::Path computeAStarPath(double start_x, double start_y, double goal_x, double goal_y)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map"; // Assume the path is in the map frame

        // Convert world coordinates to grid cells
        GridCell start_cell = worldToGrid(start_x, start_y);
        GridCell goal_cell = worldToGrid(goal_x, goal_y);

        RCLCPP_INFO(this->get_logger(), "A* planning from grid (%d, %d) to (%d, %d)",
                    start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);

        // Check if start and goal cells are valid
        if (!isValidCell(start_cell))
        {
            RCLCPP_WARN(this->get_logger(), "Start position is invalid or occupied!");
            return path;
        }

        if (!isValidCell(goal_cell))
        {
            RCLCPP_WARN(this->get_logger(), "Goal position is invalid or occupied!");
            return path;
        }

        // Define movement directions (8-connected grid)
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}, // 4-connected
            {1, 1},
            {-1, 1},
            {-1, -1},
            {1, -1} // diagonals
        };

        // Priority queue for open list
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;

        // Closed set to track visited cells
        std::unordered_map<GridCell, bool> closed_set;

        // Parent map to reconstruct the path
        std::unordered_map<GridCell, GridCell> parent_map;

        // Add start node to open list
        AStarNode start_node = {start_cell, 0, calculateHeuristic(start_cell, goal_cell),
                                calculateHeuristic(start_cell, goal_cell), start_cell};
        open_list.push(start_node);

        // Cost so far map
        std::unordered_map<GridCell, double> cost_so_far;
        cost_so_far[start_cell] = 0;

        bool path_found = false;

        // A* algorithm main loop
        while (!open_list.empty())
        {
            // Get the node with the lowest f_cost
            AStarNode current = open_list.top();
            open_list.pop();

            // Check if we've reached the goal
            if (current.cell.x == goal_cell.x && current.cell.y == goal_cell.y)
            {
                parent_map[goal_cell] = current.parent;
                path_found = true;
                break;
            }

            // Skip if already in closed set
            if (closed_set.find(current.cell) != closed_set.end())
            {
                continue;
            }

            // Add current to closed set
            closed_set[current.cell] = true;

            // Expand neighbors
            for (const auto &dir : directions)
            {
                GridCell neighbor = {current.cell.x + dir.first, current.cell.y + dir.second};

                // Skip if not valid or already in closed set
                if (!isValidCell(neighbor) || closed_set.find(neighbor) != closed_set.end())
                {
                    continue;
                }

                // Calculate the cost to move to the neighbor
                double movement_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : 1.414; // Diagonal cost
                double new_cost = cost_so_far[current.cell] + movement_cost;

                // If we haven't visited this neighbor yet or found a better path
                if (cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor])
                {
                    cost_so_far[neighbor] = new_cost;
                    double heuristic = calculateHeuristic(neighbor, goal_cell);

                    // Create a new node for the neighbor
                    AStarNode neighbor_node = {neighbor, new_cost, heuristic, new_cost + heuristic, current.cell};
                    open_list.push(neighbor_node);

                    // Update parent
                    parent_map[neighbor] = current.cell;
                }
            }
        }

        // If a path was found, reconstruct it
        if (path_found)
        {
            // Reconstruct the path from goal to start
            std::vector<GridCell> path_cells;
            GridCell current = goal_cell;

            while (!(current.x == start_cell.x && current.y == start_cell.y))
            {
                path_cells.push_back(current);
                current = parent_map[current];
            }

            // Add the start cell
            path_cells.push_back(start_cell);

            // Reverse to get path from start to goal
            std::reverse(path_cells.begin(), path_cells.end());

            // Convert path cells to world coordinates and create path message
            double prev_x = 0.0, prev_y = 0.0;
            bool first_point = true;

            for (const auto &cell : path_cells)
            {
                auto [world_x, world_y] = gridToWorld(cell.x, cell.y);

                // Create a PoseStamped message for the waypoint
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = "map";
                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                pose.pose.position.z = 0.0;

                // Set orientation (facing towards the next point or goal)
                tf2::Quaternion q;
                if (first_point)
                {
                    // For the first point, use the direction to the second point
                    first_point = false;
                    prev_x = world_x;
                    prev_y = world_y;
                    continue; // Skip adding the first point until we know the direction
                }
                else
                {
                    // Calculate heading to current point from previous point
                    double heading = atan2(world_y - prev_y, world_x - prev_x);
                    q.setRPY(0, 0, heading);

                    // Add the previous point with the correct orientation
                    if (path.poses.empty())
                    {
                        geometry_msgs::msg::PoseStamped first_pose;
                        first_pose.header.stamp = this->now();
                        first_pose.header.frame_id = "map";
                        first_pose.pose.position.x = prev_x;
                        first_pose.pose.position.y = prev_y;
                        first_pose.pose.position.z = 0.0;
                        first_pose.pose.orientation = tf2::toMsg(q);
                        path.poses.push_back(first_pose);
                    }

                    // Update for the next iteration
                    prev_x = world_x;
                    prev_y = world_y;
                }

                pose.pose.orientation = tf2::toMsg(q);

                // Add the waypoint to the path
                path.poses.push_back(pose);
            }

            // Handle the final point orientation
            if (!path.poses.empty())
            {
                // Set the last point's orientation to be the same as the previous one
                path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
            }

            RCLCPP_INFO(this->get_logger(), "A* path found with %zu waypoints", path.poses.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No path found from start to goal!");
        }

        return path;
    }

    // Subscriber for the occupancy grid
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Interactive marker server
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    // Occupancy grid data
    nav_msgs::msg::MapMetaData grid_info_;
    std::vector<int8_t> occupancy_data_;

    // Start and goal positions
    double start_x_, start_y_, goal_x_, goal_y_;

    // Flags
    bool grid_initialized_ = false;
    bool markers_initialized_ = false;

    // Timer for debouncing
    rclcpp::TimerBase::SharedPtr debounce_timer_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}