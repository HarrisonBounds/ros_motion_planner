#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

using std::placeholders::_1;

// Structure to represent a grid cell
struct GridCell {
    int x;
    int y;
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for GridCell to use in unordered_map
namespace std {
    template<>
    struct hash<GridCell> {
        size_t operator()(const GridCell& cell) const {
            return hash<int>()(cell.x) ^ (hash<int>()(cell.y) << 1);
        }
    };
}

// Structure to represent a node in the A* algorithm
// Renamed from 'Node' to 'AStarNode' to avoid conflict with rclcpp::Node
struct AStarNode {
    GridCell cell;
    double g_cost;  // Cost from start to current node
    double h_cost;  // Heuristic cost from current node to goal
    double f_cost;  // Total cost (g + h)
    GridCell parent;
    
    // Compare nodes for priority queue (lower f_cost has higher priority)
    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : rclcpp::Node("path_planner") {
        // Subscriber for the occupancy grid
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "custom_occupancy_grid", 10, std::bind(&PathPlanner::occupancyGridCallback, this, _1));
        
        // Publisher for the planned path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        
        // Publisher for markers (start/goal)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "A* Path planner node initialized.");
    }

private:
    // Callback for the occupancy grid
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received occupancy grid. Planning path with A*...");
        
        // Store the grid info for path planning
        grid_info_ = msg->info;
        occupancy_data_ = msg->data;
        
        // Define start and goal positions (in meters)
        double start_x = 5.0, start_y = 0.0;
        double goal_x = 18.0, goal_y = 15.0;
        
        // Compute path using A*
        nav_msgs::msg::Path path = computeAStarPath(start_x, start_y, goal_x, goal_y);
        
        // Publish the path
        path_pub_->publish(path);
        
        // Publish start and goal markers
        publishStartGoalMarkers(start_x, start_y, goal_x, goal_y);
        
        RCLCPP_INFO(this->get_logger(), "A* path published to /planned_path.");
    }
    
    // Function to publish markers for start and goal
    void publishStartGoalMarkers(double start_x, double start_y, double goal_x, double goal_y) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Create start marker
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = this->now();
        start_marker.ns = "path_points";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set the marker position (start point)
        start_marker.pose.position.x = start_x;
        start_marker.pose.position.y = start_y;
        start_marker.pose.position.z = 0.0;
        
        // Set orientation
        start_marker.pose.orientation.x = 0.0;
        start_marker.pose.orientation.y = 0.0;
        start_marker.pose.orientation.z = 0.0;
        start_marker.pose.orientation.w = 1.0;
        
        // Set the marker scale
        start_marker.scale.x = 0.5;
        start_marker.scale.y = 0.5;
        start_marker.scale.z = 0.5;
        
        // Set the marker color (green for start)
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        
        start_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 means forever
        
        // Create goal marker
        visualization_msgs::msg::Marker goal_marker = start_marker;
        goal_marker.id = 1;
        
        // Set the marker position (goal point)
        goal_marker.pose.position.x = goal_x;
        goal_marker.pose.position.y = goal_y;
        
        // Set the marker color (red for goal)
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        
        // Add markers to the array
        marker_array.markers.push_back(start_marker);
        marker_array.markers.push_back(goal_marker);
        
        // Publish the marker array
        marker_pub_->publish(marker_array);
    }
    
    // Convert world coordinates to grid cell
    GridCell worldToGrid(double x, double y) {
        int grid_x = static_cast<int>((x - grid_info_.origin.position.x) / grid_info_.resolution);
        int grid_y = static_cast<int>((y - grid_info_.origin.position.y) / grid_info_.resolution);
        return {grid_x, grid_y};
    }
    
    // Convert grid cell to world coordinates
    std::pair<double, double> gridToWorld(int grid_x, int grid_y) {
        double world_x = grid_x * grid_info_.resolution + grid_info_.origin.position.x;
        double world_y = grid_y * grid_info_.resolution + grid_info_.origin.position.y;
        return {world_x, world_y};
    }
    
    // Check if a grid cell is valid (within bounds and not occupied)
    bool isValidCell(const GridCell& cell) {
        // Check bounds
        if (cell.x < 0 || cell.y < 0 || 
            cell.x >= static_cast<int>(grid_info_.width) || 
            cell.y >= static_cast<int>(grid_info_.height)) {
            return false;
        }
        
        // Get the index in the 1D array
        int index = cell.y * grid_info_.width + cell.x;
        
        // Check if the cell is free (not occupied)
        // Typically, in occupancy grids: -1 = unknown, 0 = free, 100 = occupied
        return (index < static_cast<int>(occupancy_data_.size()) && 
                occupancy_data_[index] < 50);  // Threshold for occupancy
    }
    
    // Check if a cell is an obstacle
    bool isObstacle(const GridCell& cell) {
        // First check if the cell is valid (within bounds)
        if (cell.x < 0 || cell.y < 0 || 
            cell.x >= static_cast<int>(grid_info_.width) || 
            cell.y >= static_cast<int>(grid_info_.height)) {
            return true;  // Treat out-of-bounds as obstacles
        }
        
        // Get the index in the 1D array
        int index = cell.y * grid_info_.width + cell.x;
        
        // Check if the cell is occupied
        return (index < static_cast<int>(occupancy_data_.size()) && 
                occupancy_data_[index] >= 50);  // Threshold for occupancy
    }
    
    // Calculate the Euclidean distance heuristic
    double calculateHeuristic(const GridCell& current, const GridCell& goal) {
        return std::sqrt(std::pow(current.x - goal.x, 2) + std::pow(current.y - goal.y, 2));
    }
    
    // Function to compute a path using A* algorithm
    nav_msgs::msg::Path computeAStarPath(double start_x, double start_y, double goal_x, double goal_y) {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map"; // Assume the path is in the map frame
        
        // Convert world coordinates to grid cells
        GridCell start_cell = worldToGrid(start_x, start_y);
        GridCell goal_cell = worldToGrid(goal_x, goal_y);
        
        RCLCPP_INFO(this->get_logger(), "A* planning from grid (%d, %d) to (%d, %d)", 
                   start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
        
        // Check if start and goal cells are valid
        if (!isValidCell(start_cell)) {
            RCLCPP_ERROR(this->get_logger(), "Start position is invalid or occupied!");
            return path;
        }
        
        if (!isValidCell(goal_cell)) {
            RCLCPP_ERROR(this->get_logger(), "Goal position is invalid or occupied!");
            return path;
        }
        
        // Define movement directions (8-connected grid)
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1},  // 4-connected
            {1, 1}, {-1, 1}, {-1, -1}, {1, -1}  // diagonals
        };
        
        // Priority queue for open list - use AStarNode instead of Node
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        
        // Closed set to track visited cells
        std::unordered_map<GridCell, bool> closed_set;
        
        // Parent map to reconstruct the path
        std::unordered_map<GridCell, GridCell> parent_map;
        
        // Add start node to open list - use AStarNode instead of Node
        AStarNode start_node = {start_cell, 0, calculateHeuristic(start_cell, goal_cell), 
                          calculateHeuristic(start_cell, goal_cell), start_cell};
        open_list.push(start_node);
        
        // Cost so far map
        std::unordered_map<GridCell, double> cost_so_far;
        cost_so_far[start_cell] = 0;
        
        bool path_found = false;
        
        // A* algorithm main loop
        while (!open_list.empty()) {
            // Get the node with the lowest f_cost - use AStarNode instead of Node
            AStarNode current = open_list.top();
            open_list.pop();
            
            // Check if we've reached the goal
            if (current.cell.x == goal_cell.x && current.cell.y == goal_cell.y) {
                parent_map[goal_cell] = current.parent;
                path_found = true;
                break;
            }
            
            // Skip if already in closed set
            if (closed_set.find(current.cell) != closed_set.end()) {
                continue;
            }
            
            // Add current to closed set
            closed_set[current.cell] = true;
            
            // Expand neighbors
            for (const auto& dir : directions) {
                GridCell neighbor = {current.cell.x + dir.first, current.cell.y + dir.second};
                
                // Skip if not valid or already in closed set
                if (!isValidCell(neighbor) || closed_set.find(neighbor) != closed_set.end()) {
                    continue;
                }
                
                // Calculate the cost to move to the neighbor
                double movement_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : 1.414;  // Diagonal cost
                double new_cost = cost_so_far[current.cell] + movement_cost;
                
                // If we haven't visited this neighbor yet or found a better path
                if (cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor]) {
                    cost_so_far[neighbor] = new_cost;
                    double heuristic = calculateHeuristic(neighbor, goal_cell);
                    
                    // Create a new node for the neighbor - use AStarNode instead of Node
                    AStarNode neighbor_node = {neighbor, new_cost, heuristic, new_cost + heuristic, current.cell};
                    open_list.push(neighbor_node);
                    
                    // Update parent
                    parent_map[neighbor] = current.cell;
                }
            }
        }
        
        // If a path was found, reconstruct it
        if (path_found) {
            // Reconstruct the path from goal to start
            std::vector<GridCell> path_cells;
            GridCell current = goal_cell;
            
            while (!(current.x == start_cell.x && current.y == start_cell.y)) {
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
            
            for (const auto& cell : path_cells) {
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
                if (first_point) {
                    // For the first point, use the direction to the second point
                    first_point = false;
                    prev_x = world_x;
                    prev_y = world_y;
                    continue; // Skip adding the first point until we know the direction
                } else {
                    // Calculate heading to current point from previous point
                    double heading = atan2(world_y - prev_y, world_x - prev_x);
                    q.setRPY(0, 0, heading);
                    
                    // Add the previous point with the correct orientation
                    if (path.poses.empty()) {
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
            if (!path.poses.empty()) {
                // Set the last point's orientation to be the same as the previous one
                path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
            }
            
            RCLCPP_INFO(this->get_logger(), "A* path found with %zu waypoints", path.poses.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "No path found from start to goal!");
        }
        
        return path;
    }
    
    // Subscriber for the occupancy grid
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
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