# Interactive ROS2 Path Planner

---

## Who is this for?
This project was made to read slam maps and dynamically plan paths in RVIZ using them. Our packages are tested with ROS2 Jazzy Distribution and fall under an MIT License Agreement. 

These packages can read black and white PNG maps and display them as an occupancy grid. We are currently working on being able to read PGM files as those are pretty common for SLAM applications. Once the map is read in using the **spawn_map** package, you can dynamically visualize the shortest path from the start node to the goal node with interactive markers! Just drag them around and see the path update automatically. In the future we would like to add additional path planning algorithms, but due to time constraints we have not been able to pursue this.

---

## Setup Instructions

Please make sure you have ROS2 Jazzy installed on your machine. Instructions can be found here: https://docs.ros.org/en/jazzy/Installation.html

Setup your files similar to this (need a workspace, src, and repo)

```
cd ~/ws/pathplanner/src/

git clone <this repo>

cd ../..

colcon build

Ctrl+Shift+T (Open new terminal tab)

source install/setup.bash

ros2 launch bringup bringup.launch.xml

OR 

Run each package individually to see how they work separately

```

### Running a Custom Map

When running a custom map, there are two options. There is a `generate_custom_occupancy_grid` node in the spawn_map package that manually creates a map via an array of values where 100 represents a wall and 0 represents a free area. To make your own map manually you can edit these values, or create them with an algorithm using a for loop

Another option is reading your own file. If you want to read your own PNG map file, make sure its in the `maps` folder in the `spawn_map` package. Then in the `generate_occupancy_grid` node, edit the file name to the one that matches the name of your map. 

---

### Parameters

Depending on what map you want to read, there are a few changes that may need to be made to make the sure path updates as quickly as possible.

- Map Resolution
- Free threshold
- Occupied Threshold
- Map Downsample size

Each of these parameters are in the `generate_occupancy_grid` node and require tuning for specific use cases

---

## Package Descriptions 

##### spawn_map

Reads a custom PNG map file and displays in it RVIZ.

##### pathplanner

The driver package for this project. Creates the interactive markers identified as the start and goal nodes. Calculates the path between these nodes using the a star algorithm and multi threading.

##### bringup

Houses the launch file for running the other two packages. Also specifies the config file for adding components in rviz.

---

## A Star Planning Algorithm

1. **Inputs**:
   - **Start Position**: (`start_x_`, `start_y_`) in world coordinates.
   - **Goal Position**: (`goal_x_`, `goal_y_`) in world coordinates.
   - **Occupancy Grid**: A 2D grid where each cell is either free (0), occupied (100), or unknown (-1).

2. **Grid Representation**:
   - World coordinates are converted to grid cells using `worldToGrid`.
   - Each cell is represented by a `GridCell` structure (`x`, `y` indices).

3. **Cost Functions**:
   - **`g_cost`**: Actual cost from the start to the current cell.
   - **`h_cost`**: Heuristic cost (Euclidean distance) from the current cell to the goal.
   - **`f_cost`**: Total cost (`f_cost = g_cost + h_cost`), used to prioritize cells.

4. **Exploration**:
   - Uses a **priority queue** (`open_list`) to explore cells with the lowest `f_cost` first.
   - Keeps track of visited cells in a **closed set** to avoid reprocessing.

5. **Path Reconstruction**:
   - Once the goal is reached, the path is reconstructed by backtracking from the goal to the start using a **parent map**.

6. **Output**:
   - The path is converted back to world coordinates and published as a `nav_msgs::msg::Path`.