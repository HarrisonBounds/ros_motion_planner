import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('motion_planner'),
        'rviz',
        'obstacle_map.rviz'
    )

    # Node to publish obstacle markers
    obstacle_marker_node = Node(
        package='motion_planner',
        executable='obstacle_marker_publisher',
        name='obstacle_marker_publisher',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        obstacle_marker_node,
        rviz_node
    ])