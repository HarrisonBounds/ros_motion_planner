from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    # Create the map server node
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',  
        output='screen',
        parameters=[{'yaml_filename': 'maps/map1.yaml'}],
        remappings=[("/map", "/map")]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
                'node_names': ['map_server']  # Add map_server as a managed node
            }],
        output='screen',
    )

    # Transition map_server to active state
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.name == 'map_server',
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=map_server_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'map_server',
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node,
        configure_event,
        activate_event,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'config/map_display.rviz']  # Optional: Path to your RViz config file
        )
    ])