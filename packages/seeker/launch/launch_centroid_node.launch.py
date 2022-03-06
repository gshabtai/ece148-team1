from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seeker_package = 'seeker'
    cap_node_name = 'centroid_node'

    cap = LaunchDescription()

    capture_node = Node(
        package = seeker_package,
        executable = cap_node_name,
        output='screen',
    )
    
    cap.add_action(capture_node)

    return cap