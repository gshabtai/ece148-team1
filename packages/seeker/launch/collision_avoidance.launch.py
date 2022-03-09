from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seeker_package = 'seeker'
    cen_node_name = 'collision_avoidance'

    cen = LaunchDescription()

    centroid_node = Node(
        package = seeker_package,
        executable = cen_node_name,
        output='screen',
    )
    
    cen.add_action(centroid_node)

    return cen