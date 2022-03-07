from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seeker_package = 'seeker'
    seek_node_name = 'seek_node'

    seek = LaunchDescription()

    seek_node = Node(
        package = seeker_package,
        executable = seek_node_name,
        output='screen',
    )
    
    seek.add_action(seek_node)

    return seek