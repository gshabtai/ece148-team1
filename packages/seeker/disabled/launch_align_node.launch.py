from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seeker_package = 'seeker'
    align_node_name = 'align_node'

    align = LaunchDescription()

    align_node = Node(
        package = seeker_package,
        executable = align_node_name,
        output='screen',
    )
    
    align.add_action(align_node)

    return align