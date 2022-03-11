from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Define package names
    pkg = 'seeker'
    node_name = 'collision_avoidance_node'

    # Define yaml config files
    config_file = 'seeker.config.yaml'

    # Lanuch Descriptions
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(pkg),
        'config',
        config_file)

    collision_avoidance_node = Node(
        package = pkg,
        executable = node_name,
        output = 'screen',
        parameter=[config]
    )
    
    # Add actions to launch description
    ld.add_action(collision_avoidance_node)

    return ld