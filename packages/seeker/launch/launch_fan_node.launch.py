from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    seeker_package = 'seeker'
    calibration_file = 'seeker_calibration.yaml'
    cap_node_name = 'fan_node'

    cap = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(seeker_package),
        'config',
        calibration_file)

    capture_node = Node(
        package = seeker_package,
        executable = cap_node_name,
        output='screen',
        parameters=[config]
    )
    
    cap.add_action(capture_node)

    return cap