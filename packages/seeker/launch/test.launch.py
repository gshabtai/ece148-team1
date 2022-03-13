from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Define package names
    pkg = 'seeker'
    # node_name = 'capture_node'

    # Define yaml config files
    # config_file = 'seeker_calibration.yaml'

    # Lanuch Descriptions
    ld = LaunchDescription()

    # config = os.path.join(
    #     get_package_share_directory(pkg),
    #     'config',
    #     config_file)

    components_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('seeker'),
                        'launch',
                        'seeker.launch.py')
                )
            )
    
    # Add actions to launch description
    ld.add_action(components_launch)

    return ld