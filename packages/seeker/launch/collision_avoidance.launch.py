from distutils.command.config import config
from http.server import executable
from importlib.resources import Package
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

def generate_launch_description():
    # Define package names
    seeker_pkg = 'seeker'
    actuator_pkg = 'ucsd_robocar_actuator2_pkg'
    sensor_pkg = 'ucsd_robocar_sensor2_pkg'

    # Define yaml config files
    act_calibration_file = 'adafruit_twist_calibration.yaml'
    lidar_config_file = 'ld06.yaml'

    config_actuator = os.path.join(
        get_package_share_directory(actuator_pkg),
        'config',
        act_calibration_file)

    config_lidar = os.path.join(
        get_package_share_directory(sensor_pkg),
        'config',
        lidar_config_file)

    lidar_node = Node(
        package = 'ldlidar',
        executable = 'ldlidar',
        output='screen',
        parameters=[config_lidar]
    )

    adafruit_node = Node(
        package = actuator_pkg,
        executable = 'adafruit_twist_node',
        output='screen',
        parameters=[config_actuator]
    )

    collision_avoidance_node = Node(
        package = seeker_pkg,
        executable = 'collision_avoidance',
        output = 'screen'
    )
    
    # Add actions to launch description
    ld = LaunchDescription()

    # Dominic
    ld.add_action(adafruit_node)
    ld.add_action(lidar_node)

    # Ours
    ld.add_action(collision_avoidance_node)

    return ld