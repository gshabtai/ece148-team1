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

from packages.seeker.seeker import collision_avoidance

def generate_launch_description():
    # Define package names
    seeker_pkg = 'seeker'
    actuator_pkg = 'ucsd_robocar_actuator2_pkg'
    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    intel_pkg_name = 'realsense2_camera'

    # Define yaml config files
    seeker_calibration_file = 'seeker_calibration.yaml'
    act_calibration_file = 'adafruit_twist_calibration.yaml'
    lidar_config_file = 'ld06.yaml'

    # Define node names
    fan_controller_node_name = 'fan_controller'

    # Define Intel launch file
    intel_launch_file = 'rs_launch.py'

    # Tell ros2 to use the yaml config files
    config_seeker = os.path.join(
        get_package_share_directory(seeker_pkg),
        'config',
        seeker_calibration_file)

    config_actuator = os.path.join(
        get_package_share_directory(actuator_pkg),
        'config',
        act_calibration_file)

    config_lidar = os.path.join(
        get_package_share_directory(sensor_pkg),
        'config',
        lidar_config_file)


    # Define nodes provided by the Dominic
    webcam_node = Node(
        package = sensor_pkg,
        executable = 'webcam_node',
        output='screen',
        parameters=[config_actuator]
    )

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

    # Define nodes
    webcam_publish_centroid_node = Node(
        package = seeker_pkg,
        executable = 'webcam_publish_centroid',
        output='screen',
        parameters=[config_seeker]
    )

    state_machine = Node(
        package = seeker_pkg,
        executable = 'state_controller',
        output = 'screen',
        parameters=[config_seeker]
    )

    collision_avoidance_node = Node(
        package = sensor_pkg,
        executable = 'collision_avoidance',
        output = 'screen'
    )

    # intel_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(
    #                 get_package_share_directory(intel_pkg_name),
    #                 'launch',
    #                 intel_launch_file)
    #         )
    #     )
    
    # Add actions to launch description
    ld = LaunchDescription()

    # Dominic
    ld.add_action(webcam_node)
    ld.add_action(adafruit_node)
    ld.add_action(lidar_node)

    # Ours
    ld.add_action(webcam_publish_centroid_node)
    ld.add_action(state_machine)
    ld.add_action(collision_avoidance_node)

    return ld