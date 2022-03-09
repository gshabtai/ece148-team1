from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

def generate_launch_description():
    seeker_pkg = 'seeker'
    actuator_pkg = 'ucsd_robocar_actuator2_pkg'
    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    intel_pkg_name = 'realsense2_camera'
    seeker_calibration_file = 'seeker_calibration.yaml'
    act_calibration_file = 'adafruit_twist_calibration.yaml'

    capture_node_name = 'capture_node'
    centroid_node_name = 'centroid_node'
    fan_node_name = 'fan_node'
    actuator_node_name = 'adafruit_twist_node'
    webcam_node_name = 'webcam_node'

    intel_launch_file = 'rs_launch.py'

    ld = LaunchDescription()

    config_seeker = os.path.join(
        get_package_share_directory(seeker_pkg),
        'config',
        seeker_calibration_file)

    config_actuator = os.path.join(
        get_package_share_directory(actuator_pkg),
        'config',
        act_calibration_file)

    capture_node = Node(
        package = seeker_pkg,
        executable = capture_node_name,
        output='screen',
        parameters=[config_seeker]
    )

    centroid_node = Node(
        package = seeker_pkg,
        executable = centroid_node_name,
        output='screen',
        parameters=[config_seeker]
    )

    fan_node = Node(
        package = seeker_pkg,
        executable = fan_node_name,
        output='screen',
        parameters=[config_seeker]
    )

    act_node = Node(
        package = actuator_pkg,
        executable = actuator_node_name,
        output='screen',
        parameters=[config_actuator]
    )

    webcam_node = Node(
        package = sensor_pkg,
        executable = webcam_node_name,
        output='screen',
        parameters=[config_actuator]
    )

    # intel_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(
    #                 get_package_share_directory(intel_pkg_name),
    #                 'launch',
    #                 intel_launch_file)
    #         )
    #     )
    
    ld.add_action(capture_node)
    ld.add_action(centroid_node)
    ld.add_action(fan_node)
    ld.add_action(act_node)
    ld.add_action(webcam_node)
    # ls.add_action(intel_launch)

    return ld