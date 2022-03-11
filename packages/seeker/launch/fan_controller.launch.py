from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    seeker_package = 'seeker'
    calibration_file = 'seeker_calibration.yaml'
    cap_node_name = 'intake_system'

    cap = LaunchDescription()

    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    actuator_pkg = 'ucsd_robocar_actuator2_pkg'
    act_calibration_file = 'adafruit_twist_calibration.yaml'

    config_actuator = os.path.join(
        get_package_share_directory(actuator_pkg),
        'config',
        act_calibration_file)

    webcam_node = Node(
        package = sensor_pkg,
        executable = 'webcam_node',
        output='screen',
        parameters=[config_actuator]
    )

    capture_node = Node(
        package = seeker_package,
        executable = cap_node_name,
        output='screen',
    )
    
    cap.add_action(capture_node)
    cap.add_action(webcam_node)

    return cap