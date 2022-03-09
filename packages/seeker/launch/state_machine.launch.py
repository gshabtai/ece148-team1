from http.server import executable
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
    intel_pkg_name = 'realsense2_camera'
    seeker_calibration_file = 'seeker_calibration.yaml'
    act_calibration_file = 'adafruit_twist_calibration.yaml'

    # Define node names
    collision_avoidance_node_name = 'collision_avoidance'
    fan_controller_node_name = 'fan_controller'
    webcam_publish_centroid_node_name = 'webcam_publish_centroid'
    actuator_node_name = 'adafruit_twist_node'
    webcam_node_name = 'webcam_node'
    state_machine_node_name = 'state_controller'

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

    # Define nodes provided by the Dominic
    webcam_node = Node(
        package = sensor_pkg,
        executable = webcam_node_name,
        output='screen',
        parameters=[config_actuator]
    )

    # Define nodes
    webcam_publish_centroid_node = Node(
        package = seeker_pkg,
        executable = webcam_publish_centroid_node_name,
        # output='screen',
        parameters=[config_seeker]
    )

    state_machine = Node(
        package = seeker_pkg,
        executable = state_machine_node_name,
        output = 'screen',
        parameters=[config_seeker]
    )

    # centroid_node = Node(
    #     package = seeker_pkg,
    #     executable = centroid_node_name,
    #     output='screen',
    #     parameters=[config_seeker]
    # )

    # fan_node = Node(
    #     package = seeker_pkg,
    #     executable = fan_node_name,
    #     output='screen',
    #     parameters=[config_seeker]
    # )

    # act_node = Node(
    #     package = actuator_pkg,
    #     executable = actuator_node_name,
    #     output='screen',
    #     parameters=[config_actuator]
    # )

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
    ld.add_action(webcam_node)
    ld.add_action(webcam_publish_centroid_node)
    ld.add_action(state_machine)
    # ld.add_action(fan_node)
    # ld.add_action(act_node)
    # ld.add_action(webcam_node)
    # ls.add_action(intel_launch)

    return ld