from distutils.command.config import config
from http.server import executable
from importlib.resources import Package
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

parameter_input_path = '/home/projects/ros2_ws/src/seeker/config/seeker_calibration.yaml'
packages_info_path = '/home/projects/ros2_ws/src/seeker/config/pkg_locations.yaml'

def update_parameters(parameter_input_path):
        with open(parameter_input_path, "r") as file:
            inputs = yaml.load(file, Loader=yaml.FullLoader)
            my_inputs = {}
            for key in inputs:
                value = inputs[key]
                if value==1:
                    my_inputs[key] = value
            return my_inputs

def update_packages(packages_info_path):
    with open(packages_info_path, "r") as file:
            packages_dict = yaml.load(file, Loader=yaml.FullLoader)
            inputs_dict = update_parameters(parameter_input_path)
            my_packages = {}
            for key in packages_dict:
                value = packages_dict[key]
                if key in inputs_dict:
                    my_packages[key] = value
            return my_packages

def generate_a_launch_description(package, launch):
    return LaunchDescription([
    DeclareLaunchArgument(
            'topic_name',
            default_value = 'default_value',
            description = 'REQUIRED argument for new topic name (can use original topic name if needed)'
        ),
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package),
                    'launch',
                    launch)
            ),
        ),
    ])

def generate_launch_description():
    my_packages_dict = update_packages(packages_info_path)
    ld = LaunchDescription()
    for key in my_packages_dict:
        pkg_name = my_packages_dict[key][0]
        launch_name = my_packages_dict[key][1]
        component_type = my_packages_dict[key][2]
        # ld_list = []
        try:
            ld.add_action(generate_a_launch_description(pkg_name, launch_name))
        except:
            pass
        print(f"Trying to start {component_type}: {launch_name} from {pkg_name}")
    return ld