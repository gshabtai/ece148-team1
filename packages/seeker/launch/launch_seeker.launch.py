from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seeker_package = 'seeker'
    cap_node_name = 'centroid_node'

    cap = LaunchDescription()

    return cap