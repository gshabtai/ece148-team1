from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seeker_package = 'seeker'
    ball_node_name = 'centroid_node'

    ball = LaunchDescription()

    find_ball_node = Node(
        package = seeker_package,
        executable = ball_node_name,
        output='screen',
    )
    
    ball.add_action(find_ball_node)

    return ball