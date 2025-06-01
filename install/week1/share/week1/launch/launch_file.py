from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week1',
            executable='publisher_node',
            name='publisher_node'
        ),
        Node(
            package='week1',
            executable='subscriber_node',
            name='subscriber_node'
        )
    ])