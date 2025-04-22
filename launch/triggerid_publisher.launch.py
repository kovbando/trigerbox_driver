# filepath: /home/teszt/ros2_ws/triggerbox/launch/triggerid_publisher.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='triggerbox',
            executable='triggerid_publisher',
            name='triggerid_publisher',
            output='screen',
            parameters=[
                {'udp_port': 5555},
                {'output_topic': '/triggerid'},
            ],
        ),
    ])