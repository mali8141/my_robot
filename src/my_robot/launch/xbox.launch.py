from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Joystick driver
        Node(
            package='joy',
            executable='joy_node'
        ),
        
        # Xbox controller
        Node(
            package='my_robot',
            executable='xbox_controller.py'
        ),
    ])