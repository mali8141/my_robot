from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    params_path = os.path.join(
        get_package_share_directory('my_robot'),
        'config', 'parameters.yaml'
    )

    return LaunchDescription([
        # Spawn joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        
        # Spawn front steering controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["front_steering_controller"],
        ),
        
        # Spawn rear velocity controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["rear_velocity_controller"],
        ),
        
        # Start Ackermann controller
        Node(
            package="my_robot",
            executable="ackermann_controller.py",
            parameters=[params_path],
            output="screen",
        ),
    ])