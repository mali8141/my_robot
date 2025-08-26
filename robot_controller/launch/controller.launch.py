from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    front_steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "front_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    rear_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    ackermann_controller = Node(
        package="robot_controller",
        executable="ackermann_controller.py",
        parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "parameters.yaml")],
        output="screen",
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            front_steering_controller,
            rear_velocity_controller,
            ackermann_controller,
        ]
    )