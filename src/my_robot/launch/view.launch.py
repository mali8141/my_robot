#!/usr/bin/env python3

"""
Launch file for viewing the robot in RViz only (no simulation)
This is useful for:
- Checking robot model visually
- Testing joint movements with joint_state_publisher_gui
- Debugging URDF/Xacro files
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory("my_robot")

    # Declare argument for robot model path
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_dir, "description", "robot.urdf.xacro"),
        description="Absolute path to the robot URDF file."
    )

    # Process the URDF file with xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot State Publisher - publishes robot TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False  # Set to False for RViz-only mode
        }]
    )

    # Joint State Publisher GUI - allows manual joint control
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # RViz2 node with custom configuration
    rviz_config_path = os.path.join(pkg_dir, "rviz", "view.rviz")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path] if os.path.exists(rviz_config_path) else []
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])