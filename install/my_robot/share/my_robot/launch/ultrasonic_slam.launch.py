import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('my_robot')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    # Ultrasonic Mapper Node
    ultrasonic_mapper_node = Node(
        package='my_robot',
        executable='ultrasonic_mapper',
        name='ultrasonic_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform: map -> odom (ESSENTIAL for mapping)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz node with mapping configuration
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'ultrasonic_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        static_tf_map_odom,  # ADD THIS FIRST
        ultrasonic_mapper_node,
        rviz_node,
    ])