from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_dir = get_package_share_directory('my_robot')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    
    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        
        # Robot simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'ackermann_teleop.launch.py')
            ])
        ),
        
        # Use ultrasonic_publisher instead of sensor_fusion for better sensor modeling
        Node(
            package='my_robot',
            executable='ultrasonic_publisher.py',  # Changed from sensor_fusion.py
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}]
        ),
    ])