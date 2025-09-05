from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    
    return LaunchDescription([
        
        # Robot simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'ackermann_teleop.launch.py')
            ])
        ),
        
        # Sensor fusion (combines 6 ultrasonics into /scan)
        Node(
            package='my_robot',
            executable='sensor_fusion.py',
            output='screen'
        ),
        
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])