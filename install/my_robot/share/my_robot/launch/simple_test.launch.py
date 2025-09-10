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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        
        # Robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
        ),
        
        # Gazebo with EMPTY world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]),
            launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
        ),
        
        # Spawn robot in empty world
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", "my_robot",
                "-x", "0", "-y", "0", "-z", "0.1",
            ],
        ),
        
        # ROS-Gazebo bridge (simplified - no config file needed)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
        ),
        
        # Fake odometry publisher (since we removed controllers)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),
        
        # Simple odometry publisher
        Node(
            package='my_robot',
            executable='simple_odom_publisher.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Your SRF08 sensor publisher (with simple room simulation)
        Node(
            package='my_robot',
            executable='srf08_publisher.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # SLAM with robust parameters
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_publish_period': 0.02,
                'link_match_minimum_response_fine': 0.02,
                'correlation_search_space_dimension': 1.0,
                'minimum_time_interval': 0.3,
            }]
        ),
    ])