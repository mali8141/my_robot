#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
import math
from tf2_ros import TransformListener, Buffer, TransformException

class SimpleMapper(Node):
    """
    Simple occupancy grid mapper using fused laser scan data
    """
    
    def __init__(self):
        super().__init__('simple_mapper')
        
        # Map parameters
        self.map_width = 400  # cells (40m at 0.1m resolution)
        self.map_height = 400  # cells
        self.map_resolution = 0.1  # meters per cell
        
        # Initialize occupancy grid (-1=unknown, 0=free, 100=occupied)
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Subscriber for fused laser scan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for occupancy grid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # TF2 listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to publish map
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Simple Mapper Node Started - using /scan topic')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        try:
            # Get robot pose
            robot_x, robot_y, robot_yaw = self.get_robot_pose()
            
            # Process each range measurement
            for i, range_val in enumerate(msg.ranges):
                if msg.range_min < range_val < msg.range_max:
                    # Calculate angle for this measurement
                    angle = msg.angle_min + i * msg.angle_increment
                    global_angle = robot_yaw + angle
                    
                    # Mark free space along the beam
                    self.mark_ray(robot_x, robot_y, global_angle, range_val, free_space=True)
                    
                    # Mark occupied space at the end
                    self.mark_point(robot_x, robot_y, global_angle, range_val, occupied=True)
                    
        except Exception as e:
            self.get_logger().debug(f'Could not process scan: {e}')
    
    def get_robot_pose(self):
        """Get robot pose from TF tree"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            orientation = transform.transform.rotation
            x_q, y_q, z_q, w_q = orientation.x, orientation.y, orientation.z, orientation.w
            
            siny_cosp = 2 * (w_q * z_q + x_q * y_q)
            cosy_cosp = 1 - 2 * (y_q * y_q + z_q * z_q)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
            
        except TransformException:
            return 0.0, 0.0, 0.0
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x / self.map_resolution) + (self.map_width // 2))
        grid_y = int((y / self.map_resolution) + (self.map_height // 2))
        return grid_x, grid_y
    
    def mark_ray(self, robot_x, robot_y, angle, max_range, free_space=True):
        """Mark free space along a ray"""
        step_size = self.map_resolution / 2
        num_steps = int(max_range / step_size)
        
        for i in range(num_steps):
            distance = i * step_size
            point_x = robot_x + distance * math.cos(angle)
            point_y = robot_y + distance * math.sin(angle)
            
            grid_x, grid_y = self.world_to_grid(point_x, point_y)
            
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                if free_space and self.occupancy_grid[grid_y, grid_x] == -1:
                    self.occupancy_grid[grid_y, grid_x] = 0  # Free space
    
    def mark_point(self, robot_x, robot_y, angle, distance, occupied=True):
        """Mark a specific point as occupied"""
        point_x = robot_x + distance * math.cos(angle)
        point_y = robot_y + distance * math.sin(angle)
        
        grid_x, grid_y = self.world_to_grid(point_x, point_y)
        
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            if occupied:
                self.occupancy_grid[grid_y, grid_x] = 100  # Occupied
    
    def publish_map(self):
        """Publish the occupancy grid map"""
        msg = OccupancyGrid()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Map metadata
        msg.info = MapMetaData()
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        
        # Map origin (center the map)
        msg.info.origin = Pose()
        msg.info.origin.position.x = -(self.map_width * self.map_resolution) / 2
        msg.info.origin.position.y = -(self.map_height * self.map_resolution) / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert occupancy grid to message format
        msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    simple_mapper = SimpleMapper()
    
    try:
        rclpy.spin(simple_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        simple_mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()