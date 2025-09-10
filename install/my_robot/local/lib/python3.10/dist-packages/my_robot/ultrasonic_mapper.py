#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
import math
from tf2_ros import TransformListener, Buffer, TransformException

class UltrasonicMapper(Node):
    """
    Node to create occupancy grid maps using 6 SRF08 ultrasonic sensors
    """
    
    def __init__(self):
        super().__init__('ultrasonic_mapper')
        
        # Parameters
        self.map_width = 200  # cells (20m at 0.1m resolution)
        self.map_height = 200  # cells
        self.map_resolution = 0.1  # meters per cell
        self.robot_x = self.map_width // 2
        self.robot_y = self.map_height // 2
        
        # SRF08 sensor specifications
        self.max_range = 6.0  # meters
        self.min_range = 0.03  # meters
        self.beam_width = 0.52  # ~30 degrees in radians
        
        # Sensor positions (angles in radians relative to robot front)
        self.sensor_angles = {
            's1_front': 0.0,           # Front
            's2_front_right': -1.047,   # 60° right
            's3_back_right': -2.094,    # 120° right  
            's4_back': 3.14159,         # Back (180°)
            's5_back_left': 2.094,      # 120° left
            's6_front_left': 1.047      # 60° left
        }
        
        # Initialize occupancy grid
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Subscribers for each ultrasonic sensor
        self.sensor_subscribers = {}
        for sensor_name in self.sensor_angles.keys():
            topic = f'/ultrasonic/{sensor_name}'
            self.sensor_subscribers[sensor_name] = self.create_subscription(
                LaserScan,
                topic,
                self.create_sensor_callback(sensor_name),
                10
            )
        
        # Publisher for occupancy grid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # TF2 listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to publish map
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        # Storage for latest sensor readings
        self.sensor_ranges = {}
        
        self.get_logger().info('Ultrasonic Mapper Node Started')
        self.get_logger().info(f'Subscribed to topics: {list(self.sensor_angles.keys())}')
    
    def create_sensor_callback(self, sensor_name):
        """Create a callback function for a specific sensor"""
        def callback(msg):
            self.sensor_callback(msg, sensor_name)
        return callback
    
    def sensor_callback(self, msg, sensor_name):
        """Process ultrasonic sensor data"""
        if len(msg.ranges) > 0:
            # Filter valid ranges
            valid_ranges = [r for r in msg.ranges if self.min_range < r < self.max_range]
            
            if valid_ranges:
                # Get the minimum range (closest obstacle)
                range_reading = min(valid_ranges)
                self.sensor_ranges[sensor_name] = range_reading
                self.get_logger().debug(f'{sensor_name}: {range_reading:.2f}m')
                self.update_map(sensor_name, range_reading)
            else:
                # No valid reading, possibly no obstacle in range
                self.get_logger().debug(f'No valid range for {sensor_name}')
    
    def update_map(self, sensor_name, range_reading):
        """Update occupancy grid based on sensor reading"""
        try:
            # Get robot pose in map frame
            robot_x, robot_y, robot_yaw = self.get_robot_pose()
            
            # Calculate sensor angle in global frame
            sensor_angle_global = robot_yaw + self.sensor_angles[sensor_name]
            
            # Mark free space along the beam
            self.mark_free_space(robot_x, robot_y, sensor_angle_global, range_reading)
            
            # Mark occupied space at the obstacle
            self.mark_occupied_space(robot_x, robot_y, sensor_angle_global, range_reading)
            
        except Exception as e:
            self.get_logger().debug(f'Could not update map: {e}')
    
    def get_robot_pose(self):
        """Get robot pose from TF tree"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            orientation = transform.transform.rotation
            
            # Manual quaternion to euler conversion (yaw only)
            x_q = orientation.x
            y_q = orientation.y
            z_q = orientation.z
            w_q = orientation.w
            
            # Calculate yaw from quaternion
            siny_cosp = 2 * (w_q * z_q + x_q * y_q)
            cosy_cosp = 1 - 2 * (y_q * y_q + z_q * z_q)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
            
        except TransformException as e:
            # If no TF available, use center of map
            self.get_logger().debug(f'Transform failed: {e}')
            return 0.0, 0.0, 0.0
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x / self.map_resolution) + (self.map_width // 2))
        grid_y = int((y / self.map_resolution) + (self.map_height // 2))
        return grid_x, grid_y
    
    def mark_free_space(self, robot_x, robot_y, angle, range_reading):
        """Mark free space along sensor beam"""
        # Step along the beam in small increments
        step_size = self.map_resolution / 2
        num_steps = int(range_reading / step_size)
        
        for i in range(num_steps):
            distance = i * step_size
            
            # Calculate point coordinates
            point_x = robot_x + distance * math.cos(angle)
            point_y = robot_y + distance * math.sin(angle)
            
            # Convert to grid coordinates
            grid_x, grid_y = self.world_to_grid(point_x, point_y)
            
            # Mark as free space if within bounds
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                if self.occupancy_grid[grid_y, grid_x] == -1:  # Unknown space
                    self.occupancy_grid[grid_y, grid_x] = 0  # Free space
    
    def mark_occupied_space(self, robot_x, robot_y, angle, range_reading):
        """Mark occupied space at obstacle location"""
        # Calculate obstacle position
        obstacle_x = robot_x + range_reading * math.cos(angle)
        obstacle_y = robot_y + range_reading * math.sin(angle)
        
        # Convert to grid coordinates
        grid_x, grid_y = self.world_to_grid(obstacle_x, obstacle_y)
        
        # Mark as occupied if within bounds
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            self.occupancy_grid[grid_y, grid_x] = 100  # Occupied space
            
            # Also mark neighboring cells due to beam width uncertainty
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = grid_x + dx, grid_y + dy
                    if (0 <= nx < self.map_width and 0 <= ny < self.map_height and 
                        self.occupancy_grid[ny, nx] != 100):
                        self.occupancy_grid[ny, nx] = 50  # Likely occupied
    
    def publish_map(self):
        """Publish the current occupancy grid map"""
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
        
        # Map origin (bottom-left corner)
        msg.info.origin = Pose()
        msg.info.origin.position.x = -(self.map_width * self.map_resolution) / 2
        msg.info.origin.position.y = -(self.map_height * self.map_resolution) / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten occupancy grid and convert to list
        flattened_grid = self.occupancy_grid.flatten()
        msg.data = flattened_grid.tolist()
        
        # Publish map
        self.map_publisher.publish(msg)
        
        # Log map statistics occasionally
        if hasattr(self, '_map_counter'):
            self._map_counter += 1
        else:
            self._map_counter = 1
            
        if self._map_counter % 10 == 0:  # Every 10 seconds
            occupied_cells = np.sum(self.occupancy_grid == 100)
            free_cells = np.sum(self.occupancy_grid == 0)
            unknown_cells = np.sum(self.occupancy_grid == -1)
            self.get_logger().info(
                f'Map stats - Occupied: {occupied_cells}, '
                f'Free: {free_cells}, Unknown: {unknown_cells}, '
                f'Sensors: {len(self.sensor_ranges)}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    ultrasonic_mapper = UltrasonicMapper()
    
    try:
        rclpy.spin(ultrasonic_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        ultrasonic_mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()