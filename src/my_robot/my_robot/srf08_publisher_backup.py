#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
import math
import random

class SRF08Publisher(Node):
    def __init__(self):
        super().__init__('srf08_publisher')
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Individual SRF08 sensor publishers
        self.sensor_pubs = {}
        self.srf08_sensors = {
            's1_front': {'angle': 0, 'x': 0.19, 'y': 0.0},           # Front
            's2_front_right': {'angle': -60, 'x': 0.13, 'y': -0.075},  # Front-right
            's3_back_right': {'angle': -120, 'x': -0.13, 'y': -0.075}, # Back-right
            's4_back': {'angle': 180, 'x': -0.19, 'y': 0.0},         # Back
            's5_back_left': {'angle': 120, 'x': -0.13, 'y': 0.075},   # Back-left
            's6_front_left': {'angle': 60, 'x': 0.13, 'y': 0.075}     # Front-left
        }
        
        for name in self.srf08_sensors.keys():
            self.sensor_pubs[name] = self.create_publisher(Range, f'/ultrasonic/{name}', 10)
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # SRF08 sensor readings
        self.sensor_readings = {}
        for name in self.srf08_sensors.keys():
            self.sensor_readings[name] = 0.0
        
        # Timer - SRF08 typically runs at 10Hz
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.get_logger().info('SRF08 Ultrasonic Array Publisher Started - MAZE ENVIRONMENT')
        self.get_logger().info('Environment synchronized with maze.sdf')

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

    def simulate_srf08_reading(self, sensor_name):
        """Simulate SRF08 sensor reading with realistic characteristics"""
        sensor_info = self.srf08_sensors[sensor_name]
        
        # Sensor's global position
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        
        sensor_global_x = self.robot_x + (sensor_info['x'] * cos_yaw - sensor_info['y'] * sin_yaw)
        sensor_global_y = self.robot_y + (sensor_info['x'] * sin_yaw + sensor_info['y'] * cos_yaw)
        
        # Sensor's beam direction (global frame)
        sensor_angle_rad = math.radians(sensor_info['angle'])
        global_beam_angle = self.robot_yaw + sensor_angle_rad
        
        # Simulate environment distance matching maze.sdf
        min_distance = self.simulate_maze_environment(
            sensor_global_x, sensor_global_y, global_beam_angle)
        
        # SRF08-specific characteristics
        # 1. Quantize to 1cm resolution
        distance_cm = round(min_distance * 100)
        quantized_distance = distance_cm / 100.0
        
        # 2. Add SRF08 measurement noise (±1-2cm)
        noise = random.uniform(-0.02, 0.02)
        noisy_distance = quantized_distance + noise
        
        # 3. SRF08 range limits (3cm to 6m)
        final_distance = max(0.03, min(6.0, noisy_distance))
        
        # 4. Occasionally simulate no-echo
        if random.random() < 0.05:  # 5% chance
            if sensor_name in ['s1_front', 's4_back']:
                if random.random() < 0.3:
                    return 6.0
            else:
                return 6.0
        
        return final_distance

    def simulate_maze_environment(self, sensor_x, sensor_y, beam_angle):
        """Simulate environment matching the actual maze.sdf file"""
        
        cos_beam = math.cos(beam_angle)
        sin_beam = math.sin(beam_angle)
        
        # Define maze walls from maze.sdf file
        # Format: [x_center, y_center, width, height]
        maze_walls = [
            # Major walls from maze.sdf
            [0, 5.5, 2, 9],        # wall12: large central wall
            [0, -2.5, 2, 1],       # wall13: bottom center
            [-0.5, -1.5, 1, 1],    # wall14: small wall
            [3.5, -0.5, 1, 3],     # wall22: right side
            [7, 0.5, 6, 1],        # wall23: long horizontal
            [7.5, 2, 1, 2],        # wall24: vertical
            [7.5, 6, 1, 2],        # wall34: upper right
            [8.5, 7, 1, 2],        # wall35: far upper right
            [9.5, 8, 1, 2],        # wall36: corner
            
            # Boundary walls (assuming maze boundaries)
            [-10, 0, 1, 20],       # Left boundary
            [10, 0, 1, 20],        # Right boundary  
            [0, 10, 20, 1],        # Top boundary
            [0, -10, 20, 1],       # Bottom boundary
        ]
        
        min_distance = 6.0  # SRF08 max range
        
        # Check intersections with all walls
        for wall in maze_walls:
            wall_x, wall_y, wall_w, wall_h = wall
            
            # Wall boundaries
            wall_left = wall_x - wall_w/2
            wall_right = wall_x + wall_w/2
            wall_bottom = wall_y - wall_h/2
            wall_top = wall_y + wall_h/2
            
            # Check intersection with each wall edge
            intersections = []
            
            # Left edge (vertical line at wall_left)
            if abs(cos_beam) > 0.001:
                t = (wall_left - sensor_x) / cos_beam
                if t > 0:
                    y_intersect = sensor_y + t * sin_beam
                    if wall_bottom <= y_intersect <= wall_top:
                        intersections.append(t)
            
            # Right edge (vertical line at wall_right)
            if abs(cos_beam) > 0.001:
                t = (wall_right - sensor_x) / cos_beam
                if t > 0:
                    y_intersect = sensor_y + t * sin_beam
                    if wall_bottom <= y_intersect <= wall_top:
                        intersections.append(t)
            
            # Bottom edge (horizontal line at wall_bottom)
            if abs(sin_beam) > 0.001:
                t = (wall_bottom - sensor_y) / sin_beam
                if t > 0:
                    x_intersect = sensor_x + t * cos_beam
                    if wall_left <= x_intersect <= wall_right:
                        intersections.append(t)
            
            # Top edge (horizontal line at wall_top)
            if abs(sin_beam) > 0.001:
                t = (wall_top - sensor_y) / sin_beam
                if t > 0:
                    x_intersect = sensor_x + t * cos_beam
                    if wall_left <= x_intersect <= wall_right:
                        intersections.append(t)
            
            # Use closest intersection
            if intersections:
                closest_distance = min(intersections)
                min_distance = min(min_distance, closest_distance)
        
        return min_distance

    def publish_individual_sensors(self):
        """Publish individual Range messages for each SRF08 sensor"""
        for sensor_name in self.srf08_sensors.keys():
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = f'{sensor_name}_frame'
            
            # SRF08 specifications
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = math.radians(30)  # SRF08 beam width ~30°
            range_msg.min_range = 0.03  # 3cm minimum range
            range_msg.max_range = 6.0   # 6m maximum range
            
            # Get simulated SRF08 reading
            range_msg.range = self.simulate_srf08_reading(sensor_name)
            
            self.sensor_pubs[sensor_name].publish(range_msg)
            self.sensor_readings[sensor_name] = range_msg.range

    def publish_scan(self):
        """Convert SRF08 readings to LaserScan for SLAM"""
        # Update all SRF08 sensor readings
        self.publish_individual_sensors()
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        # LaserScan parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.03  # SRF08 minimum
        scan.range_max = 6.0   # SRF08 maximum
        
        # Initialize all ranges to infinity (no reading)
        num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_points
        
        # Map SRF08 sensors to LaserScan angles
        for sensor_name, sensor_info in self.srf08_sensors.items():
            angle_rad = math.radians(sensor_info['angle'])
            angle_index = int((angle_rad - scan.angle_min) / scan.angle_increment)
            
            if 0 <= angle_index < len(scan.ranges):
                scan.ranges[angle_index] = self.sensor_readings[sensor_name]
                
                # Fill in nearby points to simulate SRF08 beam width
                beam_spread = 15  # degrees
                for offset in range(-beam_spread, beam_spread + 1):
                    nearby_index = angle_index + offset
                    if 0 <= nearby_index < len(scan.ranges):
                        adjusted_range = self.sensor_readings[sensor_name] * (1 + abs(offset) * 0.02)
                        if scan.ranges[nearby_index] == float('inf'):
                            scan.ranges[nearby_index] = min(adjusted_range, scan.range_max)
        
        self.scan_pub.publish(scan)
        
        # Debug info every 50 cycles (5 seconds at 10Hz)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 50 == 0:
            self.get_logger().info(
                f'MAZE SRF08 Readings - Front: {self.sensor_readings["s1_front"]:.2f}m, '
                f'Back: {self.sensor_readings["s4_back"]:.2f}m, '
                f'Left: {self.sensor_readings["s6_front_left"]:.2f}m, '
                f'Right: {self.sensor_readings["s2_front_right"]:.2f}m'
            )

def main():
    rclpy.init()
    node = SRF08Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()