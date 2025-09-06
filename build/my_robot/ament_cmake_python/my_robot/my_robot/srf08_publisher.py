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
        self.get_logger().info('SRF08 Ultrasonic Array Publisher Started')
        self.get_logger().info('Simulating 6 SRF08 sensors with realistic characteristics')

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
        
        # Simulate environment with walls and obstacles
        min_distance = self.simulate_environment_distance(
            sensor_global_x, sensor_global_y, global_beam_angle)
        
        # SRF08-specific characteristics
        # 1. Quantize to 1cm resolution (SRF08 characteristic)
        distance_cm = round(min_distance * 100)  # Convert to cm and round
        quantized_distance = distance_cm / 100.0  # Convert back to meters
        
        # 2. Add SRF08 measurement noise (±1-2cm typical)
        noise = random.uniform(-0.02, 0.02)
        noisy_distance = quantized_distance + noise
        
        # 3. SRF08 range limits (3cm to 6m)
        final_distance = max(0.03, min(6.0, noisy_distance))
        
        # 4. Occasionally simulate no-echo (SRF08 returns 0 when no echo)
        # This happens with soft surfaces or out-of-range targets
        if random.random() < 0.05:  # 5% chance of no echo
            if sensor_name in ['s1_front', 's4_back']:  # Less likely for front/back
                if random.random() < 0.3:
                    return 6.0  # Return max range for no echo
            else:
                return 6.0  # Return max range for no echo
        
        return final_distance

    def simulate_environment_distance(self, sensor_x, sensor_y, beam_angle):
        """Simulate realistic environment for SRF08 sensor"""
        
        # Define a test environment (room with obstacles)
        # Room boundaries: 8m x 8m room
        room_size = 4.0  # ±4m from origin
        
        cos_beam = math.cos(beam_angle)
        sin_beam = math.sin(beam_angle)
        
        # Calculate distance to walls
        wall_distances = []
        
        # Check X walls
        if abs(cos_beam) > 0.001:
            if cos_beam > 0:  # Positive X direction
                dist_to_wall = (room_size - sensor_x) / cos_beam
            else:  # Negative X direction  
                dist_to_wall = (-room_size - sensor_x) / cos_beam
            if dist_to_wall > 0:
                wall_distances.append(dist_to_wall)
        
        # Check Y walls
        if abs(sin_beam) > 0.001:
            if sin_beam > 0:  # Positive Y direction
                dist_to_wall = (room_size - sensor_y) / sin_beam
            else:  # Negative Y direction
                dist_to_wall = (-room_size - sensor_y) / sin_beam
            if dist_to_wall > 0:
                wall_distances.append(dist_to_wall)
        
        min_wall_distance = min(wall_distances) if wall_distances else 6.0
        
        # Add some obstacles in the environment
        obstacles = [
            {'x': 1.5, 'y': 1.5, 'radius': 0.3},   # Cylindrical obstacle 1
            {'x': -2.0, 'y': -1.0, 'radius': 0.25}, # Cylindrical obstacle 2
            {'x': 0.5, 'y': -2.5, 'radius': 0.4},  # Cylindrical obstacle 3
        ]
        
        min_obstacle_distance = 6.0
        
        for obstacle in obstacles:
            # Vector from sensor to obstacle center
            dx = obstacle['x'] - sensor_x
            dy = obstacle['y'] - sensor_y
            
            # Distance to obstacle center
            center_distance = math.sqrt(dx*dx + dy*dy)
            
            if center_distance > 0.001:  # Avoid division by zero
                # Angle to obstacle center
                obstacle_angle = math.atan2(dy, dx)
                
                # Check if obstacle is within SRF08 beam cone (±15 degrees)
                angle_diff = abs(beam_angle - obstacle_angle)
                if angle_diff > math.pi:
                    angle_diff = 2*math.pi - angle_diff
                
                # SRF08 has ~30-degree beam width (±15 degrees)
                if angle_diff < math.radians(15):
                    # Calculate distance to obstacle surface
                    surface_distance = center_distance - obstacle['radius']
                    if surface_distance > 0:
                        min_obstacle_distance = min(min_obstacle_distance, surface_distance)
        
        return min(min_wall_distance, min_obstacle_distance)

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
                # SRF08 has ~30° beam, so fill ±15° around main direction
                beam_spread = 15  # degrees
                for offset in range(-beam_spread, beam_spread + 1):
                    nearby_index = angle_index + offset
                    if 0 <= nearby_index < len(scan.ranges):
                        # Slightly increase range for off-axis readings
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
                f'SRF08 Readings - Front: {self.sensor_readings["s1_front"]:.2f}m, '
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