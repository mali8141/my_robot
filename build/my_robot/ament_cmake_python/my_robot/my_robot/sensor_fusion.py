#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Publisher for fused scan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer to publish scan data
        self.timer = self.create_timer(0.1, self.publish_fused_scan)
        
        self.get_logger().info('Sensor fusion with dummy data started')

    def publish_fused_scan(self):
        """Create LaserScan with simulated 6-sensor data"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'odom'
        
        # LaserScan parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.03
        scan.range_max = 6.0
        
        # Initialize with infinite ranges (no obstacles)
        num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_points
        
        # Simulate 6 ultrasonic sensors with realistic readings
        sensor_angles = {
            's1_front': 0,           # Front
            's2_front_right': -60,   # Front-right
            's3_back_right': -120,   # Back-right
            's4_back': 180,          # Back
            's5_back_left': 120,     # Back-left
            's6_front_left': 60      # Front-left
        }
        
        for sensor_name, angle_deg in sensor_angles.items():
            # Generate realistic distance readings (1-4 meters with noise)
            base_distance = 2.5
            noise = random.uniform(-0.3, 0.3)
            distance = max(0.1, base_distance + noise)  # Minimum 10cm
            
            # Convert angle to array index
            angle_rad = math.radians(angle_deg)
            angle_index = int((angle_rad - scan.angle_min) / scan.angle_increment)
            
            if 0 <= angle_index < len(scan.ranges):
                scan.ranges[angle_index] = distance
        
        self.scan_pub.publish(scan)

def main():
    rclpy.init()
    node = SensorFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()