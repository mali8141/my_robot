#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from tf2_ros import TransformListener, Buffer
import math

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Individual sensor publishers (for debugging)
        self.sensor_pubs = {}
        sensor_names = ['s1_front', 's2_front_right', 's3_back_right', 
                       's4_back', 's5_back_left', 's6_front_left']
        
        for name in sensor_names:
            self.sensor_pubs[name] = self.create_publisher(Range, f'/ultrasonic/{name}', 10)
        
        # Sensor data
        self.sensor_readings = {}
        for name in sensor_names:
            self.sensor_readings[name] = 0.0
        
        # Timer
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('Ultrasonic publisher with 6 SRF08 sensors started')

    def get_sensor_reading(self, sensor_name):
        """Get reading from specific sensor - REPLACE WITH ACTUAL I2C DATA"""
        # TODO: Replace with actual SRF08 I2C communication
        # For now, return dummy data
        import random
        return 1.0 + random.uniform(-0.5, 0.5)  # Random between 0.5-1.5m

    def publish_individual_sensors(self):
        """Publish individual Range messages for each sensor"""
        for sensor_name in self.sensor_readings.keys():
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = f'{sensor_name}_link'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26  # ~15 degrees for SRF08
            range_msg.min_range = 0.03      # 3cm
            range_msg.max_range = 6.0       # 6m
            range_msg.range = self.get_sensor_reading(sensor_name)
            
            self.sensor_pubs[sensor_name].publish(range_msg)
            self.sensor_readings[sensor_name] = range_msg.range

    def publish_scan(self):
        """Convert ultrasonic readings to LaserScan"""
        # Update sensor readings
        self.publish_individual_sensors()
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        # Scan parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi/180  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.03
        scan.range_max = 6.0
        
        # Initialize ranges
        num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_points
        
        # Sensor positions (angles in radians)
        sensor_angles = {
            's1_front': 0,           # 0°
            's2_front_right': -1.047, # -60°
            's3_back_right': -2.094,  # -120°
            's4_back': math.pi,       # 180°
            's5_back_left': 2.094,    # 120°
            's6_front_left': 1.047    # 60°
        }
        
        # Map sensor readings to scan
        for sensor_name, angle in sensor_angles.items():
            angle_index = int((angle - scan.angle_min) / scan.angle_increment)
            if 0 <= angle_index < len(scan.ranges):
                scan.ranges[angle_index] = self.sensor_readings[sensor_name]
        
        self.scan_pub.publish(scan)

def main():
    rclpy.init()
    node = UltrasonicPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()