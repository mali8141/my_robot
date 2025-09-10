#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import math

class SensorFusion(Node):
    """
    Node to fuse 6 SRF08 ultrasonic sensors into a single LaserScan message
    """
    
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # SRF08 sensor specifications
        self.max_range = 6.0  # meters
        self.min_range = 0.03  # meters
        
        # Sensor positions (angles in radians relative to robot front)
        self.sensor_angles = {
            's1_front': 0.0,           # Front (0°)
            's2_front_right': -1.047,   # 60° right
            's3_back_right': -2.094,    # 120° right  
            's4_back': 3.14159,         # Back (180°)
            's5_back_left': 2.094,      # 120° left
            's6_front_left': 1.047      # 60° left
        }
        
        # Create synthetic 360° laser scan (360 points = 1° resolution)
        self.scan_points = 360
        self.angle_min = -math.pi  # -180°
        self.angle_max = math.pi   # +180°
        self.angle_increment = (self.angle_max - self.angle_min) / self.scan_points
        
        # Initialize ranges array
        self.ranges = np.full(self.scan_points, self.max_range, dtype=np.float32)
        
        # Storage for latest sensor readings
        self.sensor_ranges = {}
        
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
        
        # Publisher for fused laser scan
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer to publish fused scan
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10Hz
        
        self.get_logger().info('Sensor Fusion Node Started')
        self.get_logger().info(f'Fusing {len(self.sensor_angles)} ultrasonic sensors into /scan topic')
    
    def create_sensor_callback(self, sensor_name):
        """Create a callback function for a specific sensor"""
        def callback(msg):
            self.sensor_callback(msg, sensor_name)
        return callback
    
    def sensor_callback(self, msg, sensor_name):
        """Process individual sensor data"""
        if len(msg.ranges) > 0:
            # Filter valid ranges
            valid_ranges = [r for r in msg.ranges if self.min_range < r < self.max_range]
            
            if valid_ranges:
                # Get the minimum range (closest obstacle)
                range_reading = min(valid_ranges)
                self.sensor_ranges[sensor_name] = range_reading
                self.update_fused_scan(sensor_name, range_reading)
            else:
                # No valid reading - set to max range
                self.sensor_ranges[sensor_name] = self.max_range
                self.update_fused_scan(sensor_name, self.max_range)
    
    def update_fused_scan(self, sensor_name, range_reading):
        """Update the fused laser scan with sensor data"""
        # Get sensor angle
        sensor_angle = self.sensor_angles[sensor_name]
        
        # Convert sensor angle to scan array index
        # Normalize angle to [-pi, pi]
        while sensor_angle > math.pi:
            sensor_angle -= 2 * math.pi
        while sensor_angle < -math.pi:
            sensor_angle += 2 * math.pi
        
        # Convert angle to array index
        angle_index = int((sensor_angle - self.angle_min) / self.angle_increment)
        
        # Ensure index is within bounds
        if 0 <= angle_index < self.scan_points:
            # Update the main direction
            self.ranges[angle_index] = range_reading
            
            # Also update neighboring points to account for beam width (~30°)
            beam_spread = int(0.26 / self.angle_increment)  # 30° in array indices
            
            for i in range(-beam_spread, beam_spread + 1):
                neighbor_index = angle_index + i
                if 0 <= neighbor_index < self.scan_points:
                    # Only update if this reading is closer
                    if range_reading < self.ranges[neighbor_index]:
                        self.ranges[neighbor_index] = range_reading
    
    def publish_scan(self):
        """Publish the fused laser scan"""
        msg = LaserScan()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Laser scan parameters
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 10Hz
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        
        # Range data
        msg.ranges = self.ranges.tolist()
        msg.intensities = []  # Empty for ultrasonic sensors
        
        # Publish
        self.scan_publisher.publish(msg)
        
        # Debug info every 50 scans (5 seconds)
        if hasattr(self, '_scan_counter'):
            self._scan_counter += 1
        else:
            self._scan_counter = 1
            
        if self._scan_counter % 50 == 0:
            active_sensors = len([r for r in self.sensor_ranges.values() if r < self.max_range])
            min_range = min(self.ranges) if len(self.ranges) > 0 else self.max_range
            self.get_logger().info(
                f'Fused scan: {active_sensors}/{len(self.sensor_angles)} sensors active, '
                f'min range: {min_range:.2f}m'
            )

def main(args=None):
    rclpy.init(args=args)
    
    sensor_fusion = SensorFusion()
    
    try:
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()