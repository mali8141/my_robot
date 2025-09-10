#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math

class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriber to cmd_vel for manual control
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Timer
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50Hz
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Simple Odometry Publisher Started')
        self.get_logger().info('Use: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...')

    def cmd_vel_callback(self, msg):
        """Update velocity from cmd_vel"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def publish_odom(self):
        """Publish odometry and TF"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Update position based on velocity
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Create quaternion from yaw
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
        self.last_time = current_time

def main():
    rclpy.init()
    node = SimpleOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()