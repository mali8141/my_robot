#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class XboxController(Node):
    def __init__(self):
        super().__init__('xbox_controller')
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publish to robot
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)
        
        print("Xbox Controller Ready!")
        print("Left stick: steering and velocity")
        print("Hold Right Bumper (RB) to enable")

    def joy_callback(self, msg):
        # Xbox controller mapping
        velocity = msg.axes[1] * 2.0    # Left stick Y * max speed
        steering = msg.axes[0] * 0.5    # Left stick X * max angle
        enable = msg.buttons[5]         # Right bumper
        
        # Only move if RB is pressed
        if enable:
            self.velocity_pub.publish(Float64(data=velocity))
            self.steering_pub.publish(Float64(data=steering))
        else:
            self.velocity_pub.publish(Float64(data=0.0))
            self.steering_pub.publish(Float64(data=0.0))

def main():
    rclpy.init()
    node = XboxController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()