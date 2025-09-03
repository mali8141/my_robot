#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publishers
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)
        
        # Current values
        self.steering = 0.0
        self.velocity = 0.0
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("\n=== Ackermann Robot Control ===")
        print("w/s: forward/backward")
        print("a/d: left/right")
        print("x: stop, q: quit")

    def get_key(self):
        """Get single keypress"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_commands(self):
        """Send commands to robot"""
        self.steering_pub.publish(Float64(data=self.steering))
        self.velocity_pub.publish(Float64(data=self.velocity))

    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':      # Forward
                    self.velocity = min(self.velocity + 0.2, 2.0)
                elif key == 's':    # Backward
                    self.velocity = max(self.velocity - 0.2, -2.0)
                elif key == 'a':    # Left - Send NEGATIVE (to make wheels turn left)
                    self.steering = max(self.steering - 0.1, -0.5)
                elif key == 'd':    # Right - Send POSITIVE (to make wheels turn right)
                    self.steering = min(self.steering + 0.1, 0.5)
                elif key == 'x':    # Stop
                    self.velocity = 0.0
                    self.steering = 0.0
                elif key == 'q':    # Quit
                    break
                
                self.publish_commands()
                print(f"Velocity: {self.velocity:.1f}, Steering: {self.steering:.1f}")
                
        except KeyboardInterrupt:
            pass
        finally:
            # Stop robot and restore terminal
            self.velocity = 0.0
            self.steering = 0.0
            self.publish_commands()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    teleop = KeyboardTeleop()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()