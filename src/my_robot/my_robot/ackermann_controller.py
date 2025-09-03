#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import TwistStamped
import math
import time


class AckermannController(Node):
    def __init__(self, timer_period=0.01, timeout_duration=1e9):
        super().__init__("ackermann_controller")

        # Declaring the parameters for the Ackermann controller
        self.declare_parameter("body_length", 0.42)
        self.declare_parameter("body_width", 0.17)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_length", 0.045)
        self.declare_parameter("wheel_connector_length", 0.1)
        self.declare_parameter("max_steering_angle", 0.5236)
        self.declare_parameter("max_velocity", 2.0)

        # Get parameters when the Node is launched
        self.body_length = self.get_parameter("body_length").get_parameter_value().double_value
        self.body_width = self.get_parameter("body_width").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_length = self.get_parameter("wheel_length").get_parameter_value().double_value
        self.wheel_connector_length = self.get_parameter("wheel_connector_length").get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value

        self.track_width = self.body_width + self.wheel_connector_length
        self.wheel_base = self.body_length - (2 * 0.05)

        self.timeout_duration = timeout_duration
        self.last_steering_time = self.get_clock().now()
        self.last_velocity_time = self.get_clock().now()

        self.steering_angle = 0.0
        self.velocity = 0.0
        self.wheel_steering_angle = [0.0, 0.0]  # [left, right]
        self.wheel_angular_velocity = [0.0, 0.0]  # [left, right]

        # Subscribers for steering angle and velocity commands
        self.steering_angle_subscriber = self.create_subscription(
            Float64, "/steering_angle", self.steering_angle_callback, 10
        )
        self.velocity_subscriber = self.create_subscription(Float64, "/velocity", self.velocity_callback, 10)

        # Publisher for the wheel commands
        self.position_publisher = self.create_publisher(Float64MultiArray, "/front_steering_controller/commands", 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, "/rear_velocity_controller/commands", 10)

        # Timer to periodically check for commands and publish wheel commands
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Ackermann Controller Node has been started.")

    def ackermann_steering_angle(self):
        """Calculates the angles for the front left and right wheels based on the input
        steering angle. The vehicle's physical dimensions such as wheelbase and track width
        are used in steering calculations.
        It returns the steering angles for the left and right front wheels."""
        left = 0
        right = 0
        if abs(self.steering_angle) > 1e-5:
            sin_angle = math.sin(abs(self.steering_angle))
            cos_angle = math.cos(abs(self.steering_angle))

            denom_left = 2 * self.wheel_base * cos_angle - self.track_width * sin_angle
            denom_right = 2 * self.wheel_base * cos_angle + self.track_width * sin_angle
            if self.steering_angle > 0:
                left = math.atan2(2 * self.wheel_base * sin_angle, denom_left)
                right = math.atan2(2 * self.wheel_base * sin_angle, denom_right)
            else:
                left = -math.atan2(2 * self.wheel_base * sin_angle, denom_right)
                right = -math.atan2(2 * self.wheel_base * sin_angle, denom_left)
            return [left, right]
        else:
            return [0.0, 0.0]

    def rear_differential_velocity(self):
        """Calculates the rear differential velocity based on ackermann steering geometry.
        For a straight line, the rear wheels should have the same velocity.
        For a turn, the inner wheel will have a lower velocity than the outer wheel.
        It returns the velocities for the left and right rear wheels."""
        left = right = self.velocity
        sa = self.steering_angle
        if abs(sa) > 1e-5:
            radius = self.wheel_base / math.tan(abs(sa))
            omega = self.velocity / radius

            inner = radius - self.track_width / 2
            outer = radius + self.track_width / 2

            if sa > 0:
                left = omega * inner
                right = omega * outer
            else:
                left = omega * outer
                right = omega * inner

            max_v = max(abs(left), abs(right))
            if max_v > self.max_velocity:
                scale = self.max_velocity / max_v
                left *= scale
                right *= scale
        return [left / self.wheel_radius, right / self.wheel_radius]

    def steering_angle_callback(self, msg):
        """Callback function for receiving steering angle messages.
        This function processes the incoming steering angle and updates the controller state.
        """
        self.last_steering_time = self.get_clock().now()
        angle = max(min(msg.data, self.max_steering_angle), -self.max_steering_angle)
        self.steering_angle = angle
        self.wheel_steering_angle = self.ackermann_steering_angle()

    def velocity_callback(self, msg):
        """Callback function for receiving velocity messages.
        This function processes the incoming velocity command and updates the controller state.
        """
        self.last_velocity_time = self.get_clock().now()
        vel = max(min(msg.data, self.max_velocity), -self.max_velocity)
        self.velocity = vel
        self.wheel_angular_velocity = self.rear_differential_velocity()

    def timer_callback(self):
        """Checks if no desired steering angle and velocity commands have been received.
        If not, it publishes a stop command to the vehicle by setting the steering angles
        and angular velocities to zero. Also publishes the current steering angles and wheel velocities."""
        now = self.get_clock().now()
        if (now - self.last_steering_time).nanoseconds > self.timeout_duration:
            self.wheel_steering_angle = [0.0, 0.0]
        if (now - self.last_velocity_time).nanoseconds > self.timeout_duration:
            self.wheel_angular_velocity = [0.0, 0.0]

        pos_msg = Float64MultiArray(data=self.wheel_steering_angle)
        vel_msg = Float64MultiArray(data=self.wheel_angular_velocity)
        self.position_publisher.publish(pos_msg)
        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()