#!/usr/bin/env python3
"""
Refactored parking_controller.py
"""

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

# Constants
DRIVE_PUB_QUEUE_SIZE = 10
ERROR_PUB_QUEUE_SIZE = 10
CONE_SUB_QUEUE_SIZE = 1

PARKING_DISTANCE = 0.75

KP_SPEED = 0.5
KP_STEER = 1.0

MAX_SPEED_COMMAND = 1.0
MIN_SPEED_COMMAND = -1.0

ANGLE_THRESHOLD = np.pi / 4
DISTANCE_ERROR_THRESHOLD = 0.5

BACKUP_TIMEOUT = 3.0
BACKUP_SPEED = -0.5

# Helper function for clamping a value.
def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class ParkingController(Node):
    def __init__(self):
        super().__init__("parking_controller")

        # Get drive topic from parameter and setup publishers and subscriptions.
        self.declare_parameter("drive_topic")
        drive_topic = self.get_parameter("drive_topic").value

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, DRIVE_PUB_QUEUE_SIZE)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", ERROR_PUB_QUEUE_SIZE)
        self.create_subscription(ConeLocation, "/relative_cone", self.relative_cone_callback, CONE_SUB_QUEUE_SIZE)

        # State variables
        self.relative_x = 0.0
        self.relative_y = 0.0
        self.current_state = "DRIVE_FORWARD"
        self.backup_start_time = None

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        # Update relative cone position.
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        # Calculate distance and angle errors.
        distance, distance_error, angle_error = self._compute_errors(self.relative_x, self.relative_y)
        self.get_logger().info(f"x: {self.relative_x} y: {self.relative_y} angle error: {angle_error}")

        # Determine commands depending on current state.
        if self.current_state == "DRIVE_FORWARD":
            speed_command, steering_command = self._drive_forward(distance_error, angle_error)
        elif self.current_state == "BACKUP":
            speed_command, steering_command = self._backup(angle_error)
        else:
            # Default safe commands.
            speed_command = 0.0
            steering_command = 0.0

        # Publish the drive command.
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = speed_command
        drive_cmd.drive.steering_angle = steering_command
        self.drive_pub.publish(drive_cmd)

        # Publish error.
        self._publish_error()

    def _compute_errors(self, x, y):
        """Return (distance, distance_error, angle_error) given relative cone coordinates."""
        distance = np.hypot(x, y)
        distance_error = distance - PARKING_DISTANCE
        angle_error = np.arctan2(y, x)
        return distance, distance_error, angle_error

    def _drive_forward(self, distance_error, angle_error):
        """Generate drive commands for the DRIVE_FORWARD state."""
        speed_cmd = KP_SPEED * distance_error
        speed_cmd = clamp(speed_cmd, MIN_SPEED_COMMAND, MAX_SPEED_COMMAND)
        steering_cmd = KP_STEER * angle_error

        # If the cone is too far off the center, switch to backup
        if abs(angle_error) > ANGLE_THRESHOLD:
            self.current_state = "BACKUP"
            self.backup_start_time = self.get_clock().now()

        # Stop if we are close enough.
        if distance_error < DISTANCE_ERROR_THRESHOLD:
            speed_cmd = 0.0
            steering_cmd = 0.0

        return speed_cmd, steering_cmd

    def _backup(self, angle_error):
        """Generate drive commands for the BACKUP state."""
        current_time = self.get_clock().now()
        if self.backup_start_time is None:
            self.backup_start_time = current_time

        # Calculate elapsed time in seconds.
        elapsed_time = (current_time - self.backup_start_time).nanoseconds / 1e9

        speed_cmd = BACKUP_SPEED
        steering_cmd = -KP_STEER * angle_error  # reverse steer correction

        # If backup period elapsed, switch back to drive forward.
        if elapsed_time >= BACKUP_TIMEOUT:
            self.current_state = "DRIVE_FORWARD"

        return speed_cmd, steering_cmd

    def _publish_error(self):
        """Publish the current error message."""
        error_msg = ParkingError()
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.hypot(self.relative_x, self.relative_y)
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    parking_controller = ParkingController()
    rclpy.spin(parking_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
