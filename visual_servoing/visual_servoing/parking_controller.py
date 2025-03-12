#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        # Desired parking distance (in meters). Adjust this so the robot is about 1.5-2ft away.
        self.parking_distance = 0.75
        
        # current cone values (relative to base_link)
        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        # Update our stored relative cone position (assumed: x = lateral, y = forward)
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        
        # Create a drive command message
        drive_cmd = AckermannDriveStamped()

        # Compute the Euclidean distance from the robot to the cone
        distance = np.sqrt(self.relative_x**2 + self.relative_y**2)
        # Compute error relative to our desired parking spacing.
        distance_error = distance - self.parking_distance

        # Compute an angular correction using arctan; if the cone is off center, we want to steer to compensate.
        # (We use arctan2 to properly handle the sign.)
        angle_error = np.arctan2(self.relative_x, self.relative_y)

        # Controller gains, which you may want to tune.
        kp_speed = 0.5  # speed gain – scales how fast we move as a function of distance error
        kp_steer = 1.0  # steering gain – scales the turning angle to keep the cone centered

        # Compute the desired speed command.
        # Note: if the robot is too far, distance_error > 0 so we drive forward;
        # if too close, distance_error < 0 so we drive in reverse.
        speed_command = kp_speed * distance_error

        # Limit the command speed to be within safe bounds:
        if speed_command > 1.0:
            speed_command = 1.0
        elif speed_command < -1.0:
            speed_command = -1.0

        # Compute the steering angle command from the angular error.
        steering_command = kp_steer * angle_error

        # Optional: if you are nearly at the desired distance, stop and center your steering.
        if abs(distance_error) < 0.05:
            speed_command = 0.0
            steering_command = 0.0

        # Populate the drive command message.
        drive_cmd.drive.speed = speed_command
        drive_cmd.drive.steering_angle = steering_command

        # Publish our desired drive command.
        self.drive_pub.publish(drive_cmd)

        # Publish error information for plotting.
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will use rqt_plot to view these quantities.
        """
        error_msg = ParkingError()

        # Fill in the error message with the current x & y errors as well as the distance error
        # Note: The "distance" here is the raw Euclidean distance from the robot to the cone.
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.dist_error = np.sqrt(self.relative_x**2 + self.relative_y**2)

        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
