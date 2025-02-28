#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # declare parameters
        self.declare_parameter("safety_cutoff_distance", 0.5)
        self.declare_parameter("danger_zone_points_thres", 10)
        self.declare_parameter("max_steering_angle", 2.0)
        self.declare_parameter("max_deceleration", 10.0)
        self.declare_parameter("max_danger_velocity", 1.0)
        self.declare_parameter("car_length", 1.0)

        # get parameter values
        self.safety_cutoff_distance = self.get_parameter("safety_cutoff_distance").get_parameter_value().double_value
        self.danger_zone_points_thres = self.get_parameter("danger_zone_points_thres").get_parameter_value().integer_value
        self.max_steering_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.max_deceleration = self.get_parameter("max_deceleration").get_parameter_value().double_value
        self.max_danger_velocity = self.get_parameter("max_danger_velocity").get_parameter_value().double_value
        self.car_length = self.get_parameter("car_length").get_parameter_value().double_value

        # create publishers/subscribers
        self.input_drive_sub = self.create_subscription(AckermannDriveStamped, "drive_input_topic", self.input_drive_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan_topic", self.scan_callback, 10)
        self.safety_pub = self.create_publisher(AckermannDriveStamped, "safety_drive_topic", 10)

        # member variables
        self.latest_scan = LaserScan()

    def check_danger(self, drive_msg):
        """
        Given a drive message, check if the number of points within the potential trajectories,
        including the cutoff distance, is larger than a certrain threshold
        """
        # predict the trajectory of the car given the message received
        # compute stopping distance using command speed and max deceleration
        # TODO: filter the points of the latest scan within the cutoff distance from the predicted trajectory (computed using the max acceleration),
        # count them, and if they are above a certain threshold the car is in danger
        stop_dist = (drive_msg.drive.speed**2)/(2*self.max_deceleration)
        if abs(drive_msg.drive.steering_angle) < 0.001:
            # just assume it's going straight to not have to deal with huge turning radiuses
            pass
        else:
            turning_radius = self.car_length / math.tan(drive_msg.drive.steering_angle) # positive if turning left, negative if right
            turn_angle = stop_dist / turning_radius

    def input_drive_callback(self, in_drive_msg):
        # check if the car is in danger given the message received
        if self.check_danger(in_drive_msg):
            # check if taking the sharpest turn possible will make the car avoid the danger (with the same parameters, just limit the velocity)
            max_left_msg = AckermannDriveStamped() # TODO
            max_right_msg = AckermannDriveStamped() # TODO
            stop_msg = AckermannDriveStamped() # TODO
            if in_drive_msg.drive.steering_angle > 0: # was going left
                if not self.check_danger(max_left_msg): # prioritize left
                    self.safety_pub.publish(max_left_msg)
                elif not self.check_danger(max_right_msg):
                    self.safety_pub.publish(max_right_msg)
                else:
                    self.safety_pub.publish(stop_msg)
            else: # was going right
                if not self.check_danger(max_right_msg): # prioritize left
                    self.safety_pub.publish(max_right_msg)
                elif not self.check_danger(max_left_msg):
                    self.safety_pub.publish(max_left_msg)
                else:
                    self.safety_pub.publish(stop_msg)


    def scan_callback(self, scan_msg):
        self.latest_scan = scan_msg

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()