#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header
import cv2
import numpy as np

class KeyboardController(Node):
    def __init__(self):
        super().__init__("keyboard_controller")

        # declare parameters
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_steering_angle", 2.0)
        self.declare_parameter("keypress_time", 0.001)
        self.declare_parameter("window_size", 100)
        # self.declare_parameter("persistent", True)

        # get parameter values
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.keypress_time = self.get_parameter("keypress_time").get_parameter_value().double_value
        self.window_size = self.get_parameter("window_size").get_parameter_value().integer_value
        # self.persistent = self.get_parameter("persistent").get_parameter_value().bool_value

        # create drive command publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "keyboard_drive_topic", 10)

        # open opencv window to detect keypresses
        cv2.imshow('Keyboard Controller',np.zeros((self.window_size,self.window_size,3), np.uint8))

        # create keypress detector timer callback
        self.key_timer = self.create_timer(self.keypress_time, self.key_detector_callback)

    def key_detector_callback(self):
        key = cv2.waitKey(0)
        header = Header(stamp = self.get_clock().now().to_msg())
        if key == ord('w'):
            # forward
            self.drive_pub.publish(AckermannDriveStamped(header = header, 
                                                         drive = AckermannDrive(steering_angle = 0.0, 
                                                                                speed = self.max_speed)))
        elif key == ord('q'):
            # half left
            self.drive_pub.publish(AckermannDriveStamped(header = header, 
                                                         drive = AckermannDrive(steering_angle = self.max_steering_angle / 2.0, 
                                                                                speed = self.max_speed)))
        elif key == ord('e'):
            # half right
            self.drive_pub.publish(AckermannDriveStamped(header = header, 
                                                         drive = AckermannDrive(steering_angle = -self.max_steering_angle / 2.0, 
                                                                                speed = self.max_speed)))
        elif key == ord('a'):
            # hard left
            self.drive_pub.publish(AckermannDriveStamped(header = header, 
                                                         drive = AckermannDrive(steering_angle = self.max_steering_angle, 
                                                                                speed = self.max_speed)))
        elif key == ord('d'):
            # hard right
            self.drive_pub.publish(AckermannDriveStamped(header = header, 
                                                         drive = AckermannDrive(steering_angle = -self.max_steering_angle, 
                                                                                speed = self.max_speed)))
        elif key == ord('s'):
            # stop
            self.drive_pub.publish(AckermannDriveStamped(header = header, 
                                                         drive = AckermannDrive(speed = 0.0)))

def main():
    rclpy.init()
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()