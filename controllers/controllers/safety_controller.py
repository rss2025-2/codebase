#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# Constant: the cone width (in degrees) to consider the central region of the lidar
CENTER_ANGLE_DEG = 30  # use center 30°: ±15° around 0 radians

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')
        # Declare ROS parameters
        self.declare_parameter("safety_cutoff_distance", 0.5)
        self.declare_parameter("forward_message", True)
        self.declare_parameter("safety_time", 0.01)  # period (in seconds) for safety timer callback
        
        # Get parameter values
        self.safety_cutoff_distance = self.get_parameter("safety_cutoff_distance").get_parameter_value().double_value
        self.forward_message = self.get_parameter("forward_message").get_parameter_value().bool_value
        self.safety_time = self.get_parameter("safety_time").get_parameter_value().double_value

        # Create publishers and subscribers
        self.input_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            "drive_input_topic",
            self.input_drive_callback,
            10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan_topic",
            self.scan_callback,
            10)
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped,
            "safety_drive_topic",
            10)
        
        # Member variables for storing latest drive command and latest lidar safety reading  
        self.last_drive_msg = None
        self.min_scan_range = None  # will store the minimum distance measured in the center cone
        
        # Safety check timer: periodically check if we should override drive commands.
        self.safety_timer = self.create_timer(
            self.safety_time,
            self.safety_callback)

    def scan_callback(self, scan_msg):
        """
        Process the latest LaserScan message.
        For each scan, look at only the measurements that lie within the central cone (±15° around zero)
        and record the smallest range value.
        """
        half_cone = (CENTER_ANGLE_DEG / 2.0) * (math.pi / 180.0)
        min_range = float('inf')
        for i, r in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            if -half_cone <= angle <= half_cone:
                # Only consider valid readings (positive range values)
                if r > 0 and r < min_range:
                    min_range = r
        # If no valid measurement is found, set to None so that safety_callback knows to wait.
        self.min_scan_range = min_range if min_range != float('inf') else None

    def input_drive_callback(self, drive_msg):
        """
        Simply store the last drive command and, if no immediate obstacle is detected,
        forward the incoming drive command.
        """
        self.last_drive_msg = drive_msg
        if self.forward_message:
            # If no obstacle is detected (or scan has not yet produced a valid minimum), publish the incoming command.
            if self.min_scan_range is None or self.min_scan_range > self.safety_cutoff_distance:
                self.safety_pub.publish(drive_msg)

    def safety_callback(self):
        """
        Check whether the minimum range in the central cone is less than the safety cutoff.
        If so, publish a safe (stop) command.
        """
        if self.min_scan_range is None:
            return  # No valid range data yet
        if self.min_scan_range < self.safety_cutoff_distance:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            safe_msg = AckermannDriveStamped(
                header=header,
                drive=AckermannDrive(speed=0.0))
            self.safety_pub.publish(safe_msg)
            self.get_logger().info(
                f"Safety engaged: obstacle detected at {self.min_scan_range:.2f} m (< {self.safety_cutoff_distance} m). Stopping.")

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
