#!/usr/bin/env python3
import numpy as np
from sklearn.linear_model import RANSACRegressor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


def polar_to_cartesian(r: np.ndarray, theta: np.ndarray) -> np.ndarray:
    """
    Convert polar coordinates to cartesian coordinates
    Args:
        r: radius
        theta: angle
    Returns:
        x, y
    """
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y


def interpolate_wall_points(r: np.ndarray, theta: np.ndarray) -> np.ndarray:
    """
    Interpolate wall points
    Args:
        r: radius
        theta: angle
    Returns:
        x, y, dist
    """
    x, y = polar_to_cartesian(r, theta)
    # Reshape x for sklearn (n_samples, n_features).
    X = x.reshape(-1, 1)
    
    # Create and fit the RANSAC regressor.
    ransac = RANSACRegressor()
    ransac.fit(X, y)
    
    # Retrieve slope and intercept from the underlying estimator.
    m = ransac.estimator_.coef_[0]
    b = ransac.estimator_.intercept_
    
    # Compute the distance from the origin to the line y = mx + b.
    distance = np.abs(b) / np.sqrt(m**2 + 1)
    
    return m, b, distance


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("drive_topic", "drive_topic")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 0.5)
        self.declare_parameter("desired_distance", 1.0)

        # PID controller parameters
        self.declare_parameter("kp", 3)
        self.declare_parameter("ki", 0.1)
        self.declare_parameter("kd", 2)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        # TODO: Initialize your publishers and subscribers here
        self.scan_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        # a publisher for our line marker
        self.line_pub = self.create_publisher(Marker, "/wall", 1)

        # TODO: Write your callback functions here
        # Time increment for the scan.
        self.hz = 20

        # PID controller variables.
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        # Forward wall slope (m) control variable.
        self.km = 10
        self.turn_rad = 1.0

        # PID controller variable tracking.
        self.prev_error = 0
        self.integral_error = 0

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)

    def scan_callback(self, msg):
        # Calculates scan frequency.
        hz = 1/msg.time_increment if msg.time_increment > 0 else self.hz
        # Get the angles of the scan.
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        # Asserts angles and distances have the same length.
        distances = np.array(msg.ranges)
        assert len(distances) == len(angles), f"len(distances) = {len(distances)}, len(angles) = {len(angles)}"
        scan_polar_vectors = np.vstack((distances, angles))

        # Declares side filters for left and right angles.
        side_cut = 30
        side_min_angle = np.radians(90 - side_cut/2)
        side_max_angle = np.radians(90 + side_cut/2)

        def side_wall_filter(side):
            # Filters the angles based on the side.
            if side == 1:
                side_wall = scan_polar_vectors[:, scan_polar_vectors[1,:] >= 0]
            else:
                side_wall = scan_polar_vectors[:, scan_polar_vectors[1,:] <= 0]
            side_wall = side_wall[ # Uses abs for readability, could optimize.
                :,  (side_min_angle <= np.abs(side_wall[1,:])) & 
                    (np.abs(side_wall[1,:]) <= side_max_angle)
            ]

            # Filters based on lookahead distance.
            max_scan_dist = 2 * (1 + np.tan(side_wall[1,:])) * self.DESIRED_DISTANCE
            side_wall_len = len(side_wall[0,:])
            side_wall = side_wall[:, side_wall[0,:] <= max_scan_dist]

            return side_wall, side_wall_len

        left_wall, l_wall_len = side_wall_filter(1)
        right_wall, r_wall_len = side_wall_filter(-1)
        tracked_wall = left_wall if self.SIDE == 1 else right_wall
        t_wall_len = l_wall_len if self.SIDE == 1 else r_wall_len

        # If there are no points in the side wall, no PID + wall_loss.
        steering_angle = 0
        error = 0
        if tracked_wall[0].size >= 2:
            # Interpolates the wall points.
            m, b, dist = interpolate_wall_points(
                tracked_wall[0], tracked_wall[1]
            )

            # Calculates the error.
            error = dist - self.DESIRED_DISTANCE
            P = self.kp * error
            # Integral component.
            I = self.ki * self.integral_error
            # Calculates the derivative.
            D = self.kd * (self.prev_error - error) * hz

            # Calculates the steering angle.
            steering_angle = P + I - D

            # If half the wall disappeared start turning in the direction of the wall.
            kl = 2
            if len(tracked_wall[0]) < 0.5 * t_wall_len:
                self.get_logger().info(f"!!!!!!!!WALL LOST!!!!!!")
                steering_angle += kl * 2 * (0.5 - len(tracked_wall[0])/t_wall_len) * np.radians(90)

        # Gets the forward wall points.
        f_cut = 22.5
        # Introduces a correction to ignore a bit of the side we're following.
        forward_cutoff = np.clip(
            msg.angle_increment * 50,
            -np.radians(f_cut/2), np.radians(f_cut/2)
        )
        forward_wall = scan_polar_vectors[
            :,  (scan_polar_vectors[1,:] >= -forward_cutoff) & 
                (scan_polar_vectors[1,:] <= forward_cutoff)
        ]
        # Prunes points that are too far away.
        e_stop_dist = (self.DESIRED_DISTANCE + self.turn_rad) + self.VELOCITY/hz
        f_wall_pts = len(forward_wall[0,:])
        forward_wall = forward_wall[:,
            forward_wall[0,:] <= (1 + np.tan(forward_wall[1, :])) * (e_stop_dist + 2 * self.VELOCITY/hz)
        ]
        # If 60% of the degree range is empty, we are not facing a wall.
        if len(forward_wall[0]) < 0.5 * f_wall_pts:
            m = 0
            b = 0
            dist = np.inf
        else:
            # Interpolates the wall points.
            m, b, dist = interpolate_wall_points(
                forward_wall[0], forward_wall[1]
            )
        # If we are within lookahead of the wall in front, make this wall on our SIDE.
        # Also, checks for the angle of the forward wall towards us and adjust avoidance
        # accordingly.
        if dist < e_stop_dist:
            self.get_logger().info(f"!!!!!!!!AVOIDING COLLISION!!!!!!")
            # Calculates the steering angle.
            if dist - self.turn_rad > 0:
                steering_angle += -self.km * self.VELOCITY/(dist - self.turn_rad) * np.abs(m)
            else:
                steering_angle += -np.inf
            error = (dist - self.DESIRED_DISTANCE)

        # Updates the previous error.
        self.prev_error = error
        # Updates the integral error.
        self.integral_error += error / hz
        self.integral_error %= self.DESIRED_DISTANCE

        # Clips and gives direction to the steering angle.
        steering_angle = np.clip(self.SIDE * steering_angle, -np.radians(90), np.radians(90))

        # Publishes the drive message.
        # self.get_logger().info(f"Steering angle: {steering_angle}")
        self.drive(self.VELOCITY, steering_angle)


    def draw_wall(self, x, y):
        """
        Plots the wall in rviz
        """
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")


    def drive(self, speed, angle):
        """
        Publishes a drive message to the robot
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()

        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle

        self.drive_pub.publish(drive_msg)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    