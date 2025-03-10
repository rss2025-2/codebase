#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import math
import numpy as np
from controllers.visualization_tools import VisualizationTools

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # declare parameters
        self.declare_parameter("safety_cutoff_distance", 0.5)
        self.declare_parameter("safety_scan_distance", 1.0)
        self.declare_parameter("safety_scan_angle_pi", 0.5)
        self.declare_parameter("point_num_thres", 5)
        self.declare_parameter("car_length", 0.5)
        self.declare_parameter("max_steering_angle", 2.0)
        self.declare_parameter("max_deceleration", 10.0)
        self.declare_parameter("max_danger_velocity", 1.0)
        self.declare_parameter("forward_message", True)
        self.declare_parameter("safety_time", 0.001)

        # get parameter values
        self.safety_cutoff_distance = self.get_parameter("safety_cutoff_distance").get_parameter_value().double_value
        self.safety_scan_distance = self.get_parameter("safety_scan_distance").get_parameter_value().double_value
        self.safety_scan_angle_pi = self.get_parameter("safety_scan_angle_pi").get_parameter_value().double_value
        self.point_num_thres = self.get_parameter("point_num_thres").get_parameter_value().integer_value
        self.car_length = self.get_parameter("car_length").get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.max_deceleration = self.get_parameter("max_deceleration").get_parameter_value().double_value
        self.max_danger_velocity = self.get_parameter("max_danger_velocity").get_parameter_value().double_value
        self.forward_message = self.get_parameter("forward_message").get_parameter_value().bool_value
        self.safety_time = self.get_parameter("safety_time").get_parameter_value().integer_value

        # create publishers/subscribers
        self.input_drive_sub = self.create_subscription(AckermannDriveStamped, "drive_input_topic", self.input_drive_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan_topic", self.scan_callback, 10)
        self.safety_pub = self.create_publisher(AckermannDriveStamped, "safety_drive_topic", 10)
        self.wall_pub = self.create_publisher(Marker, 'wall_pub', 10)

        self.safety_timer = self.create_timer(self.safety_time, self.safety_callback)

        # member variables
        self.point_cloud = []
        self.last_drive_msg = AckermannDriveStamped()

    def compute_wall(self, point_set):
        """
        Given a non-empty list of points, perform linear regression to compute the coefficient and slope of the wall next to the robot
        """
        x_list, y_list = zip(*point_set)
        X = np.array([x_list]).T
        X_aug = np.hstack([X,np.full((len(x_list),1),1.0)])
        Y = np.array(y_list).T
        try:
            analytic_sol = np.linalg.inv(X_aug.T@X_aug)@(X_aug.T@Y)
        except np.linalg.LinAlgError as error:
            lam = 0.01
            analytic_sol = np.linalg.inv(X_aug.T@X_aug+len(x_list)*lam*np.eye(2))@(X_aug.T@Y) # add normalization
        slope, offset = analytic_sol[0], analytic_sol[1]
        distance = abs(offset)/math.sqrt(slope**2+1)
        return ((slope, offset), distance)
    
    def distance(self, point, wall_params):
        """
        Computes the distance of a point from a wall with given params
        """
        x, y = point
        slope, offset = wall_params
        return (slope*x+offset-y)/math.sqrt(slope**2+offset**2)

    def compute_min_distance(self, drive_msg, wall_params):
        """
        Given a drive message and a wall, compute the minimum distance of the trajectory of the robot, until it stops, from the wall
        """
        # predict the trajectory of the car given the message received
        # compute stopping distance using command speed and max deceleration
        stop_dist = (drive_msg.drive.speed**2)/(2*self.max_deceleration)
        ((slope, offset), _) = wall_params
        if abs(drive_msg.drive.steering_angle) < 0.001:
            # just assume it's going straight to not have to deal with huge turning radiuses
            # check if it crosses the predicted wall
            if (-offset)*(stop_dist-offset) < 0:
                # crosses
                return 0.0
            else:
                return min(abs(offset), abs(stop_dist-offset))
        else:
            turning_radius = self.car_length / math.tan(drive_msg.drive.steering_angle) # positive if turning left, negative if right
            turn_angle = stop_dist / turning_radius # positive if left, negative if right,m -pi/2 to pi/2
            wall_turn_center_dist = self.distance((-turning_radius, 0),(slope, offset)) # positive if line above point, negative if below
            # compute 0 to pi angle perpendicular to the wall
            if wall_turn_center_dist > 0:
                wall_angle = math.pi/2.0 + math.atan(slope)
            else:
                wall_angle = math.atan(slope) + 3.0*math.pi/2.0
            # compute the endpoints of the arc
            stop_point = (turning_radius*(math.cos(turn_angle)-1),turning_radius*math.sin(turn_angle))
            endpoint_min = min(abs(self.distance((0,0),(slope, offset))), abs(self.distance(stop_point,(slope, offset))))
            # check configuration
            if abs(turning_radius) < abs(wall_turn_center_dist):
                # definitely doesn't crash
                if (turn_angle > 0.0 and wall_angle > 0.0 and wall_angle < turn_angle) or (turn_angle < 0.0 and wall_angle < math.pi and wall_angle > math.pi + turn_angle):
                    return abs(wall_turn_center_dist)-abs(turning_radius)
                else:
                    return endpoint_min
            else:
                if (turn_angle > 0.0 and turn_angle > (wall_angle - math.acos(abs(wall_turn_center_dist/turning_radius)))) or (turn_angle < 0.0 and (wall_angle+math.acos(abs(wall_turn_center_dist/turning_radius))) > math.pi + turn_angle):
                    # crash
                    return 0.0
                else:
                    return endpoint_min

    def input_drive_callback(self, in_drive_msg):
        """
        Gets the drive message, stores it, checks if it's dangerous and if not forward it if needed
        """
        self.last_drive_msg = in_drive_msg
        if self.forward_message and (len(self.point_cloud) < self.point_num_thres or self.compute_min_distance(in_drive_msg, self.compute_wall(self.point_cloud)) > self.safety_cutoff_distance):
            self.safety_pub.publish(in_drive_msg)

    def scan_callback(self, scan_msg):
        """
        Gets the LiDAR scan, filters the points in a scan cone, and stores their x, y coordinates
        """
        self.point_cloud = []
        for i, range in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i*scan_msg.angle_increment
            point = (range*math.cos(angle+math.pi/2.0),range*math.sin(angle+math.pi/2.0)) # x right, y front
            if abs(angle) < self.safety_scan_angle_pi*math.pi/2.0 and range < self.safety_scan_distance:
                self.point_cloud.append(point)

    def safety_callback(self):
        """
        Using the last-received drive message and laser scan, compute if the robot needs to stop or it can avoid a crash by turning
        """
        # TODO: Implement/check backwards safety
        if(len(self.point_cloud) < self.point_num_thres):
            return
        pred_wall = self.compute_wall(self.point_cloud)
        VisualizationTools.visualize_wall(pred_wall, self.wall_pub)
        if self.compute_min_distance(self.last_drive_msg, pred_wall) < self.safety_cutoff_distance:
            # check if taking the sharpest turn possible will make the car avoid the danger (with the same parameters, just limit the velocity)
            header = Header(stamp = self.get_clock().now().to_msg())
            max_left_msg = AckermannDriveStamped(header = header, 
                                                 drive = AckermannDrive(steering_angle = self.max_steering_angle, 
                                                                        speed = self.max_danger_velocity))
            max_right_msg = AckermannDriveStamped(header = header, 
                                                  drive = AckermannDrive(steering_angle = -self.max_steering_angle, 
                                                                         speed = self.max_danger_velocity))
            stop_msg = AckermannDriveStamped(header = header, drive = AckermannDrive(speed = 0.0))
            ((slope, offset), distance) = pred_wall
            if offset > 0.0:
                wall_angle = math.pi/2.0 + math.atan(slope)
            else:
                wall_angle = math.atan(slope) + 3.0*math.pi/2.0
            self.get_logger().info(f"WALL ANGLE: {wall_angle}")
            if wall_angle < math.pi/2:
                # should go left
                if self.compute_min_distance(max_left_msg, pred_wall) > self.safety_cutoff_distance:
                    self.safety_pub.publish(max_left_msg)
                else:
                    self.safety_pub.publish(stop_msg)
            else:
                # should go right
                if self.compute_min_distance(max_right_msg, pred_wall) > self.safety_cutoff_distance:
                    self.safety_pub.publish(max_right_msg)
                else:
                    self.safety_pub.publish(stop_msg)

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()