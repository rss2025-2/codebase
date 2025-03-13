#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Float32
from visualization_msgs.msg import Marker

import cv2
import numpy as np
import random

from wall_follower.visualization_tools import VisualizationTools

class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/vesc/high_level/input/nav_2', self.drive_cb, 10)
        self.wall_pub = self.create_publisher(Marker, '/wall_viz_pub', 10)
        self.front_wall_pub = self.create_publisher(Marker, '/front_wall_viz_pub', 10)
        self.drive_viz_pub = self.create_publisher(Marker, '/drive_viz_pub', 10)
        self.dist_pub = self.create_publisher(Float32, '/wall_dist', 10)
        self.goal_pub = self.create_publisher(Float32, '/goal_dist', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer_angle', 10)

        self.declare_parameter('side', 1)
        self.declare_parameter('desired_distance', 0.5)
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        self.declare_parameter("keypress_time", 0.001)
        self.declare_parameter("window_size", 100)
        self.keypress_time = self.get_parameter("keypress_time").get_parameter_value().double_value
        self.window_size = self.get_parameter("window_size").get_parameter_value().integer_value
        
        # custom params
        self.front_angle_pi = 0.25
        self.inlier_threshold = 0.001
        self.num_ransac_iters = 25
        self.inlier_num = 50

        # data collection
        self.avg_abs_dist = 0.0
        self.count_pts = 0
        self.max_peak = 0.0
        self.min_peak = 0.0
        self.max_peak_time = 0.0
        self.min_peak_time = 0.0
        self.avg_peak_dist = 0.0
        self.count_peak_dist = 0

        # open opencv window to detect keypresses
        cv2.imshow('Evaluator Start/Stop',np.zeros((self.window_size,self.window_size,3), np.uint8))

        # create keypress detector timer callback
        self.key_timer = self.create_timer(self.keypress_time, self.key_detector_callback)

        self.evaluating = False
        self.first_scan = True
    
    # a lot of my code for Lab 2 is used here (Panos)
    def lidar_cb(self, lidar_msg):
        if not self.evaluating:
            return
        sliced_scan, front_scan = self.slice_scan(lidar_msg)
        (a, b, c), distance = self.find_wall_ransac(sliced_scan)
        if len(front_scan) != 0:
            (front_a, front_b, front_c), front_distance = self.find_wall_ransac(front_scan)
        else:
            front_distance = 10.0
        # self.get_logger().info(f'Wall found: {a}x + {b}y + {c} = 0')
        VisualizationTools.visualize_wall(a, b, c, self.wall_pub, stamp = self.get_clock().now().to_msg(), color=(0.0,0.0,1.0), frame='/laser')
        if len(front_scan) != 0:
            VisualizationTools.visualize_wall(front_a, front_b, front_c, self.front_wall_pub, stamp = self.get_clock().now().to_msg(), color=(1.0,0.0,0.0), frame='/laser')
        if(abs(math.atan(-front_a/front_b))/np.pi > 0.15): # front line is perpendicular and not some line to the right
            distance = min(distance, front_distance)
        if(abs(b) < 0.0001):
            wall_angle = np.pi/2.0
        else:
            wall_angle = math.atan(-a/b)

        # data logging
        self.avg_abs_dist = self.avg_abs_dist + abs(distance-self.DESIRED_DISTANCE)
        self.count_pts = self.count_pts + 1
        self.dist_pub.publish(Float32(data=distance))
        self.goal_pub.publish(Float32(data=self.DESIRED_DISTANCE))
        scan_time_seconds = rclpy.time.Time.from_msg(lidar_msg.header.stamp).nanoseconds/1e9
        if self.first_scan:
            self.min_peak_time = self.max_peak_time = scan_time_seconds
        else:
            if(distance > self.max_peak):
                dt = scan_time_seconds - self.min_peak_time
                self.max_peak_time = scan_time_seconds
                self.max_peak = distance
                self.avg_peak_dist = self.avg_peak_dist + 2*dt # since we measure from previous min peak
                self.count_peak_dist = self.count_peak_dist + 1
            elif(distance < self.min_peak):
                dt = scan_time_seconds - self.max_peak_time
                self.min_peak_time = scan_time_seconds
                self.min_peak = distance
                self.avg_peak_dist = self.avg_peak_dist + 2*dt
                self.count_peak_dist = self.count_peak_dist + 1

        self.get_logger().info(f'WALL DISTANCE: {distance}, ANGLE: {wall_angle}')        
        self.first_scan = False

    def slice_scan(self, laser_scan):
        if(self.SIDE == -1):
            # start_angle = max(laser_scan.angle_min, -2*np.pi/3.0) # only look forward
            start_angle = laser_scan.angle_min
            end_angle = 0.0
            front_start_angle = 0.0
            front_end_angle = self.front_angle_pi*np.pi
        else:
            start_angle = 0.0
            # end_angle = min(laser_scan.angle_max, 2*np.pi/3.0)
            end_angle = laser_scan.angle_max
            front_start_angle = -self.front_angle_pi*np.pi
            front_end_angle = 0.0
        # go through the points and check if they are needed based on the above angles
        # then compute their position relative to the robot and store it in a list
        filtered_points = []
        front_points = []
        for i, range in enumerate(laser_scan.ranges):
            angle = laser_scan.angle_min + i*laser_scan.angle_increment
            point = (range*math.cos(angle),range*math.sin(angle))
            if start_angle < angle and angle < end_angle:
                # point wanted
                filtered_points.append(point)
            if front_start_angle < angle and angle < front_end_angle:
                # front points
                front_points.append(point)
        return filtered_points, front_points

    def find_wall_ransac(self, sliced_scan):
        # RANSAC
        wall_params = ((0.0, 0.0, 0.0), 0.0)
        best_num = 0
        count_iters = 0
        avg_error = 0.0
        while best_num < self.inlier_num and count_iters < self.num_ransac_iters:
            i1 = random.randint(0,len(sliced_scan)-1)
            i2 = random.randint(0,len(sliced_scan)-1)
            if(i1==i2):
                continue
            pair_points = (sliced_scan[i1],sliced_scan[i2])
            hypothesis_params = self.fit_pair(pair_points)
            # (a,b,c),_ = hypothesis_params
            # self.get_logger().info(f'Wall checked in RANSAC: {a}x + {b}y + {c} = 0')
            num_in, inlier_error = self.num_inliers(hypothesis_params, sliced_scan)
            if(num_in > best_num):
                wall_params = hypothesis_params
                best_num = num_in
                avg_error = inlier_error
            count_iters = count_iters + 1
        self.get_logger().info(f'Wall found with {best_num} inliers in {count_iters} iterations and average inlier error {avg_error}')
        return wall_params
            
    def num_inliers(self, wall_params, points):
        (a, b, c), _ = wall_params
        count = 0
        in_error_avg = 0.0
        for x,y in points:
            if ((a*x+b*y+c)**2)/(a**2+b**2) < self.inlier_threshold:
                count = count + 1
                in_error_avg = in_error_avg + ((a*x+b*y+c)**2)/(a**2+b**2)
        if(count != 0):
            in_error_avg = in_error_avg/count
        return count, in_error_avg
    
    def fit_pair(self, point_pair):
        (x1,y1), (x2,y2) = point_pair
        # (y-y1)(x2-x1) = (y2-y1)*(x-x1) -> y(x2-x1)+x(y1-y2)+x1(y2-y1)+y1(x1-x2) = 0 -> y(x2-x1)+x(y1-y2)+x1y2-x2y1
        return (y1-y2, x2-x1, x1*y2-x2*y1), abs(x1*y2-x2*y1)/math.sqrt((y1-y2)**2+(x2-x1)**2)

    def drive_cb(self, drive_msg):
        if not self.evaluating:
            return
        # visualize the drive command as a marker
        VisualizationTools.plot_line([0.0,drive_msg.drive.speed*math.cos(drive_msg.drive.steering_angle)],[0.0, drive_msg.drive.speed*math.sin(drive_msg.drive.steering_angle)],self.drive_viz_pub, stamp = self.get_clock().now().to_msg(), color=(0.0,1.0,0.0), frame='/laser')
        self.steer_pub.publish(Float32(data=drive_msg.drive.steering_angle))

    def key_detector_callback(self):
        key = cv2.waitKey(int(self.keypress_time*1000))
        if key == ord('p'):
            if self.evaluating:
                self.evaluating = False
                self.get_logger().info('STOPPED EVALUATING')
                # store results
                self.avg_abs_dist = self.avg_abs_dist/float(self.count_pts)
                self.get_logger().info(f'AVERAGE ABSOLUTE ERROR: {self.avg_abs_dist}')
                self.avg_peak_dist = self.avg_peak_dist/float(self.count_peak_dist)
                self.get_logger().info(f'AVERAGE FREQUENCY OF OSCILLATIONS: {1.0/self.avg_peak_dist}hz')
                # reset variables
                self.avg_abs_dist = 0.0
                self.count_pts = 0
                self.max_peak = 0.0
                self.min_peak = 0.0
                self.max_peak_time = 0.0
                self.min_peak_time = 0.0
                self.avg_peak_dist = 0.0
                self.count_peak_dist = 0
        elif key == ord('s'):
            if not self.evaluating:
                # start evaluating
                self.evaluating = True
                self.get_logger().info('STARTED EVALUATING')

def main():
    rclpy.init()
    evaluator = Evaluator()
    rclpy.spin(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()