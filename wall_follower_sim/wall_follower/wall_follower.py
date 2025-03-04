#!/usr/bin/env python3
import numpy as np
import rclpy
import random
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_2")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 0.5)
        self.declare_parameter("desired_distance", 0.5)
        # self.declare_parameter("front_ignore_angle_pi", 0.25)
        # self.declare_parameter("kp", 1.0)
        # self.declare_parameter("ki", 0.0)
        # self.declare_parameter("kd", 0.0)
        # self.declare_parameter("acceleration", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        print(self.SCAN_TOPIC)
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        # self.front_ignore_angle_pi = self.get_parameter('front_ignore_angle_pi').get_parameter_value().double_value
        # self.kp = self.get_parameter('kp').get_parameter_value().double_value
        # self.ki = self.get_parameter('ki').get_parameter_value().double_value
        # self.kd = self.get_parameter('kd').get_parameter_value().double_value
        # self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        
        # custom params
        self.front_angle_pi = 0.25
        self.kp = 20
        self.ki = 0.01 # can't have an integral controller since it doesn't restart the node each time and I don't know when the test stops so the becomes huge
        self.kd = 2
        self.inlier_threshold = 0.001
        self.num_ransac_iters = 15
        self.inlier_num = 20
        self.pair_space = 3
        self.wall_angle_weight = 0.0
        self.n_sparse = 1
        self.front_distance_weight = 0.5

        self.get_logger().info(f'KP: {self.kp}, KI: {self.ki}, KD: {self.kd}')
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)   
		
        self.control_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.laser_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.wall_pub = self.create_publisher(Marker, 'wall_pub', 10)
        self.front_wall_pub = self.create_publisher(Marker, 'front_wall_pub', 10)

        # INIT PID VARIABLES
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        self.last_update = rclpy.time.Time()
        self.dt = 0.0
        self.first_scan = True

    def scan_callback(self, msg):
        new_time_scan = rclpy.time.Time.from_msg(msg.header.stamp)
        if(not self.first_scan):
            self.dt = (new_time_scan - self.last_update).nanoseconds/1e9
        self.last_update = new_time_scan

        sliced_scan, front_scan = self.slice_scan(msg)
        (a, b, c), distance = self.find_wall_ransac(sliced_scan)
        if len(front_scan) != 0:
            (front_a, front_b, front_c), front_distance = self.find_wall_ransac(front_scan)
        else:
            front_distance = 10.0
        # self.get_logger().info(f'Wall found: {a}x + {b}y + {c} = 0')
        VisualizationTools.visualize_wall(a, b, c, self.wall_pub)
        if len(front_scan) != 0:
            VisualizationTools.visualize_wall(front_a, front_b, front_c, self.front_wall_pub, color=(1.0,0.0,0.0))
        if(abs(math.atan(-front_a/front_b))/np.pi > 0.15): # front line is perpendicular and not some line to the right
            control = self.pid_update(min(distance, self.front_distance_weight*front_distance))
        else:
            control = self.pid_update(distance)
        # we control the angle of the wheels with the wall, to not go against it
        if(abs(b) < 0.0001):
            wall_angle = np.pi/2.0
        else:
            wall_angle = math.atan(-a/b)
        steer = control + self.wall_angle_weight*wall_angle
        # self.get_logger().info(f'Control: {control}, wall angle: {wall_angle}')
        self.drive_command(steer)
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
            if(i%self.n_sparse != 0): # make the points sparser to make the algorithm faster
                continue
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
        # for i in range(len(sliced_scan)-self.pair_space): # instead of randomly selecting points, since they are in order of angle, choose pair spaced a certain number of points apart to have a good line
        #     pair_points = (sliced_scan[i], sliced_scan[i+self.pair_space])
        # while best_num < self.inlier_num and count_iters < self.num_ransac_iters:
        #     i1 = random.randint(0,len(sliced_scan)-self.pair_space-1) # combine the above two approaches
            # pair_points = (sliced_scan[i1],sliced_scan[i1+self.pair_space])
            hypothesis_params = self.fit_pair(pair_points)
            # (a,b,c),_ = hypothesis_params
            # self.get_logger().info(f'Wall checked in RANSAC: {a}x + {b}y + {c} = 0')
            num_in, inlier_error = self.num_inliers(hypothesis_params, sliced_scan)
            if(num_in > best_num):
                wall_params = hypothesis_params
                best_num = num_in
                avg_error = inlier_error
            count_iters = count_iters + 1
        # self.get_logger().info(f'Wall found with {best_num} inliers in {count_iters} iterations and average inlier error {avg_error}')
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
    
    def fit_points(self, point_set):
        # linear regression, analytical solution
        x_list, y_list = zip(*point_set)
        X = np.array([x_list]).T
        X_aug = np.hstack([X,np.full((len(x_list),1),1.0)])
        Y = np.array(y_list).T
        try:
            analytic_sol = np.linalg.inv(X_aug.T@X_aug)@(X_aug.T@Y)
        except np.linalg.LinAlgError as error:
            analytic_sol = np.linalg.inv(X_aug.T@X_aug+len(x_list)*0.01*np.eye(2))@(X_aug.T@Y)
        slope, offset = analytic_sol[0], analytic_sol[1]
        distance = abs(offset)/math.sqrt(slope**2+1)
        return ((slope, offset), distance)
    
    def fit_pair(self, point_pair):
        (x1,y1), (x2,y2) = point_pair
        # (y-y1)(x2-x1) = (y2-y1)*(x-x1) -> y(x2-x1)+x(y1-y2)+x1(y2-y1)+y1(x1-x2) = 0 -> y(x2-x1)+x(y1-y2)+x1y2-x2y1
        return (y1-y2, x2-x1, x1*y2-x2*y1), abs(x1*y2-x2*y1)/math.sqrt((y1-y2)**2+(x2-x1)**2)

    def pid_update(self, distance):
        self.get_logger().info(f'Distance: {distance}, desired distance: {self.DESIRED_DISTANCE}, side: {self.SIDE}')
        new_error = self.SIDE*(distance - self.DESIRED_DISTANCE) # signed error to work with controller in both ways
        if(not self.first_scan):
            error_diff = new_error - self.error
        self.error = new_error
        if(not self.first_scan):
            self.error_integral = self.error_integral + self.error*self.dt
            self.error_derivative = error_diff/self.dt
        control = self.kp*self.error + self.ki*self.error_integral + self.kd*self.error_derivative
        self.get_logger().info(f'Control: {control}')
        self.get_logger().info(f'Error: {self.error}, Integral: {self.error_integral}, Derivative: {self.error_derivative}')
        return control

    def drive_command(self, steering):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.steering_angle_velocity = 0.0 # steer as fast as possible
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.acceleration = 0.0 # reach needed velocity as fast as possible
        drive_msg.drive.jerk = 0.0
        self.get_logger().info(f'Output {steering}')
        self.control_pub.publish(drive_msg)
    
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


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
