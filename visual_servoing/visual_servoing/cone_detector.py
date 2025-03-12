#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel

# Import your color segmentation algorithm.
# In this example we assume that cd_color_segmentation returns a bounding box tuple:
#    (x_min, y_min, x_max, y_max)
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Publishers
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        # Subscribe to ZED camera RGB frames
        self.image_sub = self.create_subscription(Image,
                                                  "/zed/zed_node/rgb/image_rect_color",
                                                  self.image_callback,
                                                  5)
        self.bridge = CvBridge()  # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        bbox = cd_color_segmentation(image)
        if bbox is not None:
            x_min, y_min, x_max, y_max = bbox
            cone_pixel = ConeLocationPixel()
            cone_pixel.x_pos = int((x_min + x_max) / 2)
            cone_pixel.y_pos = int(y_max)

            self.cone_pub.publish(cone_pixel)

            cv2.rectangle(image,
                          (int(x_min), int(y_min)),
                          (int(x_max), int(y_max)),
                          (0, 255, 0), 2)
            cv2.circle(image, (u, v), 5, (0, 0, 255), -1)
        else:
            self.get_logger().info("No cone detected in this frame.")

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error when publishing debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    cone_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
