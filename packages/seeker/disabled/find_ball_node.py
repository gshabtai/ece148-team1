import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Point
from std_msg.msg import Bool
import time
import os
import json

NODE_NAME = 'find_ball_node'
CAMERA_TOPIC_NAME = '' # one of the intel node topics
BALL_TOPIC_NAME = '/ball_found'
BALL_CEN_TOPIC_NAME = '/ball_centroid'

class FindCentroid(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.ball_publisher = self.create_publisher(Bool, BALL_TOPIC_NAME, 10)
        self.centroid_publisher = self.create_publisher(Point, BALL_CEN_TOPIC_NAME, 10)
        self.camera_subscription = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.find_ball, 10)
        self.bridge = CvBridge()
        # Set Color detection paramenters
        self.lower_hue1 = 0
        self.lower_sat1 = 98
        self.lower_val1 = 121
        self.upper_hue1 = 7
        self.upper_sat1 = 195
        self.upper_val1 = 182
        self.calibration_mode = False
        # Centroid data
        self.centroid_info = Float32()

        self.ball_cmd = Bool()
        self.point_cmd = Point()

    def find_ball(self, data):
        self.ball_cmd = bool(0) # Is there a ball?

        self.ball_publisher.publish(self.ball_cmd)
        
        self.point_cmd.x = 0 # x_centroid
        self.point_cmd.y = 0 # y_centroid
        self.point_cmd.z = 0 # depth

        self.centroid_publisher.publish(self.point_cmd)        

def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = FindCentroid()
    try:
        rclpy.spin(centroid_publisher)
    except KeyboardInterrupt:
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()
        robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()