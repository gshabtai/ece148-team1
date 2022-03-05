import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import numpy as np
# from geometry_msgs.msg import Twist
import time
import os

NODE_NAME = 'centroid_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'


class FindCentroid(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.camera_subscription = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.bridge = CvBridge()

    def locate_centroid(self, data):
        # Image processing from rosparams
        frame = self.bridge.imgmsg_to_cv2(data)

        print(np.shape(frame))
        cv2.imshow('img', frame)
        cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = FindCentroid()
    try:
        rclpy.spin(centroid_publisher)
    except KeyboardInterrupt:
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()