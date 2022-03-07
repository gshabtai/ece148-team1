import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
# from geometry_msgs.msg import Twist
import time
import os
import json
from std_msgs.msg import Float64MultiArray

NODE_NAME = 'centroid_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'

class FindCentroid(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_publisher = self.create_publisher(Float64MultiArray, CENTROID_TOPIC_NAME, 10)
        self.camera_subscription = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
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

    def locate_centroid(self, data):
        # Image processing from rosparams
        self.frame = self.bridge.imgmsg_to_cv2(data)

        self.hsv_search()
        cv.waitKey(1)

    def hsv_search(self):
        # convert to hsv colorspace
        hsv = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
        self.mask = cv.inRange(hsv, np.array([self.lower_hue1,self.lower_sat1,self.lower_val1]) , 
            np.array([self.upper_hue1, self.upper_sat1, self.upper_val1]))

        # Find largest contour in intermediate image
        countours, _ = cv.findContours(self.mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        out = np.zeros(self.mask.shape, np.uint8)

        if len(countours):
            biggest_blob = max(countours, key=cv.contourArea)
            cv.drawContours(out, [biggest_blob], -1, 255, cv.FILLED)
        
        self.mask = cv.bitwise_and(self.mask, out)

        self.moment_search()
        
        if self.calibration_mode:
            cv.imshow('Mask', out)

    def moment_search(self):
        '''calculate moments of binary image'''
        M = cv.moments(self.mask)

        if int(M["m00"]) != 0:
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # self.get_logger().info(f'Centroid found at: {(cX-400,cY-300)}')

            # Publish centroid data
            msg = Float64MultiArray()
            data = [cX-400.0, cY-300.0]
            msg.data = data
            self.centroid_publisher.publish(msg)
            
            if self.calibration_mode:
                # put text and highlight the center
                cv.circle(self.frame, (cX, cY), 5, (255, 255, 255), -1)
                cv.putText(self.frame, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
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