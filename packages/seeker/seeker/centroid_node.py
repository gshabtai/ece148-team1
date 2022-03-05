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

NODE_NAME = 'centroid_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'

class JSONManager():
    def __init__(self):
        f = open('settings.json')
        self.settings = json.load(f)
        f.close

        # Set global parameters
        self.calibration_mode = self.settings['calibration_mode']

        # Set Color detection paramenters
        self.lower_hue1 = self.settings['color_detection']['lower_hue1']
        self.lower_sat1 = self.settings['color_detection']['lower_sat1']
        self.lower_val1 = self.settings['color_detection']['lower_val1']
        self.upper_hue1 = self.settings['color_detection']['upper_hue1']
        self.upper_sat1 = self.settings['color_detection']['upper_sat1']
        self.upper_val1 = self.settings['color_detection']['upper_val1']

    def save_settings(self):
        if self.calibration_mode:
            # Save color detection parameters
            self.settings['color_detection']['lower_hue1'] = self.lower_hue1
            self.settings['color_detection']['lower_sat1'] = self.lower_sat1
            self.settings['color_detection']['lower_val1'] = self.lower_val1
            self.settings['color_detection']['upper_hue1'] = self.upper_hue1
            self.settings['color_detection']['upper_sat1'] = self.upper_sat1
            self.settings['color_detection']['upper_val1'] = self.upper_val1

            f = open('settings.json','w')
            f.write(json.dumps(self.settings, indent=4))
            f.close()
class FindCentroid(Node,JSONManager):
    def __init__(self):
        Node.__init__(self, NODE_NAME)
        JSONManager.__init__(self)
        self.centroid_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.camera_subscription = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.bridge = CvBridge()

    def locate_centroid(self, data):
        # Image processing from rosparams
        self.frame = self.bridge.imgmsg_to_cv2(data)

        print(self.lower_hue1)
        # self.hsv_search()

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

            print(f'Centroid found at: {(cX,cY)}')

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
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()