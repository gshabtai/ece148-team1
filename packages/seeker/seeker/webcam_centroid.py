import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float64MultiArray

NODE_NAME = 'webcam_centroid_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/webcam_centroid'

class FindCentroid(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_publisher = self.create_publisher(Float64MultiArray, CENTROID_TOPIC_NAME, 10)
        self.camera_subscription = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.bridge = CvBridge()
        # Set Color detection paramenters
        self.lower_hue1 = 175
        self.lower_sat1 = 135
        self.lower_val1 = 97
        self.upper_hue1 = 181
        self.upper_sat1 = 254
        self.upper_val1 = 219
        self.msg = Float64MultiArray()

        # Initial moment value
        self.relX = -50.0
        self.relY = -50.0
        self.detected = 0.0

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

    def moment_search(self):
        '''calculate moments of binary image'''
        M = cv.moments(self.mask)

        if int(M["m00"]) != 0:
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            self.detected = 1.0

            h, w = np.shape(self.mask)
            self.relX = 100.0*((cX - w/2)/w)
            self.relY = 100.0*((h-cY)/h)

            # Publish centroid data
            self.msg.data = [self.relX, self.relY, self.detected]
        else:
            self.detected = 0.0
            self.msg.data = [self.relX, self.relY, self.detected]

        self.centroid_publisher.publish(self.msg)

        # Debugging
        # self.get_logger().info(f'Centroid found at: {(self.relX,self.relY,self.detected)}')
    
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