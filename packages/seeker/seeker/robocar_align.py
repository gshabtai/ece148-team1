import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String, Bool
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from .dynamic_centering_control import DynamicCenteringControl

NODE_NAME = 'robocar_align_node'
CENTROID_TOPIC_NAME = '/intel_centroid'
POINT_CLOUD_TOPIC_NAME = '/camer/aligned_depth_to_color/color/points'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
STATE_TOPIC_NAME = '/state'
BALL_DISTANCE_TOPIC_NAME = '/ball_distance_bool'

class CaptureControl(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.centroid_subscription = self.create_subscription(Float64MultiArray, CENTROID_TOPIC_NAME, self.compute_maneuver, 10)
        self.depth_subscription = self.create_subscription(PointCloud2, CENTROID_TOPIC_NAME, self.find_depth, 10)
        self.state_subscription = self.create_subscription(String, STATE_TOPIC_NAME, self.update_state, 10)
        self.ball_distance_publisher = self.create_publisher(Bool, BALL_DISTANCE_TOPIC_NAME, self.update_ball_distance, 10)
        self.twist_cmd = Twist()

        self.state = ''
        self.conduct = 0
        self.ek = 0
        self.cen_X = 0
        self.cen_Y = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.04),
                ('min_throttle', 0.02),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0),
                ('cen_offset', 0),
                ('min_ball_depth', 0)
            ])
        self.dyn_cmd = DynamicCenteringControl()

        self.dyn_cmd.Kp_steering = ( self.get_parameter('Kp_steering').value) # between [0,1]
        self.dyn_cmd.Ki_steering = ( self.get_parameter('Ki_steering').value) # between [0,1]
        self.dyn_cmd.Kd_steering = ( self.get_parameter('Kd_steering').value) # between [0,1]
        self.dyn_cmd.error_threshold = ( self.get_parameter('error_threshold').value) # between [0,1]
        self.dyn_cmd.zero_throttle = ( self.get_parameter('zero_throttle').value) # between [-1,1] but should be around 0
        self.dyn_cmd.max_throttle = ( self.get_parameter('max_throttle').value) # between [-1,1]
        self.dyn_cmd.min_throttle = ( self.get_parameter('min_throttle').value) # between [-1,1]
        self.dyn_cmd.max_right_steering = ( self.get_parameter('max_right_steering').value) # between [-1,1]
        self.dyn_cmd.max_left_steering = ( self.get_parameter('max_left_steering').value) # between [-1,1]
        
        self.cen_offset = ( self.get_parameter('cen_offset').value) # pixal val
        self.min_ball_depth = ( self.get_parameter('min_ball_depth').value) # pixal val

        self.get_logger().info(
            f'\nKp_steering: {self.dyn_cmd.Kp_steering}'
            f'\nKi_steering: {self.dyn_cmd.Ki_steering}'
            f'\nKd_steering: {self.dyn_cmd.Kd_steering}'
            f'\nerror_threshold: {self.dyn_cmd.error_threshold}'
            f'\nzero_throttle: {self.dyn_cmd.zero_throttle}'
            f'\nmax_throttle: {self.dyn_cmd.max_throttle}'
            f'\nmin_throttle: {self.dyn_cmd.min_throttle}'
            f'\nmax_right_steering: {self.dyn_cmd.max_right_steering}'
            f'\nmax_left_steering: {self.dyn_cmd.max_left_steering}'
            f'\ncen_offest: {self.cen_offset}'
            f'\nmin_ball_depth: {self.min_ball_depth}'
        )

    def update_state(self, data):
        '''Update state for align node object'''
        self.state = data.data

    def find_depth(self, depth_feild):
        '''Sets Point Feild from RealSense Camera'''
        image = self.bridge.imgmsg_to_cv2(depth_feild)
        #In mm
        depth = image[int(self.cen_X), int(self.cen_Y)]
        #In meters
        depth = depth / 100.0

        if depth < self.min_ball_depth:
            return self.ball_distance_publisher(True)
        return self.ball_distance_publisher(False)
    
    def compute_maneuver(self, data):
        '''PID Controler and Twist Pulbisher for robo-movement navigate'''

        if self.state != 'navigate':
            return
        else:
            
            rel_X = data.data[0]
            self.cen_Y = data.data[3]
            self.cen_Y = data.data[4]
            #Offset for ball alignment
            self.ek = rel_X - self.cen_offset

            # setting up PID control
            scale = 50.0
            self.ek = self.ek / scale

            
            # Publish values
            try:
                # publish control signals
                self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.ek)
                self.twist_cmd.angular.z = self.dyn_cmd.cal_steering(self.ek)
                self.twist_publisher.publish(self.twist_cmd)

                # shift current time and error values to previous values
                self.dyn_cmd.ek_1 = self.ek

            except KeyboardInterrupt:
                self.twist_cmd.linear.x = self.zero_throttle
                self.twist_publisher.publish(self.twist_cmd)

def main(args=None):
    rclpy.init(args=args)
    twist_publisher = CaptureControl()
    try:
        rclpy.spin(twist_publisher)
    except KeyboardInterrupt:
        twist_publisher.destroy_node()
        rclpy.shutdown()
        twist_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()