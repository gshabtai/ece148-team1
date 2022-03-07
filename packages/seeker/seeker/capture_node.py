import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from .dynamic_centering_control import DynamicCenteringControl
from .parameters import Parameters

# import time
# import os

NODE_NAME = 'capture_node'
CENTROID_TOPIC_NAME = '/centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'


class CaptureControl(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.srv = self.create_service(Trigger, 'capture', self.capture_callback)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.centroid_subscription = self.create_subscription(Float64MultiArray, CENTROID_TOPIC_NAME, self.compute_capture, 10)
        self.twist_cmd = Twist()

        self.conduct = 0
        self.ek = 0

        self.param = Parameters()
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
            ])
        self.param.upd_Kp( self.get_parameter('Kp_steering').value) # between [0,1]
        self.param.upd_Ki( self.get_parameter('Ki_steering').value) # between [0,1]
        self.param.upd_Kd( self.get_parameter('Kd_steering').value) # between [0,1]
        self.param.upd_error_threshold( self.get_parameter('error_threshold').value) # between [0,1]
        self.param.upd_zero_throttle( self.get_parameter('zero_throttle').value) # between [-1,1] but should be around 0
        self.param.upd_max_throttle( self.get_parameter('max_throttle').value) # between [-1,1]
        self.param.upd_min_throttle( self.get_parameter('min_throttle').value) # between [-1,1]
        self.param.upd_max_right_steering( self.get_parameter('max_right_steering').value) # between [-1,1]
        self.param.upd_max_left_steering( self.get_parameter('max_left_steering').value) # between [-1,1]

        self.dyn_cmd = DynamicCenteringControl(self.param)

        self.get_logger().info(
            f'\nKp_steering: {self.param.Kp}'
            f'\nKi_steering: {self.param.Ki}'
            f'\nKd_steering: {self.param.Kd}'
            f'\nerror_threshold: {self.param.error_threshold}'
            f'\nzero_throttle: {self.param.zero_throttle}'
            f'\nmax_throttle: {self.param.max_throttle}'
            f'\nmin_throttle: {self.param.min_throttle}'
            f'\nmax_right_steering: {self.param.max_right_steering}'
            f'\nmax_left_steering: {self.param.max_left_steering}'
        )

    def capture_callback(self, request, response):

        # # Publish values
        # try:
        #     # publish control signals
        #     self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.ek, self.parameters)
        #     self.twist_cmd.angular.z = self.dyn_cmd.cal_steering(self.ek, self.parameters)
        #     self.twist_publisher.publish(self.twist_cmd)

        #     # shift current time and error values to previous values
        #     update_ek_1(self.ek)

        # except KeyboardInterrupt:
        #     self.twist_cmd.linear.x = self.zero_throttle
        #     self.twist_publisher.publish(self.twist_cmd)
    
        # return response
        pass

    #Update ek (Dont know if this works)
    # def compute_capture(self, data):
    #     self.ek = float(data.data[0] / 200)

    # Does computation for to align robot
    def compute_capture(self, data):
        # setting up PID control
        self.ek = float(data.data[0] / 200)

        # Publish values
        try:
            # publish control signals
            self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.ek, self.parameters)
            self.twist_cmd.angular.z = self.dyn_cmd.cal_steering(self.ek, self.parameters)
            self.twist_publisher.publish(self.twist_cmd)

            # shift current time and error values to previous values
            update_ek_1(self.ek)

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
        robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()