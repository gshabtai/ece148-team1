import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from .dynamic_centering_control import DynamicCenteringControl

NODE_NAME = 'robocar_align_node'
CENTROID_TOPIC_NAME = '/intel_centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
STATE_TOPIC_NAME = '/state'

class CaptureControl(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.centroid_subscription = self.create_subscription(Float64MultiArray, CENTROID_TOPIC_NAME, self.compute_maneuver, 10)
        self.state_subscription = self.create_subscription(String, STATE_TOPIC_NAME, self.update_state, 10)
        self.twist_cmd = Twist()

        self.state = ''
        self.conduct = 0
        self.ek = 0

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
                ('cen_offset', 0)
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
        )

    def update_state(self, data):
        '''Update state for align node object'''
        self.state = data.data

    def compute_maneuver(self, data):
        '''PID Controler and Twist Pulbisher for robo-movement navigate'''

        if self.state != 'navigate':
            return
        else:
            
            #Offset for ball alignment
            self.ek = data.data[0] - self.cen_offset

            # setting up PID control
            scale = 50
            self.ek = float(self.ek / scale)

            
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