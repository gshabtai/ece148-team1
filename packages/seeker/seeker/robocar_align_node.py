import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import dynamic_centering_control

NODE_NAME = 'align_node'
BALL_TOPIC_NAME = '/ball_found'
BALL_CEN_TOPIC_NAME = '/ball_centroid'

ACTUATOR_TOPIC_NAME = '/cmd_vel'

class Robocar_align(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.client = self.create_client(Trigger, 'capture')
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.find_ball_subscriber = self.create_subscription(Bool, BALL_TOPIC_NAME, self.update_ball, 10)
        self.ball_cen_subscriber = self.create_subscription(Point, BALL_CEN_TOPIC_NAME, self.update_ball_cen, 10)

        self.ball = 0
        self.cen.x = 0
        self.cen.y = 0
        self.cen.depth = 0
        self.twist_cmd = Twist()

        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = Trigger.Request()

        # Default actuator values
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
        self.Kp = self.get_parameter('Kp_steering').value # between [0,1]
        self.Ki = self.get_parameter('Ki_steering').value # between [0,1]
        self.Kd = self.get_parameter('Kd_steering').value # between [0,1]
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_throttle = self.get_parameter('zero_throttle').value # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value # between [-1,1]
        self.max_right_steering = self.get_parameter('max_right_steering').value # between [-1,1]
        self.max_left_steering = self.get_parameter('max_left_steering').value # between [-1,1]
        
        self.parameters.Kp = self.Kp
        self.parameters.Ki = self.Ki
        self.parameters.Kd = self.Kd
        self.parameters.error_threshold = self.error_threshold
        self.parameters.zero_throttle = self.zero_throttle
        self.parameters.max_throttle = self.max_throttle
        self.parameters.min_throttle = self.min_throttle
        self.parameters.max_right_steering = self.max_right_steering
        self.parameters.max_left_steering = self.max_left_steering

        self.dyn_cmd = DynamicCenteringControl(self.parameters)

        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )

    def send_request(self):
        # send the request
        self.future = self.client.call_async(self.req)

    def update_ball(self, data):
        self.ball = bool(data.data)

    def update_ball_cen(self, data):
        if (self.ball):
            self.cen.x = float(data.x)
            self.cen.y = float(data.y)
            self.cen.depth = float(data.z)

            if (self.cen.depth > 100):
                #Offset to align ball to right side of robot
                self.cen.x = self.cen.x - 20

                # Publish values
                try:
                    # publish control signals
                    self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.cen.x, self.parameters)
                    self.twist_cmd.angular.z = self.dyn_cmd.cal_steering(self.cen.x, self.parameters)
                    self.twist_publisher.publish(self.twist_cmd)

                    # shift current time and error values to previous values
                    update_ek_1(self.cen.x)

                except KeyboardInterrupt:
                    self.twist_cmd.linear.x = self.zero_throttle
                    self.twist_publisher.publish(self.twist_cmd)

            else:
                # turn on fans
                self.twist_cmd.angualr.y = 1
                self.twist_publisher.publish(self.twist_cmd)
                
                # perform capture
                self.send_request()
                # I want to test the future to see what the message and result is

                # turn off fans
                self.twist_cmd.angualr.y = 0
                self.twist_publisher.publish(self.twist_cmd)

        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    robocar_seek = Robocar_Seek()
    robocar_seek.send_request()

    try:
        while rclpy.ok():
            rclpy.spin_once(robocar_seek)
            if robocar_seek.future.done():
                try:
                    response = robocar_seek.future.result()        
                except Exception as e:
                    robocar_seek.get_logger().info('Service call failed %r' % (e,))
                else:
                    robocar_seek.get_logger().info("RESULT: %d" % int(response.success))
                    robocar_seek.get_logger().info('Success')
                # break
    except KeyboardInterrupt:
        robocar_seek.destroy_node()
        rclpy.shutdown()
        robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()