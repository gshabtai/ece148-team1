import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

class DynamicCenteringControl():
    '''Caculate needed throttle and steering to center robocar on ball'''
    def __init__(self, param):
        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8

        self.Kp = param.Kp
        self.Ki = param.Ki
        self.Kd = param.Kd
        self.error_threshold = param.error_threshold
        self.zero_throttle = param.zero_throttle
        self.max_throttle = param.max_throttle
        self.min_throttle = param.min_throttle
        self.max_right_steering = param.max_right_steering
        self.max_left_steering = param.max_left_steering

    def update_ek_1(self, ek_1):
        self.ek_1 = ek_1

    def cal_throttle(self, ek):
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = (self.min_throttle - self.max_throttle) * abs(ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)
        return float(throttle_float)

    def cal_steering(self, ek):
        self.proportional_error = self.Kp * ek
        self.derivative_error = self.Kd * (ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)
        return float(steering_float)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 

class Parameters():

    def __init__(self):
        self.Kp = 1
        self.Ki = 0
        self.Kd = 0
        self.error_threshold = 0.15
        self.zero_throttle = 0.0
        self.max_throttle = 0.2
        self.min_throttle = 0.1
        self.max_right_steering = 1.0
        self.max_left_steering = -1.0

    def upd_Kp(self, val): self.Kp = val
    def upd_Ki(self, val): self.Ki = val
    def upd_Kd(self, val): self.Kd = val
    def upd_error_threshold(self, val): self.error_threshold = val
    def upd_zero_throttle(self, val): self.zero_throttle = val
    def upd_max_throttle(self, val): self.max_throttle = val
    def upd_min_throttle(self, val): self.min_throttle = val
    def upd_max_right_steering(self, val): self.max_right_steering = val
    def upd_max_left_steering(self, val): self.max_left_steering = val

NODE_NAME = 'align_node'
BALL_TOPIC_NAME = '/ball_found'
BALL_CEN_TOPIC_NAME = '/ball_centroid'

ACTUATOR_TOPIC_NAME = '/cmd_vel'

class Robocar_align(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.client = self.create_client(Trigger, 'capture')
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        #self.find_ball_subscriber = self.create_subscription(Bool, BALL_TOPIC_NAME, self.update_ball, 10)
        #self.ball_cen_subscriber = self.create_subscription(Point, BALL_CEN_TOPIC_NAME, self.update_ball_cen, 10)

        # self.ball = 0
        # self.cen.x = 0
        # self.cen.y = 0
        # self.cen.depth = 0
        self.twist_cmd = Twist()

        self.param = Parameters()

        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = Trigger.Request()

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

    def send_request(self):
        # send the request
        self.future = self.client.call_async(self.req)
        response = self.future.result()
        return response.success

    def test_send_request(self):
        # send the request
        self.future = self.client.call_async(self.req)
        response = self.future.result()
        # suc = response.success
        self.get_logger().info(f"{response}")
        # self.get_logger().info('%d' % int(suc))

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
                    self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.cen.x)
                    self.twist_cmd.angular.z = self.dyn_cmd.cal_steering(self.cen.x)
                    self.twist_publisher.publish(self.twist_cmd)

                    # shift current time and error values to previous values
                    self.dyn_cmd.update_ek_1(self.cen.x)

                except KeyboardInterrupt:
                    self.twist_cmd.linear.x = self.zero_throttle
                    self.twist_publisher.publish(self.twist_cmd)

            else:
                # turn on fans
                self.twist_cmd.angualr.y = 1
                self.twist_publisher.publish(self.twist_cmd)
                
                # perform capture
                self.get_logger().info('%d' % int(self.send_request()))
                # I want to test the future to see what the message and result is

                # turn off fans
                self.twist_cmd.angualr.y = 0
                self.twist_publisher.publish(self.twist_cmd)

        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    robocar_seek = Robocar_align()
    robocar_seek.test_send_request()

    try:
        rclpy.spin_once(robocar_seek)
        robocar_seek.destory_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        robocar_seek.destroy_node()
        rclpy.shutdown()
        robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()