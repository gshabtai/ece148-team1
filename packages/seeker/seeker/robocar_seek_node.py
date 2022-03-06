import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import board
import busio

NODE_NAME = 'seeker_node'
BALL_TOPIC_NAME = '/ball'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

class Robocar_Seek(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.client = self.create_client(SetBool, 'capture')
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.find_ball_subscriber = self.create_subscription(Twist, BALL_TOPIC_NAME, self.move_bot, 10)

        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = SetBool.Request()

    def send_request(self):
        # send the request
        self.req.data = bool(1)
        self.future = self.client.call_async(self.req)
        response = self.future.result()
        self.get_logger().info('RESPONSE: %d' % int(response.success))

    def move_bot(self, data):
        self.ball_dis = data.linear.z
        self.ball_x = data.angular.z

        if(self.ball_dis == 0):
            self.twist_cmd.linear.x = .5
            self.twist_cmd.angular.z = 1
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    robocar_seek = Robocar_Seek()
    robocar_seek.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(robocar_seek)
        if robocar_seek.future.done():
            try:
                response = robocar_seek.future.result()
            except Exception as e:
                robocar_seek.get_logger().info('Service call failed %r' % (e,))
        else:
            robocar_seek.get_logger().info('Success')
        break

    robocar_seek.destroy_node()
    rclpy.shutdown()
    robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
