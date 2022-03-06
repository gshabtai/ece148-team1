from urllib import response
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import board
import busio

NODE_NAME = 'robocar_node'
BALL_TOPIC_NAME = '/ball'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

class Robocar_Seek(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.find_ball_subscriber = self.create_subscription(Twist, BALL_TOPIC_NAME, self.move_bot, 10)
        self.client = self.create_client(Empty, 'capture')

        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = Empty.Request()

    def send_request(self):
        # send the request
        self.future = self.client.call_async(self.req)

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
    client = ClientSync()
    try:
        spin_thread = Thread(target=rclpy.spin, args=(client,))
        spin_thread.start()
        response = client.send_request()
        client.get_logger().info(f'{NODE_NAME} client service request sent')
        rclpy.spin(robocar_seek)
        robocar_seek.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        robocar_seek.destroy_node()
        minimal_client.destory_node()
        rclpy.shutdown()
        robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()
