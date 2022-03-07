import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

NODE_NAME = 'seek_node'
BALL_TOPIC_NAME = '/ball_found'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

class Robocar_Seek(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.find_ball_subscriber = self.create_subscription(Bool, BALL_TOPIC_NAME, self.move_bot, 10)
        self.twist_cmd = Twist()

    def move_bot(self, data):
        if(not data.data):
            self.twist_cmd.linear.x = 0.2
            self.twist_cmd.angualr.z = 1
            self.twist_publisher.publish(self.twist_cmd)
            
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    capture_node = CaptureControl()
    try:
        rclpy.spin(capture_node)
    except KeyboardInterrupt:
        capture_node.destroy_node()
        rclpy.shutdown()
        robocar_seek.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()