import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

NODE_NAME = 'simple_states_node'
TWIST_TOPIC_NAME = '/cmd_vel'
STATE_TPOIC_NAME = '/state'

class SimpleModes(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, TWIST_TOPIC_NAME, 10)
        self.subscriber = self.create_subscription(String, STATE_TPOIC_NAME, self.set_twist,10)
        self.twist = Twist()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_throttle', 0.2),
                ('min_throttle', 0.1)
            ])
        
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value

        self.get_logger().info(
            f'\nKd_steering: {self.max_throttle}'
            f'\nKd_steering: {self.min_throttle}'
        )

    def __del__(self):
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist_publisher.publish(self.twist)

    def set_twist(self, data):
        state = data.data

        if state == 'search':
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = -1.0
            self.twist.linear.x = float(self.max_throttle)
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist_publisher.publish(self.twist)
        
        elif state == 'idle':
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist_publisher.publish(self.twist)

        elif state == 'drive_back':
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.twist.linear.x = float(-(self.max_throttle))
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist_publisher.publish(self.twist)
        

def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    simple_states = SimpleModes() # declare the node constructor
    rclpy.spin(simple_states) # pause the program execution, waits for a request to kill the node (ctrl+c)
    simple_states.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()