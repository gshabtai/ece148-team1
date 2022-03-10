import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SimpleModes(Node):
    def __init__(self):
        super().__init__('search_node')
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(String, '/state', self.set_twist,10)
        self.twist = Twist()
        # self.create_timer(0.2, self.update)

    def set_twist(self, data):
        state = data.data

        # Do nothing if not in search mode
        if state != 'search_mode':
            return

        self.twist.angular.z = -1.0
        self.twist.linear.x = 0.05
        self.twist_publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    simple_states = SimpleModes() # declare the node constructor
    rclpy.spin(simple_states) # pause the program execution, waits for a request to kill the node (ctrl+c)
    simple_states.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()