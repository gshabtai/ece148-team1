import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SearchMode(Node):
    def __init__(self):
        super().__init__('search_node')
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(String, '/state', self.set_twist,10)
        self.twist = Twist()
        # self.create_timer(0.2, self.update)

    def set_twist(self, data):
        self.get_logger().info('Searching')

def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    searchObj = SearchMode() # declare the node constructor
    rclpy.spin(searchObj) # pause the program execution, waits for a request to kill the node (ctrl+c)
    searchObj.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()