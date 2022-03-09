import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy

 
class CollisionAvoidance(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('counter_publisher')
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.talker_callback,10)

 
    def talker_callback(self, data):
        filtered_data = data.ranges[270:359] + data.ranges[0:90]
#        [minVal, index] = min(data.ranges[])
        self.get_logger().info(str(filtered_data))
            
def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    lidarInfoSize = CollisionAvoidance() # declare the node constructor
    rclpy.spin(lidarInfoSize) # pause the program execution, waits for a request to kill the node (ctrl+c)
    lidarInfoSize.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()
