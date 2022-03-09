import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

 
class CollisionAvoidance(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('counter_publisher')
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.talker_callback,10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

 
    def talker_callback(self, data):
        r_outer = 2
        r_inner = .45
        filtered_data = data.ranges[270:359] + data.ranges[0:90]
        # to-do: optimization
        for num in filtered_data:
            if num < r_inner:
                filtered_data[filtered_data.index(num)] = 999
#        filtered_data = np.where(filtered_data < np.zeros((1,len(filtered_data))) + 0.45, filtered_data, 999)
        minVal = min(filtered_data)
        index = filtered_data.index(minVal)
    
        self.get_logger().info("Angle: " + str(index) + ", Distance: " + str(minVal))

      #  if minVal < r_outer:
      #      steering_out(distance = minVal, angle = index)


    def steering_out(self,distance,angle):
        sensitivity = 1

        # Publish values
        try:
            # publish control signals
            #self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.ek)
            self.twist_cmd.angular.z = angle + (1/distance)**sensitivity
            self.twist_publisher.publish(self.twist_cmd)

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)
            
def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    lidarInfoSize = CollisionAvoidance() # declare the node constructor
    rclpy.spin(lidarInfoSize) # pause the program execution, waits for a request to kill the node (ctrl+c)
    lidarInfoSize.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()
