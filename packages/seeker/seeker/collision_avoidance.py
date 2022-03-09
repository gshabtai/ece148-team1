import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Bool

 
class CollisionAvoidance(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('counter_publisher')
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.talker_callback,10)
        self.collision__avoidance_state = self.create_publisher(Bool, '/collision_avoidance_state', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bool_cmd = Bool()
        self.twist_cmd = Twist()

    def steering_out(self,distance,angle,index):
        sensitivity = 1.5
        

        # Publish values
        try:
            polarity = angle/abs(angle)
            #self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.ek)
            self.twist_cmd.angular.z = (angle + polarity*(1/distance)**sensitivity)/90
            self.twist_publisher.publish(self.twist_cmd)

        except KeyboardInterrupt:
            #self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def talker_callback(self, data):
        r_outer = .5
        r_inner = .15
        filtered_data = data.ranges[270:359] + data.ranges[0:90]

        # to-do: optimization
        for num in filtered_data:
            if num < r_inner:
                filtered_data[filtered_data.index(num)] = 999

#        filtered_data = np.where(filtered_data < np.zeros((1,len(filtered_data))) + 0.45, filtered_data, 999)
        minVal = min(filtered_data)
        index = filtered_data.index(minVal)
        angle = index - 90

        if minVal > r_outer:
            self.get_logger().info("No Object Within Range")
            self.bool_cmd = bool(0)
        else:
            self.get_logger().info("Angle: " + str(angle) + ", Distance: " + str(minVal))
            self.bool_cmd = bool(1)

        if minVal < r_outer:
            self.steering_out(distance = minVal, angle = angle, index = index)

            
def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    lidarInfoSize = CollisionAvoidance() # declare the node constructor
    rclpy.spin(lidarInfoSize) # pause the program execution, waits for a request to kill the node (ctrl+c)
    lidarInfoSize.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()
