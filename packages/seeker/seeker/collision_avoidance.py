import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Bool

 
class CollisionAvoidance(Node):
    count = 0
    data_range = 10
    collected_data_log = np.zeros((1,data_range))

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
        polarity = angle/abs(angle)

        # Publish values
        try:
            #self.twist_cmd.linear.x = self.dyn_cmd.cal_throttle(self.ek)
            self.twist_cmd.angular.z = (angle + polarity*(1/distance)**sensitivity)/90
            self.twist_publisher.publish(self.twist_cmd)

        except KeyboardInterrupt:
            #self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def talker_callback(self, data):
        collected_data = data.ranges[270:359] + data.ranges[0:90]

        # to-do: optimization
        r_outer = .5
        r_inner = .15
        for num in collected_data:
            if num < r_inner:
                collected_data[collected_data.index(num)] = 999

#        collected_data = np.where(collected_data < np.zeros((1,len(collected_data))) + 0.45, collected_data, 999)
        minVal = min(collected_data)
        index = collected_data.index(minVal)
        angle = index - 90

        collected_data_log[count] = minVal
        count = count + 1

        if count == data_range - 1:
            count = 0
            filtered_data = sum(collected_data_log)/data_range
        else:
            filtered_data = 999

# possibly put these lines into steering_out() vvv
        if filtered_data > r_outer:
            self.get_logger().info("No Object Within Range")
            self.bool_cmd = bool(0)
        else:
            self.get_logger().info("Angle: " + str(angle) + ", AvgDistance: " + str(filtered_data))
            self.bool_cmd = bool(1)
            self.steering_out(distance = filtered_data, angle = angle, index = index)          

            
def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    lidarInfoSize = CollisionAvoidance() # declare the node constructor
    rclpy.spin(lidarInfoSize) # pause the program execution, waits for a request to kill the node (ctrl+c)
    lidarInfoSize.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()
