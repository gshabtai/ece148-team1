import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Bool, String
import math
from rcl_interfaces.msg import ParameterType


NODE_NAME = 'collision_avoidance_node'
 
class CollisionAvoidance(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__(NODE_NAME)

        self.declare_parameter('r_outer',.5)
        self.declare_parameter('r_inner',.15)
        self.declare_parameter('r_reverse',.2)

        self.r_inner = self.get_parameter('r_inner').value
        self.r_outer = self.get_parameter('r_outer').value
        self.r_reverse = self.get_parameter('r_reverse').value

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.talker_callback,10)
        self.collision__avoidance_state = self.create_publisher(Bool, '/collision_avoidance_state', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_state_node = self.create_subscription(String, '/state', self.set_state,10)

        self.data_range = 10 #these will be used if I decide to filter out data
        self.count = 0
        self.collected_data_log = np.zeros(self.data_range)

        self.onoff = "Idle"

        self.bool_cmd = Bool()
        self.twist_cmd = Twist()

    def set_state(self,data):
        pass
        self.onoff = data.data

    def steering_out(self,distance,angle,index,reverse):
        if self.onoff != "collision_avoidance":
            return

        sensitivity_turn = .25
        #sensitivity_forward = .04
        speed = .04

        # Publish values
        try:
            self.twist_cmd.linear.x = speed*reverse
            self.twist_cmd.angular.z = -(abs(angle) - 90)*math.copysign(sensitivity_turn/90,angle)*(reverse+1)
            self.twist_publisher.publish(self.twist_cmd)

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = 0
            #self.twist_publisher.publish(self.twist_cmd)

    def talker_callback(self, data):
        

        collected_data = data.ranges[270:359] + data.ranges[0:90]

        # to-do: optimization
        for num in collected_data:
            if num < self.r_inner:
                collected_data[collected_data.index(num)] = 999

#        collected_data = np.where(collected_data < np.zeros((1,len(collected_data))) + 0.45, collected_data, 999)
        minVal = min(collected_data)
        index = collected_data.index(minVal)
        angle = index - 90

        filtered_data = minVal

        #self.collected_data_log[self.count] = minVal
        #self.count = self.count + 1

        #if self.count == self.data_range:
        #    self.count = 0
        #    filtered_data = sum(self.collected_data_log)/self.data_range
        #else:
        #    filtered_data = 999

        if filtered_data > self.r_outer:
            self.get_logger().info("No Object Within Range")
            self.bool_cmd.data = False
            self.collision__avoidance_state.publish(self.bool_cmd)
        else:
            self.get_logger().info("Angle: " + str(angle) + ", AvgDistance: " + str(filtered_data))
            self.bool_cmd.data = True
            self.collision__avoidance_state.publish(self.bool_cmd)
            self.steering_out(distance = filtered_data, angle = angle, index = index, reverse = math.copysign(1,(filtered_data - self.r_reverse)))          

            
def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    lidarInfoSize = CollisionAvoidance() # declare the node constructor
    rclpy.spin(lidarInfoSize) # pause the program execution, waits for a request to kill the node (ctrl+c)
    lidarInfoSize.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()
