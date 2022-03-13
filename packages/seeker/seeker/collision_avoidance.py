from ctypes.wintypes import MAX_PATH
from threading import main_thread
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Bool, String
import math
from rcl_interfaces.msg import ParameterType
from time import time


NODE_NAME = 'collision_avoidance_node'
class CollisionAvoidance(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__(NODE_NAME)

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.controller,10)
        self.collision__avoidance_state = self.create_publisher(Bool, '/collision_avoidance_state', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_state_node = self.create_subscription(String, '/state', self.set_state,10)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('r_outer', .5),
                ('r_inner', .15),
                ('r_reverse', .2),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1)
            ]
        )
        
        self.r_inner = self.get_parameter('r_inner').value
        self.r_outer = self.get_parameter('r_outer').value
        self.r_reverse = self.get_parameter('r_reverse').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value

        self.get_logger().info(
            f'\nKp_steering: {self.r_inner}'
            f'\nKi_steering: {self.r_outer}'
            f'\nKd_steering: {self.r_reverse}'
            f'\nKd_steering: {self.max_throttle}'
            f'\nKd_steering: {self.min_throttle}'
        )

        self.data_range = 10 #these will be used if I decide to filter out data
        self.count = 0
        self.collected_data_log = np.zeros(self.data_range)

        self.state = "Idle"

        self.bool_cmd = Bool()
        self.twist_cmd = Twist()

    def set_state(self,data):
        pass
        self.state = data.data

    def steering_out(self, steering, throttle):
        if self.state != "collision_avoidance":
            return
        else:
            try:
                self.twist_cmd.linear.x = throttle
                self.twist_cmd.angular.z = steering
                self.twist_publisher.publish(self.twist_cmd)

            except KeyboardInterrupt:
                pass

    def logic(self, distance, angle):
        #, reverse = minVal > self.r_reverse
        if self.state != "collision_avoidance":
            return

        if abs(angle) < 15:
            timer = time() #Start timmer
            while abs(timer-time()) < 2:
                steering = float(0)
                throttle = float(-(self.max_throttle))
                self.steering_out(steering, throttle)
            return

        # Calculate values
        steering = -(abs(angle) - 90)/90
        throttle = self.min_throttle + (self.max_throttle - self.min_throttle) * (1 - abs(steering))
        if throttle < self.min_throttle:
            throttle = self.min_throttle
        elif throttle > self.max_throttle:
            throttle = self.max_throttle

        print("STEERING", steering)
        print("THROTTLE", throttle)
        self.steering_out(steering, throttle)

    def filterdata(self, data):
        '''returns minval of angle (angle = 0 at front of car)'''
        collected_data = data.ranges[270:359] + data.ranges[0:90]

        # to-do: optimization
        data_filtered = collected_data
        for i in range(0, len(collected_data)):
            if collected_data[i] < self.r_inner:
                data_filtered[i] = 999 

        # collected_data = np.where(collected_data < np.zeros((1,len(collected_data))) + 0.45, collected_data, 999)
        minVal = min(collected_data)
        index = collected_data.index(minVal)
        angle = index - 90

        data = [minVal, angle]

        return data

    def controller(self, data):
        '''Logic for collision Avoidance'''

        collected_data = data.ranges[270:359] + data.ranges[0:90]

        minVal, angle = self.filterdata(data)

        if minVal > self.r_outer:
            self.get_logger().info("No Object Within Range")
            self.bool_cmd.data = False
            self.collision__avoidance_state.publish(self.bool_cmd)
        else:
            self.get_logger().info("Angle: " + str(angle) + ", AvgDistance: " + str(minVal))
            self.bool_cmd.data = True
            self.collision__avoidance_state.publish(self.bool_cmd)
            self.logic(minVal, angle)          

            
def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    lidarInfoSize = CollisionAvoidance() # declare the node constructor
    rclpy.spin(lidarInfoSize) # pause the program execution, waits for a request to kill the node (ctrl+c)
    lidarInfoSize.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()
