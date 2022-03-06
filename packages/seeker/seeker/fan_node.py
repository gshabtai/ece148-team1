import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
from geometry_msgs.msg import Twist
import board
import busio

NODE_NAME = 'fan_node'
TOPIC_NAME = '/cmd_vel'

class AdafruitFan(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.fan_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.send_values_to_adafruit, 10)
        self.default_bus_num = int(1)
        self.default_fan1_channel = int(14)
        self.default_fan2_channel = int(15)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', self.default_bus_num),
                ('fan1_channel', self.default_fan1_channel),
                ('fan2_channel', self.default_fan2_channel)
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.fan1_channel = int(self.get_parameter('fan1_channel').value)
        self.fan2_channel = int(self.get_parameter('fan2_hannel').value)

        if self.bus_num == 0:
            i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
            self.kit = ServoKit(channels=16, i2c=i2c_bus0)
        else:
            self.kit = ServoKit(channels=16)

    def send_values_to_adafruit(self, data):
        fan_power = data.linear.y
        self.kit.continuous_servo[self.fan1_channel].throttle = fan_power
        self.kit.continuous_servo[self.fan2_channel].throttle = fan_power

def main(args=None):
    rclpy.init(args=args)
    adafruit_fan = AdafruitFan()
    try:
        rclpy.spin(adafruit_fan)
        adafruit_fan.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        adafruit_fan.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        adafruit_fan.destroy_node()
        rclpy.shutdown()
        adafruit_fan.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
