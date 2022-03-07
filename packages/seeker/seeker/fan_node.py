import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
from geometry_msgs.msg import Twist
# import board
# import busio
import RPi.GPIO as GPIO

NODE_NAME = 'fan_node'
TOPIC_NAME = '/cmd_vel'

class AdafruitFan(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.fan_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.send_values_to_adafruit, 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', int(1)),
                ('fan1_channel', int(14)),
                ('fan2_channel', int(15))
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.fan1_channel = int(self.get_parameter('fan1_channel').value)
        self.fan2_channel = int(self.get_parameter('fan2_channel').value)

        # GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.fan1_channel, GPIO.OUT)

        # if self.bus_num == 0:
        #     i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
        #     self.kit = ServoKit(channels=16, i2c=i2c_bus0)
        # else:
        #     self.kit = ServoKit(channels=16)

    def send_values_to_adafruit(self, data):
        fan_power = data.linear.y

        if (fan_power):
            GPIO.output(self.fan1_channel, GPIO.HIGH)
        else:
            GPIO.output(self.fan1_channel, GPIO.LOW)

        # self.kit.continuous_servo[self.fan1_channel].throttle = fan_power
        # self.kit.continuous_servo[self.fan2_channel].throttle = fan_power

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
