import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO

NODE_NAME = 'fan_node'
TOPIC_NAME = '/webcam_centroid'

class AdafruitFan(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.webcam_subscriber = self.create_subscription(Float64MultiArray, TOPIC_NAME, self.send_values_to_adafruit, 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', int(1)),
                ('fan1_channel', int(13)),
                ('fan2_channel', int(15))
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.fan1_channel = int(self.get_parameter('fan1_channel').value)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.fan1_channel, GPIO.OUT)

    def send_values_to_adafruit(self, data):
        # fan_power = int(data[2])
        self.get_logger().info(str(data))

        # if (fan_power == int(1)):
        #     GPIO.output(self.fan1_channel, GPIO.HIGH)
        #     self.get_logger().info('On')
        # else:
        #     GPIO.output(self.fan1_channel, GPIO.LOW)
        #     self.get_logger().info('Off')

def main(args=None):
    rclpy.init(args=args)
    adafruit_fan = AdafruitFan()
    try:
        rclpy.spin(adafruit_fan)
    except KeyboardInterrupt:
        adafruit_fan.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        adafruit_fan.destroy_node()
        rclpy.shutdown()
        adafruit_fan.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()