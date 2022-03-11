from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
import RPi.GPIO as GPIO

NODE_NAME = 'intake_system_node'
TOPIC_NAME = '/webcam_centroid'

class AdafruitFan(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # update called every time topic publishes new value
        self.webcam_subscriber = self.create_subscription(Float64MultiArray, TOPIC_NAME, self.update, 10)
        self.num_balls = self.create_publisher(Int8,'/num_ball_picked_up', 10)

        self.pub_data = Int8()
        self.pub_data.data = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', int(1)),
                ('fan1_channel', int(13)),
                ('fan2_channel', int(15))
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.fan1_channel = int(self.get_parameter('fan1_channel').value)
        
        self.cur_fan_on = False     # keeps track if the fan is on
        self.cur_ball_detected = False # keeps track if a ball has been detected


        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.fan1_channel, GPIO.OUT)

    # turn on fan
    def fan_on(self):
        GPIO.output(self.fan1_channel, GPIO.HIGH)
        self.cur_fan_on = True
        self.get_logger().info('Fan ON')

    # turn off fan
    def fan_off(self):
        GPIO.output(self.fan1_channel, GPIO.LOW)
        self.cur_fan_on = False
        self.get_logger().info('Fan OFF')

    # is the ball in the collection area?
    def ball_in_area(self, relX, relY):
        return (abs(relX) < 30 and abs(relY) < 40)

    # updates when input from cam is new
    def update(self, data):
        ball_detected = int(data.data[2])
        relX = int(data.data[0])
        relY = int(data.data[1])

        # is a ball being detcted?
        if (ball_detected):
            # TODO: NAVIGATE TO BALL

            self.cur_ball_detected = True

            # is the fan off, and is the ball in the area?
            if (self.cur_fan_on == False and self.ball_in_area(relX,relY)):
                self.fan_on()

            # sleep(5)
        # was a ball being detected before?
        elif (self.cur_ball_detected == True and ball_detcted == False):
            self.cur_ball_detected = False
            self.fan_off()

            # did the ball disappear in the area
            if self.ball_in_area(relX,relY):
                # TODO: PUBLISH NEW STATE: SEARCHING. INCREMENT BALLS COLLETED  [state,increment?]
                self.pub_data.data = self.pub_data.data + 1
                self.num_balls.publish(self.pub_data)

            

            

def main(args=None):
    rclpy.init(args=args)
    adafruit_fan = AdafruitFan()
    try:
        rclpy.spin(adafruit_fan)
    except KeyboardInterrupt:
        GPIO.output(int(13), GPIO.LOW)
        adafruit_fan.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        adafruit_fan.destroy_node()
        rclpy.shutdown()
        adafruit_fan.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()