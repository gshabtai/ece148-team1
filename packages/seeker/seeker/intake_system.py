from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
import RPi.GPIO as GPIO
import math

NODE_NAME = 'intake_system_node'
TOPIC_NAME = '/webcam_centroid'

class IntakeProcess(Node):
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
        self.prev_ball_detected = False # keeps track if a ball has been detected
        self.prev_ball_relX = 0
        self.prev_ball_relY = 0


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

            # is the fan off, and is the ball in the area?
            if (self.cur_fan_on == False and self.ball_in_area(relX,relY)):
                self.fan_on()

            # was the ball previously in the area, and did its centroid jump?
            JUMP_SENSITIVITY = 5 # if too low, try 10
            p1 = (relX,relY)
            p2 = (self.prev_ball_relX,self.prev_ball_relY)
            if (self.ball_in_area(self.prev_ball_relX,self.prev_ball_relY) and math.dist(p1,p2) > 5):
                self.pub_data.data = self.pub_data.data + 1
                self.num_balls.publish(self.pub_data)

            self.prev_ball_detected = True
            self.prev_ball_relX = relX
            self.prev_ball_relY = relY

            # sleep(5)
        # was a ball being detected before?
        elif (self.prev_ball_detected == True and ball_detected == False):
            self.prev_ball_detected = False
            self.fan_off()

            # did the ball disappear in the area
            if self.ball_in_area(relX,relY):
                # TODO: PUBLISH NEW STATE: SEARCHING. INCREMENT BALLS COLLETED  [state,increment?]
                self.pub_data.data = self.pub_data.data + 1
                self.num_balls.publish(self.pub_data)

def main(args=None):
    rclpy.init(args=args)
    intake_system = IntakeProcess()
    try:
        rclpy.spin(intake_system)
    except KeyboardInterrupt:
        GPIO.output(int(13), GPIO.LOW)
        intake_system.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        intake_system.destroy_node()
        rclpy.shutdown()
        intake_system.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()