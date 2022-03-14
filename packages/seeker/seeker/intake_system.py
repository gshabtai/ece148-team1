from time import sleep, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8, String
import RPi.GPIO as GPIO
import math

NODE_NAME = 'intake_system_node'
WEBCAM_CEN_TOPIC_NAME = '/webcam_centroid'
STATE_TOPIC_NAME = '/state'
BALL_TOPIC_NAME = '/num_ball_picked_up'

class IntakeProcess(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # update called every time topic publishes new value
        self.webcam_subscriber = self.create_subscription(Float64MultiArray, WEBCAM_CEN_TOPIC_NAME, self.update, 10)
        self.state_subscrition = self.create_subscription(String, STATE_TOPIC_NAME, self.set_state, 10)
        self.num_balls = self.create_publisher(Int8, BALL_TOPIC_NAME, 10)

        self.pub_data = Int8()
        self.pub_data.data = 0
        self.tracking_ball = False

        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', int(1)),
                ('fan_channel', int(13)),
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.fan_channel = int(self.get_parameter('fan_channel').value)
        
        self.cur_fan_on = False     # keeps track if the fan is on
        self.state = 'idle'

        #Unused
        self.prev_in_rect = False # keeps track if a ball has been detected
        self.prev_ball_relX = 0
        self.prev_ball_relY = 0


        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.fan_channel, GPIO.OUT)

    def set_state(self, data):
        self.state = data.data

    # turn on fan
    def fan_on(self):
        if not self.cur_fan_on:
            GPIO.output(self.fan_channel, GPIO.HIGH)
            self.cur_fan_on = True
            self.get_logger().info('Fan ON')

    # turn off fan
    def fan_off(self):
        if self.cur_fan_on:
            GPIO.output(self.fan_channel, GPIO.LOW)
            self.cur_fan_on = False
            self.get_logger().info('Fan OFF')

    # publishes that it picked up a ball
    def pickup_success(self):
        self.pub_data.data = self.pub_data.data + 1
        self.num_balls.publish(self.pub_data)

    # is the ball in the collection area?
    def ball_in_area(self, relX, relY, ball_detected):
        if (ball_detected):
            return (abs(relX) < 30 and abs(relY) < 40)
        else:
            return False

    # updates when input from cam is new
    def update(self, data):
        '''Update detection of ball based on centroid to switch fans on/off'''

        if self.state != 'collect_ball':
            return

        ball_detected = int(data.data[2])
        relX = int(data.data[0])
        relY = int(data.data[1])

        #Capture ball
        if (ball_detected and self.ball_in_area(relX, relY, ball_detected)):
            self.fan_on()
            if not self.tracking_ball:
                self.timer = time()
            self.tracking_ball = True

        #Ball Capture success
        elif (not self.ball_in_area(relX, relY, ball_detected) and self.tracking_ball):
            sleep(1)
            self.fan_off()

            self.tracking_ball = False
            # Why wait a second and why > 5 for success?
            if abs(time()-self.timer) > 2:
                self.pickup_success()


        # # is a ball being detcted?
        # if (ball_detected):
        #     # is the fan off, and is the ball in the area?
        #     if (self.cur_fan_on == False and self.ball_in_area(relX,relY)):
        #         self.fan_on()

        #     # was the ball previously in the area, and did its centroid jump?
        #     JUMP_SENSITIVITY = 5 # if too low, try 10
        #     p1 = (relX,relY)
        #     p2 = (self.prev_ball_relX,self.prev_ball_relY)
        #     if (self.ball_in_area(self.prev_ball_relX,self.prev_ball_relY) and math.dist(p1,p2) > 5):
        #         self.pickup()

        #     self.prev_ball_detected = True
        #     self.prev_ball_relX = relX
        #     self.prev_ball_relY = relY

        #     # sleep(5)
        # # was a ball being detected before?
        # elif (self.prev_ball_detected == True and ball_detected == False):
        #     self.prev_ball_detected = False
        #     self.fan_off()

        #     # did the ball disappear in the area
        #     if self.ball_in_area(relX,relY):
        #         self.pickup()
        
        # # is there no ball detected, and fan is on?
        # elif (ball_detected == False and self.cur_fan_on == True):
        #     self.fan_off()

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