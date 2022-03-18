from re import S
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray, Int8
from time import time, sleep

STATE = {
    'idle':                         'idle',
    'collision_avoidance':          'collision_avoidance',
    'search':                       'search',
    'navigate':                     'navigate',
    'collect_ball':                 'collect_ball',
    'drive_back':                   'drive_back'
}

NODE_NAME = 'state_machine_node'
STATE_TPOIC_NAME = '/state'
COLLISION_TOPIC_NAME = '/collision_avoidance_state'
WEBCAM_CEN_TOPIC_NAME = '/webcam_centroid'
INTEL_CEN_TOPIC_NAME = '/intel_centroid'
BALL_TOPIC_NAME = '/num_ball_picked_up'
BALL_DISTANCE_TOPIC_NAME = '/ball_distance_bool'

class StateController(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.state_publisher = self.create_publisher(String, STATE_TPOIC_NAME, 10)
        self.collision_avoidance_state = self.create_subscription(Bool,COLLISION_TOPIC_NAME, self.collison_update, 10)
        self.webcam_subscriber = self.create_subscription(Float64MultiArray, WEBCAM_CEN_TOPIC_NAME, self.set_webcam_sees_ball, 10)
        self.webcam_subscriber = self.create_subscription(Float64MultiArray, INTEL_CEN_TOPIC_NAME, self.set_intel_sees_ball, 10)
        self.num_ball_subscriber = self.create_subscription(Int8, BALL_TOPIC_NAME, self.set_num_balls, 10)
        self.ball_distance_bool = self.create_subscription(Int8, BALL_DISTANCE_TOPIC_NAME, self.update_ball_distance, 10)
        self.create_timer(0.2, self.update)
        self.current_state = 'idle'
        self.next_state = 'idle'
        self.msg = String()
        self.time_threshold = 2
        self.ball_lost_time = time() # Init time var

        # Set starting params
        self.number_loaded_ball = 0
        self.proposed_num_collected_balls = 0
        self.imminent_collision = False
        self.webcam_sees_ball = False
        self.intel_sees_ball = False
        self.tracking = False

        self.declare_parameters(
            namespace='',
            parameters=[
                ('capture_full', 4),
            ])
        
        self.capture_full = self.get_parameter('capture_full').value

        self.get_logger().info(
            f'\ncapture_full: {self.capture_full}'
        )

    def set_num_balls(self, data):
        self.proposed_num_collected_balls = data.data

    def set_intel_sees_ball(self, obj):
        self.intel_sees_ball = obj.data[2] == 1.0

    def set_webcam_sees_ball(self, obj):
        self.webcam_sees_ball = obj.data[2] == 1.0

    def collison_update(self,data):
        self.imminent_collision = data.data

    def update_ball_distance(self, data):
        self.ball_distance_bool = data.data

    def update(self):
        if self.current_state != self.next_state: # Log to console if state has changed
            self.get_logger().info(f'Changing state from: {self.current_state} to {self.next_state}')
        
        # Calculate the next state
        self.next_state = self.calc_next_state()

        # Let all other nodes know that the state has change
        self.msg.data = self.next_state
        self.state_publisher.publish(self.msg)

        # Set next_state as current_state
        self.current_state = self.next_state

    # WORKING State Machine
    def calc_next_state(self):
        '''State Machine Controller'''
        
        ########## ON IDLE ##########
        if self.current_state == STATE['idle']:
            if self.number_loaded_ball < self.capture_full and abs(time()-self.ball_lost_time) > 20:
                return STATE['search']
            else:
                return STATE['idle']

        ########## ON SEARCH MODE ##########
        elif self.current_state == STATE['search']:
            if self.webcam_sees_ball:
                return STATE['collect_ball']
            elif self.intel_sees_ball:
                self.tracking = True
                return STATE['navigate']
            else:
                return STATE['search']

        ########## ON COLLECT BALL MODE ##########
        elif self.current_state == STATE['collect_ball']:
            if self.number_loaded_ball >= self.capture_full:
                return STATE['idle']
            elif self.number_loaded_ball != self.proposed_num_collected_balls: # Success in ball collections
                self.number_loaded_ball = self.proposed_num_collected_balls
                self.get_logger().info(f'Number of balls collected: {self.number_loaded_ball}')
                return STATE['search']
            elif not self.webcam_sees_ball:
                self.ball_lost_time = time() # Start time
                return STATE['search']
            else:
                return STATE['collect_ball']
                
        ########## ON NAVIGATE MODE ###########
        elif self.current_state == STATE['navigate']:
            if self.webcam_sees_ball and self.ball_distance_bool:
                return STATE['collect_ball']
            elif not self.intel_sees_ball and self.tracking:
                self.ball_lost_time = time() # Start time
                self.tracking = False
                return STATE['navigate']
            elif not self.intel_sees_ball and (time() - self.ball_lost_time) < 1:
                return STATE['navigate']
            elif not self.intel_sees_ball and (time() - self.ball_lost_time) > 1:
                return STATE['search']
            else:
                return STATE['navigate']

        ########## ON INVALID STATE, RETURN TO IDLE ##########
        else:
            return STATE['idle']

    #Goal State Machine
    def calc_next_state_goal(self):

        ########## ON IDLE ##########
        if self.current_state == STATE['idle']:
            if self.number_loaded_ball < 4 and abs(time()-self.ball_lost_time) > 20:
                return STATE['search']
            else:
                return STATE['idle']

        ########## ON COLLISION AVOIDANCE MODE ##########
        elif self.current_state == STATE['collision_avoidance']:
            if self.imminent_collision:
                return STATE['collision_avoidance']
            else:
                if self.webcam_sees_ball:
                    return STATE['collect_ball']
                else:
                    return STATE['search']
        
        ########## ON SEARCH MODE ##########
        elif self.current_state == STATE['search']:
            if self.imminent_collision:
                return STATE['collision_avoidance']
            elif self.webcam_sees_ball:
                return STATE['collect_ball']
            elif self.intel_sees_ball:
                return STATE['navigate']
            else:
                return STATE['search']

        ########## ON NAVIGATE MODE ###########
        elif  self.current_state == STATE['navigate']:
            if self.imminent_collision:
                return STATE['collision_avoidance']
            elif self.webcam_sees_ball:
                return STATE['collect_ball']
            elif not self.intel_sees_ball:
                self.ball_lost_time = time() # Start time
                return STATE['drive_back']
            else:
                return STATE['navigate']

        ########## ON COLLECT BALL MODE ##########
        elif self.current_state == STATE['collect_ball']:
            if self.number_loaded_ball >= 4:
                return STATE['idle']
            elif self.imminent_collision:
                return STATE['collision_avoidance']
            elif self.number_loaded_ball != self.proposed_num_collected_balls: # Success in ball collections
                self.number_loaded_ball = self.proposed_num_collected_balls
                return STATE['search']
            elif not self.webcam_sees_ball:
                self.ball_lost_time = time() # Start time
                return STATE['drive_back']
            else:
                return STATE['collect_ball']
            
        ########## ON DRIVE BACK MODE ##########
        elif self.current_state == STATE['drive_back']:
            if abs(time()-self.ball_lost_time) > self.time_threshold: # Ball has been lost for this much time
                return STATE['search']
            elif self.webcam_sees_ball:
                return STATE['collect_ball']
            else:
                return STATE['drive_back']
        
        ########## ON INVALID STATE, RETURN TO IDLE ##########
        else:
            return STATE['idle']


def main(args=None):
    rclpy.init(args=args)
    controller = StateController()
    rclpy.spin(controller)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()