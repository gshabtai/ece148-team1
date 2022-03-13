import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
from time import time

STATE = {
    'idle':                         'idle',
    'collision_avoidance':          'collision_avoidance',
    'search_mode':                  'search_mode',
    'navigate':                     'navigate',
    'collect_ball':                 'collect_ball',
    'drive_back':                   'drive_back'
}

NODE_NAME = 'state_machine_node'
STATE_TPOIC_NAME = '/state'
COLLISION_TOPIC_NAME = '/collision_avoidance_state'
WEBCAM_CEN_TOPIC_NAME = '/webcam_centroid'

class StateController(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.state_publisher = self.create_publisher(String, STATE_TPOIC_NAME, 10)
        self.collision_avoidance_state = self.create_subscription(Bool,COLLISION_TOPIC_NAME, self.collison_update, 10)
        self.webcam_subscriber = self.create_subscription(Float64MultiArray, WEBCAM_CEN_TOPIC_NAME, self.set_webcam_sees_ball, 10)
        self.num_ball_subscriber = self.create_subscription(Int8, '/num_ball_picked_up', self.set_num_balls, 10)
        self.create_timer(0.2, self.update)
        self.current_state = 'idle'
        self.next_state = 'idle'
        self.msg = String()
        self.time_threshold = 2
        self.ball_lost_time = 0 # Init time var

        # Set starting params
        self.number_loaded_ball = 0
        self.proposed_num_collected_balls = 0
        self.imminent_collision = False
        self.webcam_sees_ball = False

    def set_num_balls(self, data):
        self.proposed_num_collected_balls = data.data

    def set_webcam_sees_ball(self, obj):
        self.webcam_sees_ball = obj.data[2] == 1.0

    def collison_update(self,data):
        self.imminent_collision = data.data

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

    def calc_next_state(self):

        ########## ON IDLE ##########
        if self.current_state == STATE['idle']:
            if self.number_loaded_ball < 4:
                return STATE['search_mode']
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
                    return STATE['search_mode']
        
        ########## ON SEARCH MODE ##########
        elif self.current_state == STATE['search_mode']:
            if self.imminent_collision:
                return STATE['collision_avoidance']
            elif self.webcam_sees_ball:
                return STATE['collect_ball']
            else:
                return STATE['search_mode']

        ########## ON COLLECT BALL MODE ##########
        elif self.current_state == STATE['collect_ball']:
            if self.number_loaded_ball >= 4:
                return STATE['idle']
            elif self.imminent_collision:
                return STATE['collision_avoidance']
            elif self.number_loaded_ball != self.proposed_num_collected_balls: # Success in ball collections
                self.number_loaded_ball = self.proposed_num_collected_balls
                return STATE['search_mode']
            elif not self.webcam_sees_ball:
                self.ball_lost_time = time() # Start time
                return STATE['drive_back']
            else:
                return STATE['collect_ball']
            
        
        ########## ON DRIVE BACK MODE ##########
        elif self.current_state == STATE['drive_back']:
            if abs(time()-self.ball_lost_time) > self.time_threshold: # Ball has been lost for this much time
                return STATE['search_mode']
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