import rclpy 
from rclpy.node import Node
from std_msgs.msg import String


class StateController(Node):
    def __init__(self) -> None:
        super().__init__('state_controller')
        self.state_publisher = self.create_publisher(String, '/state', 10)
        self.create_timer(0.2, self.update)
        self.current_state = 'idle'
        self.next_state = 'noop'
        self.msg = String()

        # Set starting params
        self.number_loaded_ball = 0

    def update(self):
        self.next_state = self.calc_next_state()
        self.msg.data = self.next_state
        
        if self.current_state != self.next_state:
            self.get_logger().info(f'Changing state from: {self.current_state} to {self.next_state}')
        
        self.state_publisher.publish(self.msg)

        self.current_state = self.next_state

    def calc_next_state(self):
        if self.current_state == 'idle' and self.number_loaded_ball < 4:
            return 'search_mode'
        elif self.current_state == 'idle' and self.number_loaded_ball >= 4:
            return 'idle'
        
        if self.current_state == 'search_mode':
            return 'search_mode'

        # Default parameter
        return 'idle'


def main(args=None):
    rclpy.init(args=args)
    controller = StateController()
    rclpy.spin(controller)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()