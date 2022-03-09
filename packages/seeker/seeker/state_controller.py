import rclpy 
from rclpy.node import Node
from std_msgs.msg import String


class StateController(Node):
    def __init__(self) -> None:
        super().__init__('state_controller')
        self.state_publisher = self.create_publisher(String, '/state', 10)
        self.create_timer(0.2, self.next_state)

    def next_state(self):
        self.get_logger().info("Hello ROS2")


def main(args=None):
    rclpy.init(args=args)
    controller = StateController()
    rclpy.spin(controller)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()