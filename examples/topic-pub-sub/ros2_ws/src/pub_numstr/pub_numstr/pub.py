import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NumStrPublisher(Node):

    def __init__(self):
        super().__init__('numstr_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '%d' % (self.i % 10)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    numstr_publisher = NumStrPublisher()

    rclpy.spin(numstr_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    numstr_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
