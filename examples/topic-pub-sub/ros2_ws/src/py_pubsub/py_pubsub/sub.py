import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class StrSubscriber(Node):

    def __init__(self):
        super().__init__('str_subscriber')
        self.subscription = self.create_subscription(String, 'aaa', self.listener_callback, 10)
        self.subscription

    def listener_callback(self):
        self.get_logger().info('Received: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    str_subscriber = StrSubscriber()

    rclpy.spin(str_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    str_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
