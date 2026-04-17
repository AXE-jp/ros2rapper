import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class StrSubscriber(Node):

    def __init__(self):
        super().__init__('str_subscriber')
        qos_profile = QoSProfile(depth=0, history=HistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(String, 'bbb', self.listener_callback, qos_profile)
        self.subscription

    def listener_callback(self, msg):
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
