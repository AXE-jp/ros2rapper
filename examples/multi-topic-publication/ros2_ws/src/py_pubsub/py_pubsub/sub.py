import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class StrSubscriber(Node):

    def __init__(self):
        super().__init__('str_subscriber')
        qos_profile = QoSProfile(depth=0, history=HistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription_0 = self.create_subscription(String, 'bbb', lambda msg: self.listener_callback('bbb', msg), qos_profile)
        self.subscription_1 = self.create_subscription(String, 'ccc', lambda msg: self.listener_callback('ccc', msg), qos_profile)
        self.subscription_2 = self.create_subscription(String, 'ddd', lambda msg: self.listener_callback('ddd', msg), qos_profile)
        self.subscription_3 = self.create_subscription(String, 'eee', lambda msg: self.listener_callback('eee', msg), qos_profile)
        self.subscription_0
        self.subscription_1
        self.subscription_2
        self.subscription_3

    def listener_callback(self, topic_name, msg):
        self.get_logger().info(f'Received[{topic_name}]: "{msg.data}"')


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
