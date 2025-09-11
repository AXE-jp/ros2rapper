from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class StrSubscriber(Node):

    def __init__(self, topic_name):
        super().__init__(f'str_subscriber_{topic_name}')
        qos_profile = QoSProfile(depth=0, history=HistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.topic_name_ = topic_name
        self.subscription_ = self.create_subscription(String, topic_name, self.listener_callback, qos_profile)
        self.subscription_

    def listener_callback(self, msg):
        self.get_logger().info(f'Received[{self.topic_name_}]: "{msg.data}"')
