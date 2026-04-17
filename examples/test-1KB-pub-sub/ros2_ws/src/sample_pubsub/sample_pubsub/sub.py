import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from sample_msgs.msg import Uint16x512


class SampleSubscriber(Node):

    def __init__(self):
        super().__init__("sample_subscriber")
        qos_profile = QoSProfile(depth=0, history=HistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.counter_ = 0
        self.subscription_ = self.create_subscription(Uint16x512, "sample_topic_b", self.listener_callback, qos_profile)
        self.subscription_

    def listener_callback(self, msg):
        self.counter_ += 1
        n = self.counter_

        base = msg.data[0]
        valid = True
        for i in range(1, 512):
            if msg.data[i] != (base + i):
                valid = False

        self.get_logger().info(f"Received {n}: {'Valid' if valid else 'Invalid'}")


def main(args=None):
    rclpy.init(args=args)
    sample_subscriber = SampleSubscriber()
    rclpy.spin(sample_subscriber)
    sample_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
