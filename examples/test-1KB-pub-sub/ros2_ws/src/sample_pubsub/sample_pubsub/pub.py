import rclpy
from rclpy.node import Node
from sample_msgs.msg import Uint16x512


class SamplePublisher(Node):

    def __init__(self):
        super().__init__("sample_publisher")
        self.publisher_ = self.create_publisher(Uint16x512, "sample_topic", 10)
        self.timer_ = self.create_timer(1, self.timer_callback)
        self.counter_ = 0

    def timer_callback(self):
        n = self.counter_
        self.counter_ += 1

        msg = Uint16x512()
        msg.data = [ n + i for i in range(512) ]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publish: {self.counter_}")


def main(args=None):
    rclpy.init(args=args)
    sample_publisher = SamplePublisher()
    rclpy.spin(sample_publisher)
    sample_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
