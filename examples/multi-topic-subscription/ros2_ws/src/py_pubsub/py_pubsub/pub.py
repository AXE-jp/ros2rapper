from rclpy.node import Node
from std_msgs.msg import String


class NumStrPublisher(Node):

    def __init__(self, topic_name, char):
        super().__init__('numstr_publisher')
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.char = char
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = self.char * (self.i % 10 + 1)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
