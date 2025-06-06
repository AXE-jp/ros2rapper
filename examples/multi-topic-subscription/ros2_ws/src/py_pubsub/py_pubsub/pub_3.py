import rclpy
from py_pubsub.pub import NumStrPublisher


def main(args=None):
    rclpy.init(args=args)

    numstr_publisher = NumStrPublisher("eee", "E")

    rclpy.spin(numstr_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    numstr_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
