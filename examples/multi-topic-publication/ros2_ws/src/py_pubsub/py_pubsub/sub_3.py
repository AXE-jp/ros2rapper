import rclpy
from py_pubsub.sub import StrSubscriber


def main(args=None):
    rclpy.init(args=args)

    str_subscriber = StrSubscriber("eee")

    rclpy.spin(str_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    str_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
