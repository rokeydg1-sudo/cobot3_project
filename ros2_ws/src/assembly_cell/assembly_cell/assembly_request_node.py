import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AssemblyRequestNode(Node):

    def __init__(self):
        super().__init__('assembly_request_node')

        self.publisher_ = self.create_publisher(
            String,
            '/assembly/request',
            10
        )

        self.get_logger().info(
            'Assembly Request Node started'
        )

    def send_request(self):

        msg = String()
        msg.data = 'need'

        self.publisher_.publish(msg)

        self.get_logger().info(
            'Published assembly request: need'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AssemblyRequestNode()

    try:
        input('Press ENTER to send request...')

        node.send_request()

        rclpy.spin_once(
            node,
            timeout_sec=0.5
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
