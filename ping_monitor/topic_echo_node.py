#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


class TopicEchoNode(Node):
    """link_ping/request を受けたらそのまま link_ping/response に返すノード(応答側)。

    計測側のtopic_ping_nodeと対で使う。
    """

    def __init__(self):
        super().__init__('topic_echo_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.response_pub = self.create_publisher(Header, 'link_ping/response', qos)
        self.request_sub = self.create_subscription(
            Header, 'link_ping/request', self.request_callback, qos)

        self.get_logger().info('Topic echo started')

    def request_callback(self, msg):
        self.response_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicEchoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
