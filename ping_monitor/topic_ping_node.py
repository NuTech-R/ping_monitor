#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Header


class TopicPingNode(Node):
    """ROSトピックの往復時間で通信リンクの遅延・ロスを測るノード(計測側)。

    link_ping/request に載せたseqを計測対象側のtopic_echo_nodeがそのまま返し、
    受信までの時間をRTTとして ping_latency にpublishする。経路解決はrmw任せなので、
    LAN直結でもWAN(中継サーバ経由)でも実際のデータ経路そのものを測れる。
    送信・受信ともこのノードの時計だけで完結するため、対向との時刻同期は不要。
    """

    PING_INTERVAL = 0.5  # 秒
    TIMEOUT = 1.0  # 秒。WAN最悪RTTより十分大きく、これ超過はロスとみなす

    def __init__(self):
        super().__init__('topic_ping_node')

        # 実データと同じ土俵で測るためbest_effort(落ちたらロスとして観測する)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.request_pub = self.create_publisher(Header, 'link_ping/request', qos)
        self.response_sub = self.create_subscription(
            Header, 'link_ping/response', self.response_callback, qos)

        self.latency_pub = self.create_publisher(Float32, 'ping_latency', 10)
        self.packet_loss_pub = self.create_publisher(Float32, 'ping_packet_loss', 10)

        self.seq = 0
        self.pending = {}  # seq -> 送信時刻(ns)

        self.timer = self.create_timer(self.PING_INTERVAL, self.timer_callback)
        self.get_logger().info('Topic ping monitor started')

    def timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds

        # タイムアウトした要求はロス扱いにする
        for seq, sent_ns in list(self.pending.items()):
            if (now_ns - sent_ns) > self.TIMEOUT * 1e9:
                del self.pending[seq]
                self.publish_metrics(-1.0, 100.0)

        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.seq)
        self.pending[self.seq] = now_ns
        self.seq += 1
        self.request_pub.publish(msg)

    def response_callback(self, msg):
        try:
            seq = int(msg.frame_id)
        except ValueError:
            return
        sent_ns = self.pending.pop(seq, None)
        if sent_ns is None:
            return  # タイムアウト済みか重複
        rtt_ms = (self.get_clock().now().nanoseconds - sent_ns) * 1e-6
        self.publish_metrics(rtt_ms, 0.0)

    def publish_metrics(self, latency, loss):
        msg_lat = Float32()
        msg_lat.data = float(latency)
        self.latency_pub.publish(msg_lat)

        msg_loss = Float32()
        msg_loss.data = float(loss)
        self.packet_loss_pub.publish(msg_loss)


def main(args=None):
    rclpy.init(args=args)
    node = TopicPingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
