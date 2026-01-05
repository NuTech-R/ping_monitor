#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess
import re
import threading

class PingMonitorNode(Node):
    def __init__(self):
        super().__init__('ping_monitor_node')
        
        self.declare_parameter('target_host', '192.168.1.1') 
        self.declare_parameter('ping_interval', 1.0)
        
        self.target_host = self.get_parameter('target_host').value
        self.ping_interval = self.get_parameter('ping_interval').value
        
        self.latency_pub = self.create_publisher(Float32, 'ping_latency', 10)
        self.packet_loss_pub = self.create_publisher(Float32, 'ping_packet_loss', 10)
        
        self._is_pinging = False
        self.timer = self.create_timer(self.ping_interval, self.timer_callback)
        self.get_logger().info(f'Ping monitor started for: {self.target_host}')

    def timer_callback(self):
        if self._is_pinging:
            return
        thread = threading.Thread(target=self._run_ping_task, daemon=True)
        thread.start()

    def _run_ping_task(self):
        self._is_pinging = True
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', self.target_host],
                capture_output=True,
                text=True,
                timeout=2.0 
            )
            
            if result.returncode == 0:
                latency = self.parse_latency(result.stdout)
                if latency is not None:
                    self.publish_metrics(latency, 0.0)
            else:
                self.handle_failure()

        except subprocess.TimeoutExpired:
            self.get_logger().warn('Ping timeout')
            self.handle_failure()
            
        except Exception as e:
            self.get_logger().error(f'Ping error: {e}')
            self.handle_failure()
            
        finally:
            self._is_pinging = False

    def handle_failure(self):
        self.publish_metrics(-1.0, 100.0)

    def publish_metrics(self, latency, loss):
        msg_lat = Float32()
        msg_lat.data = latency
        self.latency_pub.publish(msg_lat)

        msg_loss = Float32()
        msg_loss.data = loss
        self.packet_loss_pub.publish(msg_loss)

    def parse_latency(self, output):
        match = re.search(r'time=([\d.]+)', output)
        if match:
            return float(match.group(1))
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PingMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()