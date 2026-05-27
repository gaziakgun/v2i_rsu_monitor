#!/usr/bin/env python3
"""
Headless test consumer for v2i topics. Subscribe to v2i/sdsm/raw, v2i/map/raw, v2i/spat/raw
and attempt to decode messages with pycmssdk; prints counts and sample decoded keys.

Run after sourcing ROS2 and building/installing dependencies:

PYTHONPATH=. python3 v2i_rsu_monitor/test_consumer.py

or after colcon build/source:
ros2 run v2i_rsu_monitor rsu_monitor  # GUI
# or for consumer:
PYTHONPATH=. python3 v2i_rsu_monitor/test_consumer.py

"""
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import UInt8MultiArray
from pycmssdk.asn1 import Asn1Type, asn1_decode


class Consumer(Node):
    def __init__(self):
        super().__init__('v2i_test_consumer')
        self.counts = {'SDSM': 0, 'MAP': 0, 'SPAT': 0}
        self.samples = {'SDSM': None, 'MAP': None, 'SPAT': None}
        self.lock = threading.Lock()
        raw_topic_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            UInt8MultiArray, 'v2i/sdsm/raw', lambda m: self.cb(m, 'SDSM'), raw_topic_qos
        )
        self.create_subscription(
            UInt8MultiArray, 'v2i/map/raw', lambda m: self.cb(m, 'MAP'), raw_topic_qos
        )
        self.create_subscription(
            UInt8MultiArray, 'v2i/spat/raw', lambda m: self.cb(m, 'SPAT'), raw_topic_qos
        )

        self.timer = self.create_timer(5.0, self.report)

    def cb(self, msg, kind):
        try:
            payload = bytes(msg.data)
            decoded = asn1_decode(payload, Asn1Type.US_MESSAGE_FRAME)
        except Exception as e:
            decoded = None
        with self.lock:
            self.counts[kind] += 1
            if self.samples[kind] is None and decoded is not None:
                # store a small summary of decoded
                if isinstance(decoded, dict):
                    keys = list(decoded.keys())[:8]
                else:
                    keys = [str(type(decoded))]
                self.samples[kind] = keys

    def report(self):
        with self.lock:
            self.get_logger().info(f"counts: {self.counts}")
            self.get_logger().info(f"samples: {self.samples}")


def main():
    rclpy.init()
    node = Consumer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
