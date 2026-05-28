#!/usr/bin/env python3
"""
Headless test consumer for v2i topics. Subscribe to v2i/sdsm/raw, v2i/map/raw,
v2i/spat/raw using the custom V2I message packages when available, or raw
UInt8MultiArray ASN.1 bytes as a fallback. Prints counts and sample summaries.

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
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import UInt8MultiArray
from pycmssdk.asn1 import Asn1Type, asn1_decode

try:
    from v2i_map_msgs.msg import MapData
except ImportError:
    MapData = None

try:
    from v2i_sdsm_msgs.msg import SDSM
except ImportError:
    SDSM = None

try:
    from v2i_spat_msgs.msg import SpatPacket
except ImportError:
    SpatPacket = None


class Consumer(Node):
    def __init__(self):
        super().__init__('v2i_test_consumer')
        self.counts = {'SDSM': 0, 'MAP': 0, 'SPAT': 0}
        self.samples = {'SDSM': None, 'MAP': None, 'SPAT': None}
        self.lock = threading.Lock()
        raw_topic_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        map_topic_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        if SDSM is not None:
            self.create_subscription(
                SDSM, 'v2i/sdsm/raw', lambda m: self.cb_typed(m, 'SDSM'), raw_topic_qos
            )
            self.get_logger().info('Subscribed to v2i/sdsm/raw as v2i_sdsm_msgs/msg/SDSM')
        else:
            self.create_subscription(
                UInt8MultiArray, 'v2i/sdsm/raw', lambda m: self.cb_raw(m, 'SDSM'), raw_topic_qos
            )
            self.get_logger().warn('v2i_sdsm_msgs unavailable; using UInt8MultiArray fallback')

        if MapData is not None:
            self.create_subscription(
                MapData, 'v2i/map/raw', lambda m: self.cb_typed(m, 'MAP'), map_topic_qos
            )
            self.get_logger().info('Subscribed to v2i/map/raw as v2i_map_msgs/msg/MapData')
        else:
            self.create_subscription(
                UInt8MultiArray, 'v2i/map/raw', lambda m: self.cb_raw(m, 'MAP'), raw_topic_qos
            )
            self.get_logger().warn('v2i_map_msgs unavailable; using UInt8MultiArray fallback')

        if SpatPacket is not None:
            self.create_subscription(
                SpatPacket, 'v2i/spat/raw', lambda m: self.cb_typed(m, 'SPAT'), raw_topic_qos
            )
            self.get_logger().info('Subscribed to v2i/spat/raw as v2i_spat_msgs/msg/SpatPacket')
        else:
            self.create_subscription(
                UInt8MultiArray, 'v2i/spat/raw', lambda m: self.cb_raw(m, 'SPAT'), raw_topic_qos
            )
            self.get_logger().warn('v2i_spat_msgs unavailable; using UInt8MultiArray fallback')

        self.timer = self.create_timer(5.0, self.report)

    def cb_raw(self, msg, kind):
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

    def cb_typed(self, msg, kind):
        if kind == 'MAP':
            sample = {
                'type': 'v2i_map_msgs/msg/MapData',
                'intersections': len(getattr(msg, 'intersections', [])),
            }
        elif kind == 'SDSM':
            sample = {
                'type': 'v2i_sdsm_msgs/msg/SDSM',
                'objects': len(getattr(msg, 'objects', [])),
            }
        elif kind == 'SPAT':
            spat = getattr(msg, 'spat', None)
            sample = {
                'type': 'v2i_spat_msgs/msg/SpatPacket',
                'intersections': len(getattr(spat, 'intersections', [])),
            }
        else:
            sample = {'type': str(type(msg))}

        with self.lock:
            self.counts[kind] += 1
            if self.samples[kind] is None:
                self.samples[kind] = sample

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
