#!/usr/bin/env python3
"""
Replay hex/JSONL log file to ROS2 topics as UInt8MultiArray.
Usage: replay_logs --file path/to/log --topic /v2i/map/raw --rate 5

Supports lines that are either raw hex strings (no prefix) or JSON objects
with a 'hex' key containing the hex string.
"""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


def read_lines(path):
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            # try json
            try:
                obj = json.loads(s)
                if isinstance(obj, dict):
                    # look for common keys
                    for k in ("hex", "raw", "data", "raw_hex"):
                        if k in obj and isinstance(obj[k], str):
                            yield obj[k]
                            break
                    else:
                        # fallback: if there's a single string field
                        for v in obj.values():
                            if isinstance(v, str) and all(c in "0123456789abcdefABCDEF" for c in v.strip()):
                                yield v.strip()
                                break
                else:
                    # not a dict -> ignore
                    continue
            except Exception:
                # not json, treat as raw hex
                yield s


class ReplayNode(Node):
    def __init__(self, topic, lines, rate_hz, once=False):
        super().__init__("v2i_replay")
        self.pub = self.create_publisher(UInt8MultiArray, topic, 10)
        self.lines = list(lines)
        self.rate_hz = rate_hz
        self.once = once

    def run(self):
        if self.rate_hz <= 0:
            delay = 0.0
        else:
            delay = 1.0 / float(self.rate_hz)

        first = True
        while rclpy.ok():
            for hexstr in self.lines:
                b = bytes.fromhex(hexstr)
                msg = UInt8MultiArray()
                msg.data = list(b)
                self.pub.publish(msg)
                self.get_logger().info(f"published {len(b)} bytes to topic")
                time.sleep(delay)
            if self.once:
                break
            # loop again
        return


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--file", "-f", required=True)
    p.add_argument("--topic", "-t", default="/v2i/map/raw")
    p.add_argument("--rate", "-r", type=float, default=5.0)
    p.add_argument("--once", action="store_true", help="publish file once then exit")
    args = p.parse_args()

    lines = list(read_lines(args.file))
    if not lines:
        print("No hex lines found in file")
        return

    rclpy.init()
    node = ReplayNode(args.topic, lines, args.rate, once=args.once)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
