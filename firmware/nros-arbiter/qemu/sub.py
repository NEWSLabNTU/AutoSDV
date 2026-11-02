"""Subscribes to /vehicle/arbiter/mode for N seconds; prints each mode change.

Used by e2e.sh instead of `ros2 topic echo`, which asks the ROS 2 daemon for
the publisher's QoS and so reaches whatever daemon this machine already runs,
configured for a different router.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    seconds = float(sys.argv[1])
    rclpy.init()
    node = Node("arbiter_mode_probe")
    start = time.monotonic()
    state = {"last": None, "count": 0}

    def on_mode(msg: String) -> None:
        state["count"] += 1
        if msg.data != state["last"]:
            print(f"{time.monotonic() - start:6.2f} s  {msg.data}", flush=True)
            state["last"] = msg.data

    node.create_subscription(String, "/vehicle/arbiter/mode", on_mode, 10)
    while time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"messages: {state['count']}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
