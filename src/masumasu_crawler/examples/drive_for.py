#!/usr/bin/env python3
"""Bounded ROS command publisher. Requires command-mode bringup."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--linear', type=float, default=0.)
    parser.add_argument('--angular', type=float, default=0.)
    parser.add_argument('--seconds', type=float, default=1.)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.linear, args.angular, args.seconds)) or args.seconds <= 0:
        parser.error('values must be finite and seconds must be positive')
    rclpy.init(args=[])
    node = rclpy.create_node('crawler_command_example')
    pub = node.create_publisher(Twist, 'cmd_vel', 1)
    try:
        deadline = time.monotonic() + 5.
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No cmd_vel subscriber; start the driver in command mode')
        msg = Twist()
        msg.linear.x, msg.angular.z = args.linear, args.angular
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
