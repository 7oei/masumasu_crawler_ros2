"""Joystick-only input; command mode never starts this node."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


def joystick_target(axes, buttons, linear_axis, angular_axis, enable_button,
                    max_linear, max_angular):
    if enable_button >= 0 and (
            enable_button >= len(buttons) or not buttons[enable_button]):
        return None
    if any(i < 0 or i >= len(axes) for i in (linear_axis, angular_axis)):
        return None
    values = axes[linear_axis], axes[angular_axis]
    if not all(math.isfinite(v) and abs(v) <= 1.01 for v in values):
        return None
    return tuple(max(-1., min(1., v)) * scale
                 for v, scale in zip(values, (max_linear, max_angular)))


class Teleop(Node):
    def __init__(self):
        super().__init__('crawler_teleop')
        defaults = dict(linear_axis=1, angular_axis=3, enable_button=-1,
                        max_linear=0.47, max_angular=0.5,
                        joy_timeout=0.5, acceleration_rate=40.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key in ('max_linear', 'max_angular', 'joy_timeout', 'acceleration_rate'):
            value = self.settings[key]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(key + ' must be finite and positive')
        self.target = None
        self.last_rx = None
        self.previous = (0., 0.)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Joy, 'joy', self.on_joy, 1)
        self.create_timer(0.01, self.tick)

    def on_joy(self, msg):
        self.last_rx = time.monotonic()
        self.target = joystick_target(msg.axes, msg.buttons, **{
            key: self.settings[key] for key in ('linear_axis', 'angular_axis',
                                               'enable_button', 'max_linear', 'max_angular')})

    def tick(self):
        valid = (self.target is not None and self.last_rx is not None and
                 time.monotonic() - self.last_rx < self.settings['joy_timeout'])
        result = (0., 0.)
        if valid:
            steps = (self.settings['max_linear'] / self.settings['acceleration_rate'],
                     self.settings['max_angular'] / self.settings['acceleration_rate'])
            result = tuple(old + max(-step, min(step, target-old))
                           for old, target, step in zip(self.previous, self.target, steps))
        msg = Twist()
        msg.linear.x, msg.angular.z = result
        self.pub.publish(msg)
        self.previous = result


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
