"""Publish measured odometry only when a new encoder sample arrives."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('crawler_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.pub = self.create_publisher(Odometry, 'odom', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, 'odom_buf', self.on_sample, 1)

    def on_sample(self, msg):
        if len(msg.data) < 11 or not all(math.isfinite(v) for v in msg.data) or msg.data[10] < 0:
            return
        x, y, yaw, linear, angular = msg.data[:5]
        odom = Odometry()
        odom.header.stamp = Time(nanoseconds=int(msg.data[10]*1e9)).to_msg()
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = linear, angular
        self.pub.publish(odom)
        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
