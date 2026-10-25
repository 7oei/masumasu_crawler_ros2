"""Hardware-free launch smoke test. Run in an isolated ROS_DOMAIN_ID."""
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


def spin_until(node, condition, seconds=8.):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        if condition():
            return
    raise AssertionError('ROS condition not met before timeout')


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '187', 'Use isolated ROS_DOMAIN_ID=187'
    rclpy.init()
    node = rclpy.create_node('crawler_integration_probe')
    commands, odometry = [], []
    node.create_subscription(Twist, 'cmd_vel', commands.append, 10)
    node.create_subscription(Odometry, 'odom', odometry.append, 10)
    joy = node.create_publisher(Joy, 'joy', 1)
    feedback = node.create_publisher(Float64MultiArray, 'odom_buf', 1)
    try:
        for mode in ('command', 'teleop'):
            commands.clear()
            proc = subprocess.Popen([
                'ros2', 'launch', 'masumasu_crawler', 'crawler.launch.py',
                'mode:='+mode, 'start_driver:=false', 'joy_device_id:=9999'],
                start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            try:
                spin_until(node, lambda: feedback.get_subscription_count() == 1)
                if mode == 'command':
                    assert node.count_publishers('cmd_vel') == 0
                    assert joy.get_subscription_count() == 0
                else:
                    spin_until(node, lambda: joy.get_subscription_count() == 1)
                    msg = Joy(axes=[0., 1., 0., 0.])
                    deadline = time.monotonic() + 1.
                    while time.monotonic() < deadline:
                        joy.publish(msg)
                        rclpy.spin_once(node, timeout_sec=.02)
                    assert any(msg.linear.x > .2 for msg in commands)
                    commands.clear()
                    spin_until(node, lambda: len(commands) > 70)
                    assert commands[-1].linear.x == commands[-1].angular.z == 0.
                feedback.publish(Float64MultiArray(data=[1., 2., .1, .2, .3, 0., 40., 0., 40., 5., 123.]))
                spin_until(node, lambda: bool(odometry))
                assert odometry[-1].header.stamp.sec == 123
                assert odometry[-1].pose.pose.position.x == 1.
                count = len(odometry)
                deadline = time.monotonic() + .3
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                assert len(odometry) == count, 'stale samples must not be republished'
                odometry.clear()
                print(mode + ': PASS')
            finally:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                spin_until(node, lambda: feedback.get_subscription_count() == 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
