"""Select one command source, with a hardware-free interface inspection mode."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('masumasu_crawler')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='teleop', choices=['teleop', 'command']),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'crawler.yaml')),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('enable_button', default_value='-1'),
        DeclareLaunchArgument('right_port', default_value='/dev/odrive-right'),
        DeclareLaunchArgument('left_port', default_value='/dev/odrive-left'),
        Node(package='masumasu_crawler', executable='driver', name='crawler_driver',
             output='screen', respawn=True, respawn_delay=2.,
             condition=IfCondition(LaunchConfiguration('start_driver')),
             parameters=[config, {
                 'odrive0_usb_path': LaunchConfiguration('right_port'),
                 'odrive1_usb_path': LaunchConfiguration('left_port')}]),
        Node(package='masumasu_crawler', executable='odometry', name='crawler_odometry',
             output='screen', parameters=[config]),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             condition=LaunchConfigurationEquals('mode', 'teleop'),
             parameters=[{'device_id': ParameterValue(LaunchConfiguration('joy_device_id'), value_type=int),
                          'autorepeat_rate': 20., 'deadzone': 0.05, 'sticky_buttons': False}]),
        Node(package='masumasu_crawler', executable='teleop', name='crawler_teleop',
             output='screen', condition=LaunchConfigurationEquals('mode', 'teleop'),
             parameters=[config, {'enable_button': ParameterValue(
                 LaunchConfiguration('enable_button'), value_type=int)}]),
    ])
