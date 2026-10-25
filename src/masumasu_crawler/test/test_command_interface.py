import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist
from masumasu_crawler.teleop import joystick_target, Teleop
from masumasu_crawler.armadillo_driver import Abot04TwistDriver
from test_power_telemetry import driver


@pytest.mark.parametrize('age', [.5, 1., 100.])
def test_command_loss_sends_zero_even_while_driver_keeps_running(monkeypatch, age):
    d = driver(True)
    d.last_cmd_vel_time = 10.
    d.target_angular_vel = .2
    monkeypatch.setattr('masumasu_crawler.armadillo_driver.time.monotonic', lambda: 10.+age)
    d.control()
    d._write_track_velocities.assert_called_once_with(0., 0.)


def test_no_command_cannot_arm():
    d = driver(False)
    d.last_cmd_vel_time = None
    d.target_linear_vel = .3
    d.control()
    d._activate_odrives.assert_not_called()


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', ['linear', 'angular'])
def test_invalid_command_stops_and_does_not_refresh_watchdog(value, axis):
    d = driver(True)
    msg = Twist()
    if axis == 'linear':
        msg.linear.x = value
    else:
        msg.angular.z = value
    d.twist_callback(msg)
    assert d.last_cmd_vel_time is None
    assert d.target_linear_vel == d.target_angular_vel == 0.


def test_fresh_command_can_resume_after_timeout():
    d = driver(True)
    d.last_cmd_vel_time = None
    d.control()
    msg = Twist()
    msg.linear.x = .2
    d.twist_callback(msg)
    d.control()
    d._write_track_velocities.assert_called_with(-.2, .2)


def test_ps3_mapping_and_optional_hold_to_run():
    assert joystick_target([0., 1., 0., -.5], [], 1, 3, -1, .47, .5) == (.47, -.25)
    assert joystick_target([0., 1., 0., 0.], [0], 1, 3, 0, .47, .5) is None
    assert joystick_target([0., 1., 0., 0.], [1], 1, 3, 0, .47, .5) == (.47, 0.)
    assert joystick_target([], [], 1, 3, -1, .47, .5) is None
    assert joystick_target([math.nan, 0.], [], 0, 1, -1, .47, .5) is None


def test_joy_loss_stops_without_slow_ramp(monkeypatch):
    d = SimpleNamespace(target=(.47, .5), last_rx=10., previous=(.47, .5),
                        settings={'joy_timeout': .5}, pub=Mock())
    monkeypatch.setattr('masumasu_crawler.teleop.time.monotonic', lambda: 11.)
    Teleop.tick(d)
    msg = d.pub.publish.call_args.args[0]
    assert msg.linear.x == msg.angular.z == 0.


def test_activation_stays_zero_until_next_freshness_check():
    d = driver(False)
    d.odrv0 = d.odrv1 = Mock()
    d.odrv0.read_float.return_value = 8
    d._ascii_operation = lambda label, op: op()
    d.closed_loop_timeout = 2.
    d._reset_encoder_reference = Mock()
    d._update_power_telemetry = Mock()
    d._enable_watchdog_test = Mock()
    d.watchdog_test_enabled = True
    d.get_logger = Mock()
    Abot04TwistDriver._activate_odrives(d, .2, -.2)
    assert all(call.args == (0., 0.) for call in d._write_track_velocities.call_args_list)
    d._enable_watchdog_test.assert_called_once()
