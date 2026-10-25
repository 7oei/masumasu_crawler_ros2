from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray

from masumasu_crawler.power_telemetry import FeedbackPoll
from test_power_telemetry import ports, driver


def test_feedback_waits_for_both_complete_measured_responses(ports):
    devices, peers = ports
    poll = FeedbackPoll(devices, .15, now=0.)
    for device in devices:
        device.write.assert_called_once_with('f 0')
    peers[0].sendall(b'12.5 -0.2\n')
    peers[1].sendall(b'-5.2 ')
    assert poll.poll(now=.01) is None
    peers[1].sendall(b'0.3\r\n')
    assert poll.poll(now=.02) == ((12.5, -.2), (-5.2, .3))


@pytest.mark.parametrize('response', [b'39.0\n', b'nan 1\n', b'1 inf\n',
                                     b'1 2 3\n', b'1 2\n3 4\n', b'x y\n'])
def test_feedback_rejects_invalid_or_voltage_response(ports, response):
    devices, peers = ports
    poll = FeedbackPoll(devices, .15, now=0.)
    peers[0].sendall(response)
    with pytest.raises(ValueError):
        poll.poll(now=.01)


def test_feedback_missing_side_never_invents_zero_or_retries(ports):
    devices, peers = ports
    poll = FeedbackPoll(devices, .15, now=0.)
    peers[0].sendall(b'0 0\n')
    assert poll.poll(now=.1) is None
    with pytest.raises(TimeoutError):
        poll.poll(now=.15)
    assert all(d.write.call_count == 1 for d in devices)


def feedback_driver():
    d = driver(True)
    clock = SimpleNamespace(now=10.)
    d.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(clock.now*1e9)))
    d.start_time = Time(nanoseconds=0)
    d.odom_pub = Mock()
    d.wheel_travel_pub = Mock()
    d.forward_travel = [0., 0.]
    d.pose_x = d.pose_y = d.pose_yaw = 0.
    d.current0 = d.current1 = 0.
    d.tread = d.yaw_calibration = 1.
    d.last_feedback_publish = None
    return d, clock


def measured(d, clock, right, left, right_speed=0., left_speed=0.):
    d.feedback_stamp = clock.now
    replies = ((right, right_speed), (left, left_speed))
    d.feedback_poll = SimpleNamespace(poll=lambda: replies, values=replies)
    d.feedback_side_reported = [False, False]
    assert d._finish_feedback_poll()
    return d.odom_pub.publish.call_args.args[0]


def test_feedback_integrates_encoder_positions_and_source_time_not_commands():
    d, clock = feedback_driver()
    initial = measured(d, clock, 100., -50.)
    assert list(initial.data[:5]) == [0., 0., 0., 0., 0.]
    clock.now += .1
    # Installed forward cmd_vel uses positive right and negative left turns;
    # both odometry and wheel_travel are forward-positive.
    sample = measured(d, clock, 100.02, -50.02, .2, -.2)
    assert sample.data[0] == pytest.approx(.02)
    assert sample.data[3] == pytest.approx(.2)
    assert sample.data[-1] == clock.now
    travel = d.wheel_travel_pub.publish.call_args.args[0].data
    assert list(travel) == pytest.approx([.02, .02, clock.now, clock.now])
    assert d.target_linear_vel == .1  # Command deliberately disagrees with measurement.


def test_feedback_poll_blocks_writes_but_receives_stop_then_fault_stops():
    d = driver(True)
    d.feedback_poll = SimpleNamespace(poll=lambda: None, values=[None, None])
    d.control()
    d._write_track_velocities.assert_not_called()
    d.twist_callback(SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=0.)))
    assert d.target_linear_vel == 0.
    d.feedback_poll = SimpleNamespace(poll=Mock(side_effect=TimeoutError('late')))
    with pytest.raises(RuntimeError, match='stopped'):
        d.control()
    d._connection_lost.assert_called_once()
    d._write_track_velocities.assert_not_called()


def test_each_side_reports_its_own_valid_feedback_before_the_combined_odom():
    d, clock = feedback_driver()
    d.feedback_stamp = clock.now
    replies = [(1., 0.), None]
    d.feedback_poll = SimpleNamespace(poll=lambda: None, values=replies)
    assert not d._finish_feedback_poll()
    d.feedback_side_pubs[0].publish.assert_called_once()
    d.feedback_side_pubs[1].publish.assert_not_called()
    d.odom_pub.publish.assert_not_called()
    replies[1] = (2., 0.)
    d.feedback_poll.poll = lambda: tuple(replies)
    assert d._finish_feedback_poll()
    assert d.feedback_side_pubs[0].publish.call_count == 1
    d.feedback_side_pubs[1].publish.assert_called_once()
    d.odom_pub.publish.assert_called_once()


def test_nonzero_command_cannot_arm_before_wheel_feedback():
    d = driver(False)
    d.last_feedback_publish = None
    d.target_linear_vel = .1
    d.control()
    d._activate_odrives.assert_not_called()
    d._start_runtime_poll.assert_called_once()


def test_runtime_poll_serializes_voltage_and_feedback(monkeypatch):
    from masumasu_crawler.armadillo_driver import Abot04TwistDriver
    d, clock = feedback_driver()
    d._start_runtime_poll = lambda: Abot04TwistDriver._start_runtime_poll(d)
    d.wheel_feedback_period = .1
    d.voltage_query_timeout = .15
    d.odrv0 = d.odrv1 = Mock()
    d._start_voltage_poll = Mock()
    feedback = Mock()
    monkeypatch.setattr('masumasu_crawler.armadillo_driver.FeedbackPoll', feedback)
    monkeypatch.setattr('masumasu_crawler.armadillo_driver.time.monotonic', lambda: 10.)
    d._start_runtime_poll()
    feedback.assert_called_once()
    d._start_runtime_poll()
    assert feedback.call_count == 1
    d._start_voltage_poll.assert_not_called()
    d.feedback_poll = None
    d.last_feedback_publish = 9.99
    d._start_runtime_poll()
    d._start_voltage_poll.assert_called_once()
    d.voltage_poll = Mock()
    d.last_feedback_publish = 0.
    d._start_runtime_poll()
    assert feedback.call_count == 1
