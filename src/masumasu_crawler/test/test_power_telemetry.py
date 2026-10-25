import socket
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from masumasu_crawler.power_telemetry import VoltagePoll
from masumasu_crawler.armadillo_driver import Abot04TwistDriver


@pytest.fixture
def ports():
    pairs = [socket.socketpair() for _ in range(2)]
    devices = [SimpleNamespace(fd=host.fileno(), write=Mock()) for host, peer in pairs]
    yield devices, [peer for host, peer in pairs]
    for pair in pairs:
        for sock in pair:
            sock.close()


def test_partial_replies_require_both_fresh_voltages(ports):
    devices, peers = ports
    poll = VoltagePoll(devices, .15, now=0.)
    for device in devices:
        device.write.assert_called_once_with('r vbus_voltage')
    assert poll.poll(now=.01) is None
    peers[0].sendall(b'39.9\n')
    peers[1].sendall(b'38.')
    assert poll.poll(now=.02) is None
    peers[1].sendall(b'5\r\n')
    assert poll.poll(now=.03) == (39.9, 38.5)


@pytest.mark.parametrize('response', [b'nan\n', b'inf\n', b'0\n', b'-1\n',
                                      b'junk\n', b'39\n40\n', b'1' * 129])
def test_invalid_reply_is_not_published(ports, response):
    devices, peers = ports
    poll = VoltagePoll(devices, .15, now=0.)
    peers[0].sendall(response)
    with pytest.raises(ValueError):
        poll.poll(now=.01)


def test_missing_side_times_out_without_retry(ports):
    devices, peers = ports
    poll = VoltagePoll(devices, .15, now=0.)
    peers[0].sendall(b'39\n')
    assert poll.poll(now=.1) is None
    with pytest.raises(TimeoutError):
        poll.poll(now=.15)
    for device in devices:
        assert device.write.call_count == 1


def driver(armed):
    d = Abot04TwistDriver.__new__(Abot04TwistDriver)
    d.last_cmd_vel_time = time.monotonic()
    d.cmd_vel_timeout = .5
    d.reconnecting = False
    d.gui_estop_enabled = False
    d.armed = armed
    d.target_linear_vel = .1 if armed else 0.
    d.target_angular_vel = 0.
    d._calculate_track_velocities = lambda linear, angular: (linear, linear)
    d._activate_odrives = Mock()
    d._write_track_velocities = Mock()
    d._start_runtime_poll = Mock()
    d.battery_pub = Mock()
    d.voltage_poll = None
    d.feedback_poll = None
    d.feedback_side_pubs = [Mock(), Mock()]
    d.feedback_side_reported = [False, False]
    d.last_feedback_publish = time.monotonic()
    d.voltage_min = 30.
    d.voltage0 = d.voltage1 = 39.
    d.current0 = d.current1 = 0.
    d.current_max = 100.
    d.bypass_emergency_limits = False
    d.watchdog_test_active = True
    d.runtime_ascii_queries_enabled = False
    d.emergency = False
    d.reduction = d.crawler_teeth = d.drive_wheel_teeth = d.crawler_perimeter = 1.
    d._connection_lost = Mock(side_effect=RuntimeError('stopped'))
    return d


def test_idle_polls_without_arming_or_motor_commands():
    d = driver(False)
    d.control()
    d._start_runtime_poll.assert_called_once()
    d._activate_odrives.assert_not_called()
    d._write_track_velocities.assert_not_called()
    d.voltage_poll = SimpleNamespace(poll=lambda: (39.9, 38.5))
    d.control()
    assert list(d.battery_pub.publish.call_args.args[0].data) == [38.5, 39.9]
    assert not d.armed
    d._write_track_velocities.assert_not_called()


def test_gui_estop_starts_engaged_and_reengages_when_heartbeat_expires():
    d = driver(True)
    d.gui_estop_enabled = True
    d.gui_estop_active = True
    d.gui_estop_last_rx = None
    d.gui_estop_timeout_sec = .5
    d.control()
    d._write_track_velocities.assert_called_once_with(0., 0.)
    d.on_gui_estop(SimpleNamespace(data=False))
    d.twist_callback(SimpleNamespace(linear=SimpleNamespace(x=.1), angular=SimpleNamespace(z=0.)))
    assert d.target_linear_vel == .1
    d.gui_estop_last_rx -= 1.
    d.control()
    assert d.target_linear_vel == 0.
    assert d.target_angular_vel == 0.


def test_active_query_excludes_writes_and_uses_latest_stop_command():
    d = driver(True)
    d.control()
    d._write_track_velocities.assert_called_once_with(-.1, .1)
    d._start_runtime_poll.assert_called_once()
    d._write_track_velocities.reset_mock()
    d.voltage_poll = SimpleNamespace(poll=lambda: None)
    d.control()
    d._write_track_velocities.assert_not_called()
    d.twist_callback(SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=0.)))
    d.voltage_poll = SimpleNamespace(poll=lambda: (39.9, 38.5))
    d.control()
    d._write_track_velocities.assert_called_once_with(0., 0.)
    assert list(d.battery_pub.publish.call_args.args[0].data) == [38.5, 39.9]


def test_active_query_failure_stops_and_does_not_republish_cache():
    d = driver(True)
    d.voltage_poll = SimpleNamespace(poll=Mock(side_effect=TimeoutError('late')))
    with pytest.raises(RuntimeError, match='stopped'):
        d.control()
    d._connection_lost.assert_called_once()
    d.battery_pub.publish.assert_not_called()
    d._write_track_velocities.assert_not_called()


def test_low_voltage_stops_active_setpoints():
    d = driver(True)
    d.voltage_poll = SimpleNamespace(poll=lambda: (29., 39.))
    d.control()
    d._write_track_velocities.assert_called_once_with(0., 0.)
