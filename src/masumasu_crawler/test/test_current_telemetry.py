from unittest.mock import Mock

import pytest

from masumasu_crawler.armadillo_driver import Abot04TwistDriver
from masumasu_crawler.power_telemetry import CurrentPoll
from test_power_telemetry import driver, ports


@pytest.mark.parametrize('right,left', [(101., 0.), (0., 101.)])
def test_async_overcurrent_stops_both_tracks(right, left):
    d = driver(True)
    d.battery_current_pub = Mock()
    poll = CurrentPoll.__new__(CurrentPoll)
    poll.poll = Mock(return_value=(right, left))
    d.voltage_poll = poll
    d.control()
    assert d.emergency
    d._write_track_velocities.assert_called_once_with(0., 0.)


def test_regenerative_current_does_not_trigger_positive_current_limit():
    d = driver(True)
    d.current0 = -2.25
    d.control()
    assert not d.emergency
    d._write_track_velocities.assert_called_once_with(-.1, .1)


def test_signed_bus_currents_wait_for_both_sides(ports):
    devices, peers = ports
    poll = CurrentPoll(devices, .15, now=0.)
    for device in devices:
        device.write.assert_called_once_with('r ibus')
    peers[0].sendall(b'12.5\n')
    assert poll.poll(now=.01) is None
    peers[1].sendall(b'-2.25\n')
    assert poll.poll(now=.02) == (12.5, -2.25)


@pytest.mark.parametrize('response', ['nan', 'inf', '-inf', '1 2', 'invalid'])
def test_current_parser_rejects_bad_values(response):
    with pytest.raises(ValueError):
        CurrentPoll.parse(response)


def test_current_query_excludes_motor_writes_and_publishes_left_right():
    d = driver(True)
    d.battery_current_pub = Mock()
    poll = CurrentPoll.__new__(CurrentPoll)
    poll.poll = Mock(return_value=None)
    d.voltage_poll = poll
    d.control()
    d._write_track_velocities.assert_not_called()
    d.battery_current_pub.publish.assert_not_called()
    poll.poll.return_value = (12.5, -2.25)  # right, left from hardware
    d.control()
    assert list(d.battery_current_pub.publish.call_args.args[0].data) == [-2.25, 12.5]
    assert (d.current0, d.current1) == (12.5, -2.25)
    d.battery_pub.publish.assert_not_called()
    d._write_track_velocities.assert_called_once()


def test_current_timeout_stops_without_publishing_old_values():
    d = driver(True)
    d.battery_current_pub = Mock()
    poll = CurrentPoll.__new__(CurrentPoll)
    poll.poll = Mock(side_effect=TimeoutError('current read timeout'))
    d.voltage_poll = poll
    with pytest.raises(RuntimeError, match='stopped'):
        d.control()
    d._write_track_velocities.assert_not_called()
    d.battery_current_pub.publish.assert_not_called()


def test_current_query_uses_gap_after_voltage_and_before_next_feedback(monkeypatch):
    d = driver(True)
    d.last_voltage_publish = 10.
    d.last_current_publish = None
    d.power_telemetry_period = .5
    d.last_feedback_publish = 10.
    d.wheel_feedback_period = .1
    d.voltage_query_timeout = .15
    d.odrv0 = d.odrv1 = Mock()
    monkeypatch.setattr('masumasu_crawler.armadillo_driver.time.monotonic', lambda: 10.01)
    factory = Mock()
    monkeypatch.setattr('masumasu_crawler.armadillo_driver.CurrentPoll', factory)
    Abot04TwistDriver._start_runtime_poll(d)
    factory.assert_called_once_with((d.odrv0, d.odrv1), .15, 10.01)
    Abot04TwistDriver._start_runtime_poll(d)
    assert factory.call_count == 1  # no second query while the first is pending
