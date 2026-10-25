import socket
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from masumasu_crawler.framed_ascii import FramedQuery
from masumasu_crawler.power_telemetry import VoltagePoll


@pytest.fixture
def port():
    host, peer = socket.socketpair()
    d = SimpleNamespace(fd=host.fileno(), _rx_buffer=bytearray(),
                        write=lambda s: host.sendall((s+'\n').encode()),
                        frame_report_callback=Mock(), resync_enabled=True)
    d.begin_framed_query = lambda cmd, timeout: FramedQuery(d, cmd, timeout)
    yield d, peer
    host.close()
    peer.close()


def test_stale_numeric_replies_and_partial_markers_are_discarded(port):
    d, peer = port
    frame = FramedQuery(d, 'r vbus_voltage', 1.)
    peer.recv(4096)
    peer.sendall(f'0\n542806415\n{frame.start[:-2]}'.encode())
    assert frame.poll() is None
    peer.sendall(f'{frame.start[-2:]}\n39.3\n{frame.end}\n{frame.end}\n'.encode())
    assert frame.poll() == '39.3'
    assert frame.discarded == 2
    # The trailing guard belongs to the next frame's discard phase.
    assert d._rx_buffer == (frame.end+'\n').encode()


def test_delayed_output_is_pushed_by_marker_only(port):
    d, peer = port
    frame = FramedQuery(d, 'f 0', 1.)
    initial = peer.recv(4096)
    assert initial.count(b'f 0\n') == 1
    peer.sendall((frame.start+'\n').encode())
    frame.next_pump = 0.
    assert frame.poll() is None
    assert peer.recv(4096) == b'r test_property\n'
    peer.sendall(f'12.5 0.0\n{frame.end}\n'.encode())
    assert frame.poll() == '12.5 0.0'
    assert frame.pumps == 1


@pytest.mark.parametrize('middle', ['', '39\n40\n'])
def test_missing_or_extra_payload_is_never_accepted(port, middle):
    d, peer = port
    frame = FramedQuery(d, 'r vbus_voltage', 1.)
    peer.recv(4096)
    peer.sendall(f'{frame.start}\n{middle}{frame.end}\n'.encode())
    with pytest.raises(ValueError):
        frame.poll()


def test_payload_without_end_marker_is_not_accepted(port):
    d, peer = port
    frame = FramedQuery(d, 'r vbus_voltage', 1.)
    peer.recv(4096)
    peer.sendall(f'{frame.start}\n39\n'.encode())
    assert frame.poll() is None
    frame.deadline = time.monotonic()-1
    with pytest.raises(TimeoutError):
        frame.poll()


def test_telemetry_rejects_plausible_stale_value_and_accepts_framed_value(port):
    d, peer = port
    poll = VoltagePoll([d], 1.)
    frame = poll.frames[0]
    peer.recv(4096)
    peer.sendall(f'38.2\n{frame.start}\n39.3\n{frame.end}\n'.encode())
    assert poll.poll() == (39.3,)


@pytest.mark.parametrize('value', ['542806415', 'nan', 'inf', '0', '-1'])
def test_voltage_range_rejects_corrupted_numeric_values(value):
    with pytest.raises(ValueError):
        VoltagePoll.parse(value)
