from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import masumasu_crawler.armadillo_driver as module


def test_only_failed_left_device_is_reset_after_repeated_timeouts(monkeypatch):
    devices = []
    opens = [0]
    def connect(selector, *args):
        device = Mock(path=selector)
        if selector == '/dev/left':
            opens[0] += 1
            if opens[0] <= 5:
                device.query.side_effect = TimeoutError('wedged endpoint')
        device.read_float.return_value = 40.
        devices.append(device)
        return device
    reset = Mock()
    # A callable factory with the same static reset operation.
    connect.reset_usb = reset
    monkeypatch.setattr(module, 'ODriveAscii', connect)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.time, 'sleep', lambda seconds: None)
    d = SimpleNamespace(odrv0_selector='/dev/right', odrv1_selector='/dev/left',
                        ascii_timeout=.1, ascii_query_retries=0,
                        ascii_invalid_response_retries=0, ascii_resync_timeout=.6,
                        ascii_resync_enabled=True, usb_settle_delay=0.,
                        connect_retry_delay=0., usb_reset_after_failures=5,
                        get_logger=lambda: Mock(), _report_ascii_frame=Mock())
    result = module.Abot04TwistDriver._open_two_odrives(d)
    reset.assert_called_once_with('/dev/left')
    assert result == tuple(devices[-2:])
    for device in devices[:-2]:
        device.close.assert_called_once()
    for device in result:
        device.close.assert_not_called()


@pytest.mark.parametrize('vendor,allowed', [('1209', True), ('2e8a', False)])
def test_usb_reset_checks_device_identity_and_closes_fd(tmp_path, monkeypatch, vendor, allowed):
    device = tmp_path / 'sys/class/tty/ttyACM99/device'
    device.mkdir(parents=True)
    for name, value in [('idVendor', vendor), ('idProduct', '0d32'),
                        ('busnum', '1'), ('devnum', '42')]:
        (device / name).write_text(value)
    monkeypatch.setattr(module, 'Path', lambda path: Path(tmp_path / path.lstrip('/'))
                        if path.startswith('/sys/') else Path(path))
    monkeypatch.setattr(module.ODriveAscii, '_resolve_port', lambda selector: '/dev/ttyACM99')
    opened, closed, ioctl = Mock(return_value=123), Mock(), Mock()
    monkeypatch.setattr(module.os, 'open', opened)
    monkeypatch.setattr(module.os, 'close', closed)
    monkeypatch.setattr(module.fcntl, 'ioctl', ioctl)
    if allowed:
        module.ODriveAscii.reset_usb('/dev/left')
        opened.assert_called_once_with('/dev/bus/usb/001/042', module.os.O_WRONLY)
        ioctl.assert_called_once_with(123, 0x5514, 0)
        closed.assert_called_once_with(123)
    else:
        with pytest.raises(RuntimeError, match='non-ODrive'):
            module.ODriveAscii.reset_usb('/dev/left')
        opened.assert_not_called()
        ioctl.assert_not_called()


def test_termios_failure_does_not_leak_serial_descriptor(monkeypatch):
    d = object.__new__(module.ODriveAscii)
    d.path = '/dev/fake'
    monkeypatch.setattr(module.os, 'open', Mock(return_value=123))
    monkeypatch.setattr(module.termios, 'tcgetattr', Mock(side_effect=OSError('unplug')))
    close = Mock()
    monkeypatch.setattr(module.os, 'close', close)
    with pytest.raises(OSError):
        d._open()
    close.assert_called_once_with(123)
    assert d.fd is None
