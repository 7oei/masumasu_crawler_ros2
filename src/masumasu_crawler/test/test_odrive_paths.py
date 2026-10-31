"""Hardware-free coverage for alias preference and serial-based discovery."""
import io

import pytest
from masumasu_crawler import odrive_paths as paths


def test_existing_aliases_are_preferred_without_probing(monkeypatch):
    monkeypatch.setattr(paths.os.path, 'exists', lambda p: True)
    monkeypatch.setattr(paths.os.path, 'realpath', lambda p: p)
    assert paths.resolve_odrive_paths(0) == ('/dev/odrive-right', '/dev/odrive-left')


def test_duplicate_alias_is_rejected(monkeypatch):
    monkeypatch.setattr(paths.os.path, 'exists', lambda p: True)
    monkeypatch.setattr(paths.os.path, 'realpath', lambda p: '/dev/ttyACM0')
    with pytest.raises(RuntimeError, match='same device'):
        paths.resolve_odrive_paths(0)


def test_serial_discovery_does_not_assume_tty_order(monkeypatch):
    monkeypatch.setattr(paths.os.path, 'exists', lambda p: False)
    monkeypatch.setattr(paths.glob, 'glob', lambda p: ['/sys/class/tty/ttyACM0', '/sys/class/tty/ttyACM1'])
    monkeypatch.setattr(paths.os.path, 'realpath', lambda p:
        '/sys/devices/3-1/3-1:1.0' if 'ttyACM0' in p else '/sys/devices/3-4/3-4:1.0')
    def read_file(path):
        name = path.rsplit('/', 1)[-1]
        values = {'idVendor': '1209', 'idProduct': '0d32',
                  'serial': '365338553030' if '/3-1/' in path else '335831653536'}
        return io.StringIO(values[name])
    monkeypatch.setattr('builtins.open', read_file)
    assert paths.resolve_odrive_paths(0) == ('3-4', '3-1')


def test_missing_devices_fail_for_driver_retry(monkeypatch):
    monkeypatch.setattr(paths.os.path, 'exists', lambda p: False)
    monkeypatch.setattr(paths.glob, 'glob', lambda p: [])
    with pytest.raises(RuntimeError, match='Could not resolve'):
        paths.resolve_odrive_paths(0)
