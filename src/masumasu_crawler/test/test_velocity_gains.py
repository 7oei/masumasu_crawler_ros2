"""Verify RAM gain setup without connecting hardware or commanding motion."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from masumasu_crawler.armadillo_driver import Abot04TwistDriver


class Controller:
    def __init__(self):
        self.values = {f'axis{a}.controller.config.{name}': value
                       for a in (0, 1) for name, value in
                       [('vel_gain', .0144725), ('vel_integrator_gain', .23156)]}
        self.writes = []
        self.state = 1
        self.ignore_writes = False

    def read_float(self, prop):
        return self.state if prop.endswith('.current_state') else self.values[prop]

    def write_property(self, prop, value):
        self.writes.append((prop, value))
        if not self.ignore_writes:
            self.values[prop] = float(value)


def driver():
    d = Abot04TwistDriver.__new__(Abot04TwistDriver)
    d.odrv0, d.odrv1 = Controller(), Controller()
    d.odrive_vel_gain, d.odrive_vel_integrator_gain = .01881425, .23156
    d.closed_loop_timeout = 0.
    d.closed_loop_poll_period = .001
    d._ascii_operation = lambda label, operation: operation()
    d.get_logger = Mock(return_value=Mock())
    return d


def test_all_four_axes_receive_absolute_gains_without_accumulating():
    d = driver()
    for _ in range(2):
        d._configure_velocity_gains()
    for device in (d.odrv0, d.odrv1):
        assert len(device.writes) == 8
        for axis in (0, 1):
            assert device.values[f'axis{axis}.controller.config.vel_gain'] == .01881425
            assert device.values[f'axis{axis}.controller.config.vel_integrator_gain'] == .23156
    # Controller exposes no save/reboot/motion API: any such call would fail.


def test_preserve_gains_leaves_device_untouched():
    d = driver()
    d.odrive_vel_gain = d.odrive_vel_integrator_gain = -1.
    d.odrv0 = d.odrv1 = None
    d._configure_velocity_gains()


def test_zero_integrator_is_supported_and_p_can_be_preserved():
    d = driver()
    d.odrive_vel_gain, d.odrive_vel_integrator_gain = -1., 0.
    d._configure_velocity_gains()
    for device in (d.odrv0, d.odrv1):
        assert len(device.writes) == 2
        assert all(name.endswith('vel_integrator_gain') and float(value) == 0
                   for name, value in device.writes)


def test_every_axis_must_be_idle_before_any_gain_write():
    d = driver()
    d.odrv1.state = 8
    with pytest.raises(RuntimeError, match='not idle'):
        d._configure_velocity_gains()
    assert not d.odrv0.writes and not d.odrv1.writes


def test_readback_mismatch_aborts_setup():
    d = driver()
    d.odrv0.ignore_writes = True
    with pytest.raises(RuntimeError, match='mismatch'):
        d._configure_velocity_gains()
    assert not d.odrv1.writes


@pytest.mark.parametrize('gain', [.01881425, .024458525, .122292625, .1834389375, .24458525, .4891705])
def test_ascii_six_decimal_readback_accepts_rounding_on_all_axes(gain):
    d = driver()
    d.odrive_vel_gain = gain
    for device in (d.odrv0, d.odrv1):
        read = device.read_float
        device.read_float = lambda prop, read=read: float(f'{read(prop):.6f}')
    d._configure_velocity_gains()
    assert len(d.odrv0.writes) == len(d.odrv1.writes) == 4


def test_readback_beyond_ascii_rounding_is_rejected():
    d = driver()
    read = d.odrv0.read_float
    d.odrv0.read_float = lambda prop: (
        read(prop) + .000002 if prop.endswith('.vel_gain') else read(prop))
    with pytest.raises(RuntimeError, match='mismatch'):
        d._configure_velocity_gains()


def test_setup_failure_closes_devices_and_remains_disarmed():
    d = driver()
    d.armed = False
    d._open_two_odrives = Mock(return_value=(d.odrv0, d.odrv1))
    d.watchdog_test_enabled = False
    d.odrv0.clear_errors = d.odrv1.clear_errors = Mock()
    d._safe_zero_velocity = Mock()
    d._safe_set_idle = Mock()
    d._close_odrives = Mock()
    d.odrv0.ignore_writes = True
    with pytest.raises(RuntimeError, match='mismatch'):
        d._connect_odrives()
    assert d.reconnecting and d.emergency and not d.armed
    d._close_odrives.assert_called_once()
    assert d._safe_zero_velocity.call_count == 2
    assert d._safe_set_idle.call_count == 2


@pytest.mark.parametrize('name,value', [
    ('odrive_vel_gain', 0.), ('odrive_vel_gain', -2.),
    ('odrive_vel_gain', float('nan')),
    ('odrive_vel_integrator_gain', -2.),
    ('odrive_vel_integrator_gain', float('inf'))])
def test_invalid_gain_rejected_before_hardware_access(name, value):
    d = driver()
    parameters = {}
    d.declare_parameter = lambda name, value: parameters.setdefault(name, value)
    d._declare_parameters()
    parameters[name] = value
    d.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    with pytest.raises(ValueError, match=name):
        d._read_parameters()
