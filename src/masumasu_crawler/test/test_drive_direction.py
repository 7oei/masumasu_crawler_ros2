from unittest.mock import Mock, call

import pytest

from masumasu_crawler.armadillo_driver import Abot04TwistDriver


@pytest.mark.parametrize('linear,angular,odrv0,odrv1', [
    (0.2, 0.0, 0.2, -0.2),
    (-0.2, 0.0, -0.2, 0.2),
    (0.0, 0.5, -0.2, -0.2),
    (0.0, -0.5, 0.2, 0.2),
    (0.3, 0.5, 0.1, -0.5),
    (0.0, 0.0, 0.0, 0.0),
    (2.0, 0.0, 1.0, -1.0),
])
def test_forward_reversed_and_yaw_preserved_on_all_axes(
        linear, angular, odrv0, odrv1):
    driver = Abot04TwistDriver.__new__(Abot04TwistDriver)
    driver.linear_vel_limit = 1.0
    driver.angular_vel_limit = 1.0
    driver.tread = 0.8
    driver.odrv0 = Mock()
    driver.odrv1 = Mock()
    driver._ascii_operation = lambda label, operation: operation()
    right, left = driver._calculate_track_velocities(linear, angular)
    # Apply the control loop's signs with one motor turn per metre.
    driver._write_track_velocities(-left, right)
    for device, expected in ((driver.odrv0, odrv0), (driver.odrv1, odrv1)):
        assert device.set_velocity.call_args_list == [
            call(0, pytest.approx(expected)), call(1, pytest.approx(expected))]
