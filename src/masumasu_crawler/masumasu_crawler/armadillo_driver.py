#!/usr/bin/env python3
from .odrive_paths import resolve_odrive_paths
import fcntl
from pathlib import Path
import glob
import math
import os
import select
import termios
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64MultiArray
from .power_telemetry import CurrentPoll, FeedbackPoll, VoltagePoll
from .framed_ascii import FramedQuery


AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 1 << 11


class ODriveAscii:
    """Small line-oriented transport for the ODrive USB CDC interface."""

    def __init__(
            self, selector, timeout, query_retries,
            invalid_response_retries, resync_timeout=0.6,
            resync_enabled=True):
        self.selector = selector
        self.timeout = timeout
        self.query_retries = query_retries
        self.invalid_response_retries = invalid_response_retries
        self.resync_timeout = resync_timeout
        self.resync_enabled = resync_enabled
        self.path = self._resolve_port(selector)
        self.fd = None
        self.query_wait_callback = None
        self.query_wait_poll_interval = None
        self.query_recovery_callback = None
        self._rx_buffer = bytearray()
        self._open()

    @staticmethod
    def _resolve_port(selector):
        if selector.startswith('/dev/'):
            if not os.path.exists(selector):
                raise FileNotFoundError(selector)
            return selector

        matches = []
        for sys_tty in glob.glob('/sys/class/tty/ttyACM*'):
            device_path = os.path.realpath(os.path.join(sys_tty, 'device'))
            usb_interface = f'/{selector}/{selector}:'
            if usb_interface in device_path:
                matches.append(f'/dev/{os.path.basename(sys_tty)}')
        if len(matches) != 1:
            raise RuntimeError(
                f'USB path {selector!r} resolved to {matches}, expected one port')
        return matches[0]

    @staticmethod
    def reset_usb(selector):
        """Reset only the ODrive USB device backing this resolved tty (Linux)."""
        tty = Path(os.path.realpath(ODriveAscii._resolve_port(selector))).name
        device = (Path('/sys/class/tty') / tty / 'device').resolve()
        for parent in (device, *device.parents):
            if not (parent / 'idVendor').exists():
                continue
            if ((parent / 'idVendor').read_text().strip().lower() != '1209' or
                    (parent / 'idProduct').read_text().strip().lower() != '0d32'):
                raise RuntimeError('Refusing USB reset of a non-ODrive device')
            bus = int((parent / 'busnum').read_text())
            number = int((parent / 'devnum').read_text())
            fd = os.open(f'/dev/bus/usb/{bus:03d}/{number:03d}', os.O_WRONLY)
            try:
                fcntl.ioctl(fd, 0x5514, 0)  # Linux USBDEVFS_RESET
            finally:
                os.close(fd)
            return
        raise RuntimeError('No USB device found for ' + selector)

    def _open(self):
        self.fd = os.open(
            self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            attrs[4] = termios.B115200
            attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def write(self, command):
        if self.fd is None:
            raise RuntimeError(f'{self.path} is closed')
        payload = (command + '\n').encode('ascii')
        written = os.write(self.fd, payload)
        if written != len(payload):
            raise OSError(
                f'short write to {self.path}: {written}/{len(payload)} bytes')

    def set_query_wait_callback(
            self, callback, poll_interval, recovery_callback=None):
        self.query_wait_callback = callback
        self.query_wait_poll_interval = poll_interval
        self.query_recovery_callback = recovery_callback

    def _wait_callback(self):
        if self.query_wait_callback is not None:
            self.query_wait_callback()

    def _read_line(self, deadline):
        while time.monotonic() < deadline:
            if b'\n' in self._rx_buffer:
                line, _, remainder = self._rx_buffer.partition(b'\n')
                self._rx_buffer = bytearray(remainder)
                return line.rstrip(b'\r').decode(
                    'ascii', errors='strict').strip()

            remaining = max(0.0, deadline - time.monotonic())
            wait_timeout = remaining
            if self.query_wait_callback is not None:
                wait_timeout = min(
                    wait_timeout, self.query_wait_poll_interval)
            readable, _, _ = select.select(
                [self.fd], [], [], wait_timeout)
            if not readable:
                self._wait_callback()
                continue
            chunk = os.read(self.fd, 4096)
            if not chunk:
                raise OSError(f'{self.path} returned EOF')
            self._rx_buffer.extend(chunk)
        return None

    def _query_once(self, command):
        if self.fd is None:
            raise RuntimeError(f'{self.path} is closed')
        if self.resync_enabled:
            frame = self.begin_framed_query(command, self.resync_timeout)
            while True:
                response = frame.poll()
                if response is not None:
                    return response
                # Bound each transaction; do not mix watchdog writes with it.
                time.sleep(.001)
        self.write(command)
        response = self._read_line(time.monotonic() + self.timeout)
        if response is not None:
            return response
        raise TimeoutError(
            f'{self.path}: no response to {command!r} within '
            f'{self.timeout:.3g}s')

    def begin_framed_query(self, command, timeout):
        return FramedQuery(self, command, timeout)

    def _resynchronize(self):
        """Require a complete fresh frame before retrying a failed query."""
        if self.query_recovery_callback is not None:
            self.query_recovery_callback()
        value = float(self._query_once('r vbus_voltage'))
        if not math.isfinite(value) or not 0 < value <= 60:
            raise ValueError(f'Invalid synchronized voltage: {value}')

    def query(self, command):
        if not getattr(self, 'resync_enabled', True):
            # A timeout makes a delayed reply indistinguishable from the
            # reply to a retry. Never resend on this connection. Let the
            # driver close and reopen the port. Without framing, a host flush
            # cannot guarantee that device-side delayed responses are gone.
            self._rx_buffer.clear()
            termios.tcflush(self.fd, termios.TCIFLUSH)
            try:
                return self._query_once(command)
            except TimeoutError as error:
                raise TimeoutError(
                    f'{self.path}: no response to {command!r} within '
                    f'{self.timeout:.3g}s; connection must be reopened') \
                    from error

        try:
            return self._query_once(command)
        except TimeoutError as first_error:
            last_error = first_error

        # A timeout poisons response ordering. Never resend the query on the
        # live stream: synchronize with a unique nonce first, then retry it.
        for _ in range(self.query_retries):
            try:
                self._resynchronize()
                return self._query_once(command)
            except TimeoutError as error:
                last_error = error
        raise TimeoutError(
            f'{self.path}: query {command!r} failed after '
            f'{self.query_retries} resynchronization attempts: '
            f'{last_error}') from last_error

    def _validated_query(self, command, parser):
        last_error = None
        for _ in range(self.invalid_response_retries + 1):
            response = self.query(command)
            try:
                return parser(response)
            except (TypeError, ValueError) as error:
                last_error = error
                if getattr(self, 'resync_enabled', True):
                    self._resynchronize()
                else:
                    raise ValueError(
                        f'{self.path}: invalid response to {command!r}: '
                        f'{response!r}; connection must be reopened') \
                        from error
        raise ValueError(
            f'{self.path}: invalid response to {command!r} after '
            f'{self.invalid_response_retries + 1} attempts: '
            f'{last_error}') from last_error

    def read_float(self, property_name):
        def parse(response):
            value = float(response)
            if not math.isfinite(value):
                raise ValueError(f'Non-finite {property_name}: {response}')
            if property_name == 'vbus_voltage' and not 0 < value <= 60:
                raise ValueError(f'Invalid bus voltage: {response}')
            return value
        return self._validated_query(f'r {property_name}', parse)

    def write_property(self, property_name, value):
        self.write(f'w {property_name} {value}')

    def feedback(self, axis):
        def parse_feedback(response):
            fields = response.split()
            if len(fields) != 2:
                raise ValueError(
                    f'invalid feedback for axis{axis}: {fields!r}')
            return float(fields[0]), float(fields[1])

        return self._validated_query(f'f {axis}', parse_feedback)

    def set_velocity(self, axis, velocity):
        self.write(f'v {axis} {velocity:.9g} 0')

    def feed_watchdog(self, axis):
        self.write(f'u {axis}')

    def set_state(self, axis, state):
        self.write(f'w axis{axis}.requested_state {state}')

    def clear_errors(self):
        self.write('sc')


class Abot04TwistDriver(Node):
    def __init__(self):
        super().__init__('crawler_driver')
        self._declare_parameters()
        self._read_parameters()
        self.gui_estop_active = True
        self.gui_estop_last_rx = None

        self.emergency = True
        self.reconnecting = True
        self.armed = False
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.right_pos = 0.0
        self.left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_left_pos = 0.0
        self.current0 = 0.0
        self.voltage0 = 0.0
        self.current1 = 0.0
        self.voltage1 = 0.0
        self.last_telemetry_read = None
        self.voltage_poll = None
        self.feedback_poll = None
        self.feedback_side_reported = [False, False]
        self.feedback_stamp = None
        self.last_feedback_publish = None
        self.last_voltage_publish = None
        self.last_current_publish = None
        self.last_cmd_vel_time = None
        self.last_command_left_rps = 0.0
        self.last_command_right_rps = 0.0
        self.last_successful_ascii_operation = 'initialization'
        self.watchdog_test_active = False
        self.last_watchdog_test_check = None
        self.watchdog_test_check_index = 0
        self.odrv0 = None
        self.odrv1 = None
        self.start_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Float64MultiArray, 'odom_buf', 1)
        self.wheel_travel_pub = self.create_publisher(
            Float64MultiArray, 'wheel_travel', 1)
        self.forward_travel = [0.0, 0.0]
        self.battery_pub = self.create_publisher(
            Float64MultiArray, 'battery/voltages', 1)
        self.battery_current_pub = self.create_publisher(
            Float64MultiArray, 'battery/currents', 1)
        self.feedback_side_pubs = [
            self.create_publisher(Empty, '/odrive/right/feedback', 1),
            self.create_publisher(Empty, '/odrive/left/feedback', 1),
        ]
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)
        if self.gui_estop_enabled:
            estop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.gui_estop_sub = self.create_subscription(
                Bool, self.gui_estop_topic, self.on_gui_estop, estop_qos)

        self._connect_odrives()
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control)
        self.get_logger().info(
            'Masumasu crawler ODrive ASCII driver initialized '
            f'(ports={self.odrv0.path},{self.odrv1.path}, '
            f'control_rate={self.control_rate:.1f} Hz, '
            f'power_telemetry_rate={self.power_telemetry_rate:.1f} Hz); '
            'waiting for first non-zero drive command')

    def _declare_parameters(self):
        self.declare_parameter('odrive0_usb_path', 'auto')
        self.declare_parameter('odrive1_usb_path', 'auto')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('ascii_timeout', 5.0)
        self.declare_parameter('ascii_query_retries', 2)
        self.declare_parameter('ascii_invalid_response_retries', 2)
        self.declare_parameter('ascii_resync_enabled', True)
        self.declare_parameter('ascii_resync_timeout', 0.6)
        self.declare_parameter('connect_retry_delay', 0.5)
        self.declare_parameter('usb_reset_after_failures', 0)
        self.declare_parameter('usb_settle_delay', 1.5)
        self.declare_parameter('closed_loop_timeout', 2.0)
        self.declare_parameter('closed_loop_poll_period', 0.1)
        self.declare_parameter('watchdog_test_enabled', False)
        self.declare_parameter('watchdog_test_timeout', 10.0)
        self.declare_parameter('watchdog_test_check_period', 1.0)
        self.declare_parameter('watchdog_query_wait_feed_period', 0.1)
        self.declare_parameter('runtime_ascii_queries_enabled', False)
        self.declare_parameter('voltage_query_timeout', 0.15)
        self.declare_parameter('wheel_feedback_rate', 10.0)
        # Negative sentinel preserves the controller's existing RAM setting.
        self.declare_parameter('odrive_vel_gain', -1.0)
        self.declare_parameter('odrive_vel_integrator_gain', -1.0)
        self.declare_parameter('tread', 0.8)
        self.declare_parameter('reduction', 26.4)
        self.declare_parameter('drive_wheel_teeth', 8.0)
        self.declare_parameter('crawler_teeth', 46.0)
        self.declare_parameter('crawler_perimeter', 3.312)
        self.declare_parameter('linear_vel_limit', 1.67)
        self.declare_parameter('angular_vel_limit', 6.28)
        self.declare_parameter('yaw_calibration', 1.875)
        self.declare_parameter('voltage_min', 22.2)
        self.declare_parameter('current_max', 100.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('power_telemetry_rate', 5.0)
        self.declare_parameter('bypass_emergency_limits', False)
        self.declare_parameter('gui_estop_enabled', False)
        self.declare_parameter('gui_estop_topic', '/robot_1/gui/emergency_stop')
        self.declare_parameter('gui_estop_timeout_sec', 0.5)

    def _read_parameters(self):
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        if not math.isfinite(self.cmd_vel_timeout) or self.cmd_vel_timeout <= 0:
            raise ValueError("cmd_vel_timeout must be finite and positive")
        self.odrv0_selector = str(
            self.get_parameter('odrive0_usb_path').value)
        self.odrv1_selector = str(
            self.get_parameter('odrive1_usb_path').value)
        if self.odrv0_selector == self.odrv1_selector and self.odrv0_selector != 'auto':
            raise ValueError('ODrive USB paths must be different')
        self.ascii_timeout = float(
            self.get_parameter('ascii_timeout').value)
        self.ascii_query_retries = int(
            self.get_parameter('ascii_query_retries').value)
        self.ascii_invalid_response_retries = int(
            self.get_parameter('ascii_invalid_response_retries').value)
        self.ascii_resync_enabled = bool(
            self.get_parameter('ascii_resync_enabled').value)
        self.ascii_resync_timeout = float(
            self.get_parameter('ascii_resync_timeout').value)
        self.connect_retry_delay = float(
            self.get_parameter('connect_retry_delay').value)
        self.usb_reset_after_failures = int(self.get_parameter('usb_reset_after_failures').value)
        if self.usb_reset_after_failures < 0:
            raise ValueError('usb_reset_after_failures must not be negative')
        self.usb_settle_delay = float(
            self.get_parameter('usb_settle_delay').value)
        self.closed_loop_timeout = float(
            self.get_parameter('closed_loop_timeout').value)
        self.closed_loop_poll_period = float(
            self.get_parameter('closed_loop_poll_period').value)
        self.watchdog_test_enabled = bool(
            self.get_parameter('watchdog_test_enabled').value)
        self.watchdog_test_timeout = float(
            self.get_parameter('watchdog_test_timeout').value)
        self.watchdog_test_check_period = float(
            self.get_parameter('watchdog_test_check_period').value)
        self.watchdog_query_wait_feed_period = float(
            self.get_parameter('watchdog_query_wait_feed_period').value)
        self.runtime_ascii_queries_enabled = bool(
            self.get_parameter('runtime_ascii_queries_enabled').value)
        self.voltage_query_timeout = float(
            self.get_parameter('voltage_query_timeout').value)
        self.wheel_feedback_rate = float(self.get_parameter('wheel_feedback_rate').value)
        self.odrive_vel_gain = float(self.get_parameter('odrive_vel_gain').value)
        self.odrive_vel_integrator_gain = float(
            self.get_parameter('odrive_vel_integrator_gain').value)
        for name, value in (('odrive_vel_gain', self.odrive_vel_gain),
                            ('odrive_vel_integrator_gain', self.odrive_vel_integrator_gain)):
            if not math.isfinite(value) or (value < 0 and value != -1):
                raise ValueError(f'{name} must be finite and nonnegative, or -1 to preserve')
        if self.odrive_vel_gain == 0:
            raise ValueError('odrive_vel_gain must be positive, or -1 to preserve')
        self.tread = float(self.get_parameter('tread').value)
        self.reduction = float(self.get_parameter('reduction').value)
        self.drive_wheel_teeth = float(
            self.get_parameter('drive_wheel_teeth').value)
        self.crawler_teeth = float(
            self.get_parameter('crawler_teeth').value)
        self.crawler_perimeter = float(
            self.get_parameter('crawler_perimeter').value)
        self.linear_vel_limit = float(
            self.get_parameter('linear_vel_limit').value)
        self.angular_vel_limit = float(
            self.get_parameter('angular_vel_limit').value)
        self.yaw_calibration = float(
            self.get_parameter('yaw_calibration').value)
        self.voltage_min = float(
            self.get_parameter('voltage_min').value)
        self.current_max = float(
            self.get_parameter('current_max').value)
        self.control_rate = float(
            self.get_parameter('control_rate').value)
        self.power_telemetry_rate = float(
            self.get_parameter('power_telemetry_rate').value)
        self.bypass_emergency_limits = bool(
            self.get_parameter('bypass_emergency_limits').value)
        self.gui_estop_enabled = bool(self.get_parameter('gui_estop_enabled').value)
        self.gui_estop_topic = str(self.get_parameter('gui_estop_topic').value)
        self.gui_estop_timeout_sec = max(0., float(self.get_parameter('gui_estop_timeout_sec').value))

        if self.ascii_timeout <= 0.0:
            raise ValueError('ascii_timeout must be greater than zero')
        if self.ascii_query_retries < 0:
            raise ValueError('ascii_query_retries must not be negative')
        if self.ascii_invalid_response_retries < 0:
            raise ValueError(
                'ascii_invalid_response_retries must not be negative')
        if self.ascii_resync_timeout <= 0.0:
            raise ValueError('ascii_resync_timeout must be greater than zero')
        if self.connect_retry_delay < 0.0:
            raise ValueError('connect_retry_delay must not be negative')
        if self.usb_settle_delay < 0.0:
            raise ValueError('usb_settle_delay must not be negative')
        if self.closed_loop_timeout <= 0.0:
            raise ValueError('closed_loop_timeout must be greater than zero')
        if self.closed_loop_poll_period <= 0.0:
            raise ValueError(
                'closed_loop_poll_period must be greater than zero')
        if self.watchdog_test_timeout <= 0.0:
            raise ValueError('watchdog_test_timeout must be greater than zero')
        if self.watchdog_test_check_period <= 0.0:
            raise ValueError(
                'watchdog_test_check_period must be greater than zero')
        if self.watchdog_query_wait_feed_period <= 0.0:
            raise ValueError(
                'watchdog_query_wait_feed_period must be greater than zero')
        if (
                self.watchdog_query_wait_feed_period >=
                self.watchdog_test_timeout):
            raise ValueError(
                'watchdog_query_wait_feed_period must be shorter than the '
                'watchdog timeout')
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be greater than zero')
        if self.power_telemetry_rate <= 0.0:
            raise ValueError('power_telemetry_rate must be greater than zero')
        self.power_telemetry_period = 1.0 / self.power_telemetry_rate
        if not math.isfinite(self.wheel_feedback_rate) or not 5. <= self.wheel_feedback_rate <= self.control_rate:
            raise ValueError('wheel_feedback_rate must be between 5 Hz and control_rate')
        self.wheel_feedback_period = 1.0 / self.wheel_feedback_rate
        if (not math.isfinite(self.voltage_query_timeout) or
                not 1.0 / self.control_rate < self.voltage_query_timeout <= .15 or
                (self.watchdog_test_enabled and
                 self.voltage_query_timeout + 2.0 / self.control_rate >=
                 self.watchdog_test_timeout)):
            raise ValueError('voltage query deadline must fit within 0.15 s and watchdog budget')

    @property
    def distance_per_turn(self):
        ratio = self.reduction * (
            self.crawler_teeth / self.drive_wheel_teeth)
        return self.crawler_perimeter / ratio

    def _ascii_operation(self, label, operation):
        try:
            result = operation()
        except Exception as error:
            command_age = (
                None if self.last_cmd_vel_time is None
                else time.monotonic() - self.last_cmd_vel_time)
            self.get_logger().error(
                'ASCII diagnostic failure: '
                f'operation={label}, '
                f'last_success={self.last_successful_ascii_operation}, '
                f'error={error!r}, '
                f'target_linear={self.target_linear_vel:.9g}, '
                f'target_angular={self.target_angular_vel:.9g}, '
                f'command_left_rps={self.last_command_left_rps:.9g}, '
                f'command_right_rps={self.last_command_right_rps:.9g}, '
                f'command_age={command_age!r}, '
                f'emergency={self.emergency}, '
                f'cached_power=('
                f'{self.current0:.6g}, {self.voltage0:.6g}, '
                f'{self.current1:.6g}, {self.voltage1:.6g})')
            raise
        self.last_successful_ascii_operation = label
        return result

    def _report_ascii_frame(self, path, discarded, pumps):
        if not discarded and not pumps:
            return
        now = time.monotonic()
        reports = getattr(self, '_frame_reports', {})
        if now - reports.get(path, -float('inf')) >= 10:
            self.get_logger().warning(
                f'ASCII frame synchronized on {path}: '
                f'discarded_lines={discarded}, marker_pumps={pumps}; '
                'only the response between fresh markers was accepted '
                '(discard count includes old markers, not an exact query offset)')
            reports[path] = now
            self._frame_reports = reports

    def _open_two_odrives(self):
        attempt = 0
        failures = {}
        while rclpy.ok():
            attempt += 1
            odrv0 = None
            odrv1 = None
            self.get_logger().warning(
                f'Opening ODrive ASCII ports attempt={attempt}: '
                f'{self.odrv0_selector}, {self.odrv1_selector}')
            failed_selector = self.odrv0_selector
            try:
                if 'auto' in (self.odrv0_selector, self.odrv1_selector):
                    right, left = resolve_odrive_paths()
                    if self.odrv0_selector == 'auto':
                        self.odrv0_selector = right
                    if self.odrv1_selector == 'auto':
                        self.odrv1_selector = left
                    failed_selector = self.odrv0_selector
                    self.get_logger().info(
                        f'Resolved ODrives: right={self.odrv0_selector}, '
                        f'left={self.odrv1_selector}')
                if self.odrv0_selector == self.odrv1_selector:
                    raise ValueError('Resolved ODrive USB paths must be different')
                odrv0 = ODriveAscii(
                    self.odrv0_selector, self.ascii_timeout,
                    self.ascii_query_retries,
                    self.ascii_invalid_response_retries,
                    self.ascii_resync_timeout,
                    self.ascii_resync_enabled)
                failed_selector = self.odrv1_selector
                odrv1 = ODriveAscii(
                    self.odrv1_selector, self.ascii_timeout,
                    self.ascii_query_retries,
                    self.ascii_invalid_response_retries,
                    self.ascii_resync_timeout,
                    self.ascii_resync_enabled)
                for device in (odrv0, odrv1):
                    device.frame_report_callback = (
                        lambda discarded, pumps, path=device.path:
                        self._report_ascii_frame(path, discarded, pumps))
                # ttyACM nodes appear before an ODrive that has just
                # re-enumerated is necessarily ready for sustained traffic.
                time.sleep(self.usb_settle_delay)
                failed_selector = self.odrv0_selector
                serial0 = odrv0.query('r serial_number')
                voltage0 = odrv0.read_float('vbus_voltage')
                failed_selector = self.odrv1_selector
                serial1 = odrv1.query('r serial_number')
                voltage1 = odrv1.read_float('vbus_voltage')
                self.get_logger().info(
                    'ODrive ASCII ports opened: '
                    f'{odrv0.path} serial={serial0} voltage={voltage0:.3f} V, '
                    f'{odrv1.path} serial={serial1} voltage={voltage1:.3f} V')
                return odrv0, odrv1
            except Exception as error:
                self.get_logger().warning(
                    f'Opening ODrive ASCII ports failed: {error!r}')
                for device in (odrv0, odrv1):
                    if device is not None:
                        try:
                            device.close()
                        except OSError:
                            pass
                # Reopening a tty alone cannot recover a wedged USB endpoint.
                # Close both ports first; never reset another USB peripheral.
                failures[failed_selector] = failures.get(failed_selector, 0) + 1
                threshold = self.usb_reset_after_failures
                if threshold and failures[failed_selector] >= threshold:
                    failures[failed_selector] = 0
                    try:
                        ODriveAscii.reset_usb(failed_selector)
                        self.get_logger().warning('USB reset requested: ' + failed_selector)
                        time.sleep(max(1.0, self.usb_settle_delay))
                    except Exception as reset_error:
                        self.get_logger().warning(
                            f'USB reset unavailable for {failed_selector}: {reset_error!r}; '
                            'check USB device permissions (masumasu udev rules) and power/cable')
                time.sleep(self.connect_retry_delay)
        raise RuntimeError('ROS shutdown while waiting for ODrives')

    def _connect_odrives(self):
        self.reconnecting = True
        self.emergency = True
        try:
            self.odrv0, self.odrv1 = self._open_two_odrives()
            if self.watchdog_test_enabled:
                self._prepare_watchdogs()
            self._ascii_operation(
                'clear odrv0 errors', self.odrv0.clear_errors)
            self._ascii_operation(
                'clear odrv1 errors', self.odrv1.clear_errors)
            # Stay idle until the operator provides the first real command.
            # Voltage reads do not require closed-loop. Entering closed-loop
            # with no command present can leave the USB CDC endpoint wedged.
            self._safe_zero_velocity()
            self._safe_set_idle()
            self._configure_velocity_gains()
            self.armed = False
            self.watchdog_test_active = False
            self.last_watchdog_test_check = None
            self.watchdog_test_check_index = 0
            self.reconnecting = False
            self.emergency = True
        except Exception as error:
            self.get_logger().error(
                'ODrive ASCII initialization failed; '
                f'process restart required: {error!r}')
            self._safe_zero_velocity()
            self._safe_set_idle()
            self._close_odrives()
            raise

    def _configure_velocity_gains(self):
        """Apply absolute RAM-only gains on all axes before allowing motion."""
        settings = [(name, value) for name, value in (
            ('vel_gain', self.odrive_vel_gain),
            ('vel_integrator_gain', self.odrive_vel_integrator_gain)) if value != -1]
        if not settings:
            return
        axes = [(device, label, axis) for device, label in (
            (self.odrv0, 'right'), (self.odrv1, 'left')) for axis in (0, 1)]
        # The best-effort idle request above is not sufficient for tuning:
        # require every axis to report IDLE before changing any controller.
        for device, label, axis in axes:
            deadline = time.monotonic() + self.closed_loop_timeout
            while True:
                state = self._ascii_operation(
                    f'verify {label} axis{axis} idle before gain setup',
                    lambda: device.read_float(f'axis{axis}.current_state'))
                if state == AXIS_STATE_IDLE:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'{label} axis{axis} not idle; refusing gain setup')
                time.sleep(self.closed_loop_poll_period)
        for device, label, axis in axes:
            for name, value in settings:
                prop = f'axis{axis}.controller.config.{name}'
                previous = self._ascii_operation(
                    f'read {label} {prop}', lambda: device.read_float(prop))
                self._ascii_operation(
                    f'set {label} {prop}',
                    lambda: device.write_property(prop, f'{value:.9g}'))
                actual = self._ascii_operation(
                    f'verify {label} {prop}', lambda: device.read_float(prop))
                # Legacy ASCII replies round floats to six decimal places.
                # Allow that quantization plus float32 representation error.
                if not math.isclose(actual, value, rel_tol=1e-6, abs_tol=5.1e-7):
                    raise RuntimeError(
                        f'{label} {prop} mismatch: wrote {value:.9g}, read {actual:.9g}')
                self.get_logger().info(
                    f'ODrive RAM-only gain {label} axis{axis} {name}: '
                    f'{previous:.9g} -> {actual:.9g} (not saved to flash)')

    def _prepare_watchdogs(self):
        """Configure watchdogs while disarmed, before the first drive command."""
        for device, label in (
                (self.odrv0, 'odrv0'), (self.odrv1, 'odrv1')):
            for axis in (0, 1):
                timeout_property = f'axis{axis}.config.watchdog_timeout'
                enable_property = f'axis{axis}.config.enable_watchdog'
                self._ascii_operation(
                    f'disable {label} axis{axis} watchdog',
                    lambda device=device, enable_property=enable_property:
                    device.write_property(enable_property, '0'))
                enabled_readback = self._ascii_operation(
                    f'verify {label} axis{axis} watchdog disabled',
                    lambda device=device, enable_property=enable_property:
                    device.read_float(enable_property))
                if int(enabled_readback) != 0:
                    raise RuntimeError(
                        f'{label} axis{axis} watchdog disable readback '
                        f'mismatch: {enabled_readback:.9g}')
                self._ascii_operation(
                    f'set {label} axis{axis} watchdog timeout',
                    lambda device=device, timeout_property=timeout_property:
                    device.write_property(
                        timeout_property,
                        f'{self.watchdog_test_timeout:.9g}'))
                timeout_readback = self._ascii_operation(
                    f'verify {label} axis{axis} watchdog timeout',
                    lambda device=device, timeout_property=timeout_property:
                    device.read_float(timeout_property))
                if not math.isclose(
                        timeout_readback, self.watchdog_test_timeout,
                        rel_tol=1e-6, abs_tol=1e-6):
                    raise RuntimeError(
                        f'{label} axis{axis} watchdog timeout readback '
                        f'mismatch: wrote {self.watchdog_test_timeout:.9g}, '
                        f'read {timeout_readback:.9g}')

    def _activate_odrives(self, command_left_rps, command_right_rps):
        # Match the working robot: prime the command before closed-loop,
        # then repeat it immediately after requesting the state transition.
        self._write_track_velocities(command_left_rps, command_right_rps)
        for device, label in (
                (self.odrv0, 'odrv0'), (self.odrv1, 'odrv1')):
            for axis in (0, 1):
                self._ascii_operation(
                    f'{label} set axis{axis} closed-loop',
                    lambda device=device, axis=axis:
                    device.set_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL))
        self._write_track_velocities(command_left_rps, command_right_rps)

        for device, label in (
                (self.odrv0, 'odrv0'), (self.odrv1, 'odrv1')):
            for axis in (0, 1):
                deadline = time.monotonic() + self.closed_loop_timeout
                state = None
                while time.monotonic() < deadline:
                    state = int(self._ascii_operation(
                        f'{label} read axis{axis} current_state',
                        lambda device=device, axis=axis:
                        device.read_float(f'axis{axis}.current_state')))
                    if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                        break
                    time.sleep(self.closed_loop_poll_period)
                if state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    raise RuntimeError(
                        f'{label} axis{axis} did not enter closed-loop within '
                        f'{self.closed_loop_timeout:.3g}s: {state}')

        self._reset_encoder_reference()
        self._update_power_telemetry(force=True)
        self.armed = True
        self.emergency = False
        self.get_logger().info(
            'ODrive closed-loop activated by first non-zero drive command')
        if self.watchdog_test_enabled:
            self._enable_watchdog_test()

    def _enable_watchdog_test(self):
        """Arm the preconfigured watchdog on every driven axis."""
        for device, label in (
                (self.odrv0, 'odrv0'), (self.odrv1, 'odrv1')):
            for axis in (0, 1):
                enable_property = f'axis{axis}.config.enable_watchdog'

                # Use the numeric boolean form supported by the legacy ASCII
                # property parser. Verify it before relying on the watchdog.
                self._ascii_operation(
                    f'enable {label} axis{axis} watchdog',
                    lambda device=device, enable_property=enable_property:
                    device.write_property(enable_property, '1'))
                enabled_readback = self._ascii_operation(
                    f'verify {label} axis{axis} watchdog enabled',
                    lambda device=device, enable_property=enable_property:
                    device.read_float(enable_property))
                if int(enabled_readback) != 1:
                    raise RuntimeError(
                        f'{label} axis{axis} watchdog enable readback '
                        f'mismatch: {enabled_readback:.9g}')
                self._ascii_operation(
                    f'initial feed {label} axis{axis} watchdog',
                    lambda device=device, axis=axis:
                    device.feed_watchdog(axis))

        self.watchdog_test_active = True
        self.last_watchdog_test_check = time.monotonic()
        self.watchdog_test_check_index = 0
        for device in (self.odrv0, self.odrv1):
            device.set_query_wait_callback(
                self._feed_watchdogs_during_query_wait,
                self.watchdog_query_wait_feed_period)
        self.get_logger().info(
            'Watchdogs enabled on all four axes '
            f'(timeout={self.watchdog_test_timeout:.3g}s, '
            'verified by readback); feeding in the control cycle and while '
            'ASCII queries wait or retry')

    def _feed_watchdogs_during_query_wait(self):
        if self.watchdog_test_active:
            self._feed_watchdogs()

    def _feed_watchdogs(self):
        for device, label in (
                (self.odrv0, 'odrv0'), (self.odrv1, 'odrv1')):
            for axis in (0, 1):
                self._ascii_operation(
                    f'feed {label} axis{axis} watchdog',
                    lambda device=device, axis=axis:
                    device.feed_watchdog(axis))

    def _check_watchdog_test(self):
        if not self.watchdog_test_active:
            return
        now = time.monotonic()
        if (
                self.last_watchdog_test_check is not None and
                now - self.last_watchdog_test_check <
                self.watchdog_test_check_period):
            return
        self.last_watchdog_test_check = now

        watchdog_axes = (
            (self.odrv0, 'odrv0', 0),
            (self.odrv0, 'odrv0', 1),
            (self.odrv1, 'odrv1', 0),
            (self.odrv1, 'odrv1', 1),
        )
        device, label, axis = watchdog_axes[self.watchdog_test_check_index]
        self.watchdog_test_check_index = (
            self.watchdog_test_check_index + 1) % len(watchdog_axes)
        state = int(self._ascii_operation(
            f'diagnose {label} axis{axis} current_state',
            lambda: device.read_float(f'axis{axis}.current_state')))
        if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            return

        axis_error = int(self._ascii_operation(
            f'diagnose {label} axis{axis} error',
            lambda: device.read_float(f'axis{axis}.error')))
        watchdog_expired = bool(
            axis_error & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
        self.get_logger().error(
            'Watchdog-protected axis left closed-loop: '
            f'{label} axis{axis} state={state}, '
            f'error=0x{axis_error:x}, '
            f'watchdog_expired={watchdog_expired}')
        raise RuntimeError(
            f'{label} axis{axis} left closed-loop')

    def _reset_encoder_reference(self):
        right_turns, _ = self._ascii_operation(
            'read odrv0 axis0 feedback for encoder reference',
            lambda: self.odrv0.feedback(0))
        left_turns, _ = self._ascii_operation(
            'read odrv1 axis0 feedback for encoder reference',
            lambda: self.odrv1.feedback(0))
        self.right_pos = -right_turns * self.distance_per_turn
        self.left_pos = left_turns * self.distance_per_turn
        self.last_right_pos = self.right_pos
        self.last_left_pos = self.left_pos

    def _update_power_telemetry(self, force=False):
        now = time.monotonic()
        if (
                not force and
                self.last_telemetry_read is not None and
                now - self.last_telemetry_read < self.power_telemetry_period):
            return
        self.current0 = self._ascii_operation(
            'read odrv0 ibus',
            lambda: self.odrv0.read_float('ibus'))
        self.voltage0 = self._ascii_operation(
            'read odrv0 vbus_voltage',
            lambda: self.odrv0.read_float('vbus_voltage'))
        self.current1 = self._ascii_operation(
            'read odrv1 ibus',
            lambda: self.odrv1.read_float('ibus'))
        self.voltage1 = self._ascii_operation(
            'read odrv1 vbus_voltage',
            lambda: self.odrv1.read_float('vbus_voltage'))
        self.last_telemetry_read = now
        self._publish_battery_current(self.current0, self.current1)
        self._publish_battery_voltage(self.voltage0, self.voltage1)

    def _publish_battery_current(self, right, left):
        if not all(math.isfinite(value) for value in (right, left)):
            raise ValueError('Invalid battery current')
        self.current0, self.current1 = right, left
        self.battery_current_pub.publish(Float64MultiArray(data=[left, right]))
        self.last_current_publish = time.monotonic()

    def _publish_battery_voltage(self, right, left):
        if not all(math.isfinite(v) and 0 < v <= 60 for v in (right, left)):
            raise ValueError('Invalid battery voltage')
        self.voltage0, self.voltage1 = right, left
        # Dedicated data, no invented encoder positions or currents.
        self.battery_pub.publish(Float64MultiArray(data=[left, right]))
        self.last_voltage_publish = time.monotonic()

    def _start_voltage_poll(self):
        now = time.monotonic()
        if (self.last_voltage_publish is None or
                now - self.last_voltage_publish >= self.power_telemetry_period):
            self.voltage_poll = VoltagePoll(
                (self.odrv0, self.odrv1), self.voltage_query_timeout, now)
        elif (self.last_current_publish is None or
                now - self.last_current_publish >= self.power_telemetry_period):
            # Reuse the exclusive power-query slot. A control write occurs
            # between voltage and current queries; never stack their waits.
            self.voltage_poll = CurrentPoll(
                (self.odrv0, self.odrv1), self.voltage_query_timeout, now)

    def _start_runtime_poll(self):
        # Each port has at most one outstanding request. A velocity write is
        # sent by control() between polls, keeping watchdogs fed without mixing
        # writes into a pending reply on the legacy CDC firmware.
        if self.voltage_poll is not None or self.feedback_poll is not None:
            return
        now = time.monotonic()
        if (self.last_feedback_publish is None or
                now-self.last_feedback_publish >= self.wheel_feedback_period):
            self.feedback_stamp = self.get_clock().now().nanoseconds / 1e9
            self.feedback_poll = FeedbackPoll(
                (self.odrv0, self.odrv1), self.voltage_query_timeout, now)
            self.feedback_side_reported = [False, False]
        else:
            self._start_voltage_poll()

    def _finish_feedback_poll(self):
        if self.feedback_poll is None:
            return True
        values = self.feedback_poll.poll()
        for side, reply in enumerate(self.feedback_poll.values):
            if reply is not None and not self.feedback_side_reported[side]:
                self.feedback_side_pubs[side].publish(Empty())
                self.feedback_side_reported[side] = True
        if values is None:
            return False
        self.feedback_poll = None
        (right_turns, right_rps), (left_turns, left_rps) = values
        right = -right_turns*self.distance_per_turn
        left = left_turns*self.distance_per_turn
        if self.last_feedback_publish is None:
            self.last_right_pos, self.last_left_pos = right, left
        self._publish_odometry(
            right-self.last_right_pos, left-self.last_left_pos,
            -right_rps*self.distance_per_turn, left_rps*self.distance_per_turn,
            self.current0, self.voltage0, self.current1, self.voltage1,
            stamp=self.feedback_stamp)
        self.last_right_pos, self.last_left_pos = right, left
        self.last_feedback_publish = time.monotonic()
        return True

    def _finish_voltage_poll(self):
        if self.voltage_poll is None:
            return True
        values = self.voltage_poll.poll()
        if values is None:
            return False
        current_query = isinstance(self.voltage_poll, CurrentPoll)
        self.voltage_poll = None
        if current_query:
            self._publish_battery_current(*values)
        else:
            self._publish_battery_voltage(*values)
        return True

    def _safe_zero_velocity(self):
        for device in (self.odrv0, self.odrv1):
            if device is None:
                continue
            for axis in (0, 1):
                try:
                    device.set_velocity(axis, 0.0)
                except Exception:
                    pass

    def _safe_set_idle(self):
        for device in (self.odrv0, self.odrv1):
            if device is None:
                continue
            for axis in (0, 1):
                try:
                    device.set_state(axis, AXIS_STATE_IDLE)
                except Exception:
                    pass

    def _close_odrives(self):
        for device in (self.odrv0, self.odrv1):
            if device is not None:
                try:
                    device.close()
                except Exception:
                    pass
        self.odrv0 = None
        self.odrv1 = None

    def _connection_lost(self, cause):
        self.get_logger().error(
            f'ODrive ASCII connection lost: {cause!r}')
        self.reconnecting = True
        self.emergency = True
        self.armed = False
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self._safe_zero_velocity()
        self._safe_set_idle()
        self._close_odrives()
        raise RuntimeError(
            'Restarting process to reopen ODrive ASCII ports') from cause

    def control(self):
        # A hardware watchdog alone cannot detect a dead ROS command publisher:
        # this loop would otherwise keep feeding it with the last setpoint.
        if (self.last_cmd_vel_time is None or
                time.monotonic() - self.last_cmd_vel_time >= self.cmd_vel_timeout):
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
        if self.reconnecting:
            return
        if self._gui_estop_engaged():
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
        try:
            # Never overlap setpoints/watchdog writes with a pending reply.
            # This is nonblocking: ROS can still receive stop commands.
            if not self._finish_voltage_poll() or not self._finish_feedback_poll():
                return
        except Exception as error:
            self._connection_lost(error)
        right_vel, left_vel = self._calculate_track_velocities(
            self.target_linear_vel, self.target_angular_vel)
        turns_per_metre = 1.0 / self.distance_per_turn
        command_right_rps = right_vel * turns_per_metre
        command_left_rps = -left_vel * turns_per_metre
        self.last_command_right_rps = command_right_rps
        self.last_command_left_rps = command_left_rps

        if not self.armed:
            if (
                    abs(command_left_rps) <= 1e-6 and
                    abs(command_right_rps) <= 1e-6):
                try:
                    self._start_runtime_poll()
                except Exception as error:
                    self._connection_lost(error)
                return
            try:
                if self.last_feedback_publish is None or time.monotonic()-self.last_feedback_publish > .5:
                    self._start_runtime_poll()
                    return
                self._activate_odrives(
                    command_left_rps, command_right_rps)
            except Exception as error:
                self._connection_lost(error)
            return

        try:
            # ASCII setpoint commands (including `v`) feed that axis's
            # watchdog. In write-only mode every axis receives `v` below at
            # the control rate, so an additional `u` only doubles CDC load.
            if (
                    self.watchdog_test_active and
                    self.runtime_ascii_queries_enabled):
                self._feed_watchdogs()

            # The legacy CDC firmware can stop returning a query response if
            # watchdog-feed writes arrive while that response is pending.
            # Bulk synchronous queries may wait 5 s. In the default mode,
            # only bounded voltage/feedback polls may interrupt writes.
            if not self.runtime_ascii_queries_enabled:
                if not self.bypass_emergency_limits:
                    self.emergency = (
                        self.voltage0 < self.voltage_min or
                        self.voltage1 < self.voltage_min or
                        self.current0 > self.current_max or
                        self.current1 > self.current_max)
                if self.emergency:
                    command_left_rps = 0.0
                    command_right_rps = 0.0
                self._write_track_velocities(
                    command_left_rps, command_right_rps)
                self._start_runtime_poll()
                return

            right_turns, right_rps = self._ascii_operation(
                'read odrv0 axis0 feedback',
                lambda: self.odrv0.feedback(0))
            left_turns, left_rps = self._ascii_operation(
                'read odrv1 axis0 feedback',
                lambda: self.odrv1.feedback(0))

            self.right_pos = -right_turns * self.distance_per_turn
            self.left_pos = left_turns * self.distance_per_turn
            right_delta = self.right_pos - self.last_right_pos
            left_delta = self.left_pos - self.last_left_pos
            right_mps = -right_rps * self.distance_per_turn
            left_mps = left_rps * self.distance_per_turn

            self._update_power_telemetry()
            limits_ok = (
                self.current0 <= self.current_max and
                self.current1 <= self.current_max and
                self.voltage0 >= self.voltage_min and
                self.voltage1 >= self.voltage_min)
            self.emergency = not limits_ok
            if self.bypass_emergency_limits:
                self.emergency = False

            self._publish_odometry(
                right_delta, left_delta, right_mps, left_mps,
                self.current0, self.voltage0,
                self.current1, self.voltage1)

            if self.emergency:
                command_left_rps = 0.0
                command_right_rps = 0.0

            self._write_track_velocities(
                command_left_rps, command_right_rps)
            self._check_watchdog_test()

            self.last_right_pos = self.right_pos
            self.last_left_pos = self.left_pos
        except Exception as error:
            self._connection_lost(error)

    def _write_track_velocities(self, command_left_rps, command_right_rps):
        for axis in (0, 1):
            self._ascii_operation(
                f'write odrv0 axis{axis} velocity',
                lambda axis=axis:
                self.odrv0.set_velocity(axis, command_left_rps))
        for axis in (0, 1):
            self._ascii_operation(
                f'write odrv1 axis{axis} velocity',
                lambda axis=axis:
                self.odrv1.set_velocity(axis, command_right_rps))

    def _calculate_track_velocities(self, linear, angular):
        linear = max(
            -self.linear_vel_limit, min(self.linear_vel_limit, linear))
        angular = max(
            -self.angular_vel_limit, min(self.angular_vel_limit, angular))
        # Reverse the longitudinal command for the installed motor direction.
        # Keep the differential (yaw) component in its existing direction.
        right = -linear - (self.tread / 2.0) * angular
        left = -linear + (self.tread / 2.0) * angular
        return right, left

    def _publish_odometry(
            self, right_delta, left_delta, right_mps, left_mps,
            current0, voltage0, current1, voltage1, stamp=None):
        delta_linear = -(right_delta + left_delta) / 2.0
        delta_yaw = (
            (right_delta - left_delta) /
            (self.tread * self.yaw_calibration))
        velocity_linear = -(right_mps + left_mps) / 2.0
        velocity_yaw = (
            (right_mps - left_mps) /
            (self.tread * self.yaw_calibration))

        if abs(delta_yaw) < 245e-3:
            travelled = delta_linear
        else:
            turn_radius = delta_linear / delta_yaw
            travelled = (
                2.0 * turn_radius * math.sin(delta_yaw / 2.0))
        self.pose_x += travelled * math.cos(
            self.pose_yaw + delta_yaw / 2.0)
        self.pose_y += travelled * math.sin(
            self.pose_yaw + delta_yaw / 2.0)
        self.pose_yaw += delta_yaw

        elapsed = (
            (self.get_clock().now() - self.start_time).nanoseconds / 1e9)
        message = Float64MultiArray()
        message.data = [
            self.pose_x, self.pose_y, self.pose_yaw,
            velocity_linear, velocity_yaw,
            current0, voltage0, current1, voltage1, elapsed,
            self.get_clock().now().nanoseconds/1e9 if stamp is None else stamp,
        ]
        self.odom_pub.publish(message)
        # Raw encoder positions use the opposite sign to forward motion.
        # Accumulate measured encoder deltas, never commanded velocity.
        self.forward_travel[0] -= right_delta
        self.forward_travel[1] -= left_delta
        self.wheel_travel_pub.publish(Float64MultiArray(data=[
            *self.forward_travel, elapsed, message.data[-1]]))

    def twist_callback(self, message):
        if not all(math.isfinite(v) for v in (message.linear.x, message.angular.z)):
            self.target_linear_vel = self.target_angular_vel = 0.0
            self.last_cmd_vel_time = None
            return
        self.last_cmd_vel_time = time.monotonic()
        if self._gui_estop_engaged():
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            return
        if self.reconnecting:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            return
        if not self.armed:
            self.target_linear_vel = message.linear.x
            self.target_angular_vel = message.angular.z
            return
        if self.emergency:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            return
        self.target_linear_vel = message.linear.x
        self.target_angular_vel = message.angular.z

    def on_gui_estop(self, message):
        self.gui_estop_active = message.data
        self.gui_estop_last_rx = time.monotonic()
        if message.data:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0

    def _gui_estop_engaged(self):
        return (self.gui_estop_enabled and
                (self.gui_estop_active or self.gui_estop_last_rx is None or
                 time.monotonic() - self.gui_estop_last_rx > self.gui_estop_timeout_sec))

    def stop(self):
        self._safe_zero_velocity()
        self._safe_set_idle()
        self._close_odrives()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Abot04TwistDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
