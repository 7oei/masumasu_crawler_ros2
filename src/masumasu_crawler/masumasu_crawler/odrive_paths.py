"""Resolve the original robot ODrives, ported from k_4_odrive_paths.bash."""
import glob
import os
import select
import termios
import time


def resolve_odrive_paths(enumeration_timeout=5.0):
    fixed_ports = ('/dev/odrive-right', '/dev/odrive-left')
    if all(os.path.exists(port) for port in fixed_ports):
        if os.path.realpath(fixed_ports[0]) == os.path.realpath(fixed_ports[1]):
            raise RuntimeError('ODrive right and left aliases point to the same device')
        return fixed_ports

    EXPECTED = {
        '316F324C3431': 'right',
        '3160325F3431': 'left',
        '335831653536': 'right',
        '365338553030': 'left',
    }
    ASCII_SERIAL_PREFIX = {
        # Serial values observed before reflashing the current K-4 controllers.
        '597310552': 'left',
        '564538788': 'right',
        # Serial values reported by the custom v0.5.5 firmware.  The ASCII
        # protocol reports the leading decimal digits of the full serial.
        '543536549': 'right',
        '542892317': 'left',
    }


    def query_ascii_serial(port):
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            attrs[4] = termios.B115200
            attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            for _ in range(3):
                termios.tcflush(fd, termios.TCIFLUSH)
                os.write(fd, b'r serial_number\n')
                deadline = time.monotonic() + 0.5
                response = bytearray()
                while time.monotonic() < deadline:
                    ready, _, _ = select.select(
                        [fd], [], [], max(0.0, deadline - time.monotonic()))
                    if not ready:
                        break
                    response.extend(os.read(fd, 4096))
                    if b'\n' in response:
                        value = bytes(response).splitlines()[0].decode(
                            'ascii').strip()
                        if value.isdigit():
                            return value
            raise TimeoutError('no valid serial_number response')
        finally:
            os.close(fd)


    def list_odrive_ttys():
        result = []
        for sys_tty in sorted(glob.glob('/sys/class/tty/ttyACM*')):
            try:
                interface_path = os.path.realpath(os.path.join(sys_tty, 'device'))
                usb_path = os.path.dirname(interface_path)
                with open(os.path.join(usb_path, 'idVendor')) as file:
                    vendor = file.read().strip().upper()
                with open(os.path.join(usb_path, 'idProduct')) as file:
                    product = file.read().strip().upper()
                if (vendor, product) == ('1209', '0D32'):
                    result.append(sys_tty)
            except OSError:
                continue
        return result


    # Both tty devices can appear just after a power cycle, while the launch
    # script is already running.  Give udev a short window to finish enumeration.
    enumeration_deadline = time.monotonic() + enumeration_timeout
    while True:
        odrive_ttys = list_odrive_ttys()
        if len(odrive_ttys) >= 2 or time.monotonic() >= enumeration_deadline:
            break
        time.sleep(0.1)

    found = {}
    unknown = []
    for sys_tty in odrive_ttys:
        topology = os.path.basename(sys_tty)
        try:
            port = '/dev/' + os.path.basename(sys_tty)
            interface_path = os.path.realpath(os.path.join(sys_tty, 'device'))
            usb_path = os.path.dirname(interface_path)
            topology = os.path.basename(usb_path)
            with open(os.path.join(usb_path, 'idVendor')) as file:
                vendor = file.read().strip().upper()
            with open(os.path.join(usb_path, 'idProduct')) as file:
                product = file.read().strip().upper()
            if (vendor, product) != ('1209', '0D32'):
                continue
            try:
                with open(os.path.join(usb_path, 'serial')) as file:
                    serial = file.read().strip().upper()
            except (OSError, UnicodeError):
                serial = ''
            role = EXPECTED.get(serial)
            if role is None:
                ascii_serial = query_ascii_serial(port)
                role = ASCII_SERIAL_PREFIX.get(ascii_serial)
                if role is None:
                    unknown.append((topology,
                                    f'usb={serial!r} ascii={ascii_serial!r}'))
                    continue
            found[role] = topology
        except (OSError, TimeoutError) as error:
            unknown.append((topology, str(error)))

    # One v3.6 board is known to expose a corrupted USB serial string.  It can
    # still be identified unambiguously when the other known K-4 board is present
    # and there is exactly one otherwise-unidentified ODrive.
    missing = [role for role in ('right', 'left') if role not in found]
    if len(missing) == 1 and len(unknown) == 1:
        found[missing[0]] = unknown[0][0]
        unknown.clear()

    missing = [role for role in ('right', 'left') if role not in found]
    if missing:
        detail = ', '.join(
            f'{path} serial={serial!r}' for path, serial in unknown)
        if not detail:
            detail = 'both known K-4 ODrives are not connected'
        raise RuntimeError(
            f"Could not resolve K-4 ODrive(s) {', '.join(missing)}: {detail}")

    # odrive0 is the right controller and odrive1 is the left controller.
    return found['right'], found['left']
