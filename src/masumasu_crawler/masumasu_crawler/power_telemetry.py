"""Bounded, nonblocking telemetry reads on the driver's existing CDC ports."""
import math
import os
import select
import time


class TelemetryPoll:
    command = ''
    label = 'telemetry'

    def __init__(self, devices, timeout, now=None):
        self.devices = devices  # odrive0 (right), odrive1 (left)
        self.deadline = (time.monotonic() if now is None else now) + timeout
        self.buffers = [bytearray() for _ in devices]
        self.values = [None for _ in devices]
        # The caller owns both ports exclusively until completion or failure.
        self.frames = [
            device.begin_framed_query(self.command, timeout)
            if getattr(device, 'resync_enabled', False) else None
            for device in devices]
        for device, frame in zip(devices, self.frames):
            if frame is None:
                device.write(self.command)

    def poll(self, now=None):
        now = time.monotonic() if now is None else now
        if now >= self.deadline:
            raise TimeoutError(f'{self.label} response deadline exceeded; reopen ports')
        for i, device in enumerate(self.devices):
            if self.values[i] is not None:
                continue
            if self.frames[i] is not None:
                line = self.frames[i].poll()
                if line is not None:
                    self.values[i] = self.parse(line)
                continue
            readable, _, _ = select.select([device.fd], [], [], 0)
            if not readable:
                continue
            chunk = os.read(device.fd, 4096)
            if not chunk:
                raise OSError(f'ODrive disconnected during {self.label} read')
            self.buffers[i].extend(chunk)
            if len(self.buffers[i]) > 128:
                raise ValueError(f'Oversized {self.label} response')
            if b'\n' not in self.buffers[i]:
                continue
            line, _, remainder = self.buffers[i].partition(b'\n')
            if remainder:
                raise ValueError(f'Unexpected extra {self.label} response; reopen ports')
            self.values[i] = self.parse(line.decode('ascii').strip())
        if any(value is None for value in self.values):
            return None
        return tuple(self.values)


class VoltagePoll(TelemetryPoll):
    command = 'r vbus_voltage'
    label = 'Battery voltage'

    @staticmethod
    def parse(line):
        value = float(line)
        if not math.isfinite(value) or not 0 < value <= 60:
            raise ValueError('Invalid battery voltage')
        return value


class CurrentPoll(TelemetryPoll):
    command = 'r ibus'
    label = 'Battery current'

    @staticmethod
    def parse(line):
        value = float(line)
        if not math.isfinite(value):
            raise ValueError('Invalid battery current')
        return value  # Negative is regenerative current, not a parse error.


class FeedbackPoll(TelemetryPoll):
    command = 'f 0'
    label = 'Wheel feedback'

    @staticmethod
    def parse(line):
        values = tuple(float(v) for v in line.split())
        if len(values) != 2 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid wheel position/velocity feedback')
        return values
