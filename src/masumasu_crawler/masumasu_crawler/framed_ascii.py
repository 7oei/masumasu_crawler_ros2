"""Bounded ASCII transactions bracketed by volatile test_property markers.

Only the response between this transaction's markers is accepted. Marker reads
can push a delayed CDC reply out without repeating the actual query.
"""
import os
import secrets
import select
import time


class FramedQuery:
    def __init__(self, device, command, timeout):
        self.device = device
        self.deadline = time.monotonic() + timeout
        nonce = secrets.randbelow(1_000_000_000) + 1
        self.start, self.end = str(nonce), str(nonce + 1)
        self.started = False
        self.lines = []
        self.discarded = 0
        self.pumps = 0
        self.next_pump = time.monotonic() + .01
        # One ordered write reduces interleaving; test_property is volatile.
        device.write(f'w test_property {self.start}\nr test_property\n'
                     f'{command}\nw test_property {self.end}\nr test_property')

    def poll(self):
        now = time.monotonic()
        if now >= self.deadline:
            raise TimeoutError('ASCII framing deadline exceeded '
                               f'(discarded={self.discarded}, pumps={self.pumps})')
        d = self.device
        if select.select([d.fd], [], [], 0)[0]:
            chunk = os.read(d.fd, 4096)
            if not chunk:
                raise OSError('ODrive disconnected during framed query')
            d._rx_buffer.extend(chunk)
        if len(d._rx_buffer) > 8192:
            raise ValueError('Oversized ASCII framing buffer')
        while b'\n' in d._rx_buffer:
            raw, _, rest = d._rx_buffer.partition(b'\n')
            d._rx_buffer = bytearray(rest)
            line = raw.decode('ascii').strip()
            if not self.started:
                if line == self.start:
                    self.started = True
                else:
                    self.discarded += 1
                    if self.discarded > 256:
                        raise ValueError('Too many stale ASCII responses')
            elif line == self.end:
                if len(self.lines) != 1:
                    raise ValueError('Invalid framed response count: '
                                     f'{len(self.lines)}; expected one')
                callback = getattr(d, 'frame_report_callback', None)
                if callback is not None:
                    callback(self.discarded, self.pumps)
                return self.lines[0]
            else:
                self.lines.append(line)
                if len(self.lines) > 1:
                    raise ValueError('Multiple responses inside ASCII frame')
        if now >= self.next_pump and self.pumps < 8:
            # Legacy CDC can hold responses until another read command arrives.
            # Re-read only the END marker, never repeat the requested operation.
            d.write('r test_property')
            self.pumps += 1
            self.next_pump = now + .01
        return None
