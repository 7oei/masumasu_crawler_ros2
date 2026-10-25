from collections import deque

import masumasu_crawler.armadillo_driver as driver_module
from masumasu_crawler.armadillo_driver import ODriveAscii


class ScriptedAscii(ODriveAscii):
    def __init__(self, results, retries=1):
        self.path = '/dev/test-odrive'
        self.resync_timeout = 5.0
        self.results = deque(results)
        self.query_retries = retries
        self.events = []

    def _query_once(self, command):
        self.events.append(('query', command))
        result = self.results.popleft()
        if isinstance(result, Exception):
            raise result
        return result

    def _resynchronize(self):
        self.events.append(('sync', None))


def test_timeout_resynchronizes_before_resending_query():
    device = ScriptedAscii((TimeoutError('late'), '1.0 2.0'))

    assert device.query('f 0') == '1.0 2.0'
    assert device.events == [
        ('query', 'f 0'),
        ('sync', None),
        ('query', 'f 0'),
    ]


def test_query_is_not_resent_when_resynchronization_fails():
    class FailedSyncAscii(ScriptedAscii):
        def _resynchronize(self):
            self.events.append(('sync', None))
            raise TimeoutError('no nonce')

    device = FailedSyncAscii((TimeoutError('late'),), retries=2)

    try:
        device.query('r axis0.current_state')
    except TimeoutError:
        pass
    else:
        raise AssertionError('query unexpectedly succeeded')

    assert device.events == [
        ('query', 'r axis0.current_state'),
        ('sync', None),
        ('sync', None),
    ]


def test_timeout_without_resync_requires_reopen_and_is_not_retried(
        monkeypatch):
    device = ScriptedAscii((TimeoutError('late'),), retries=2)
    device.resync_enabled = False
    device.fd = 123
    device.timeout = 0.2
    device._rx_buffer = bytearray()
    monkeypatch.setattr(driver_module.termios, 'tcflush', lambda *_: None)

    try:
        device.query('f 0')
    except TimeoutError as error:
        assert 'connection must be reopened' in str(error)
    else:
        raise AssertionError('query unexpectedly succeeded')

    assert device.events == [('query', 'f 0')]
