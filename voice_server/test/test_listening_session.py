"""Exercise single-utterance sessions and Stop during inference."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from voice_server.voice_server import VoiceServer


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.node = VoiceServer.__new__(VoiceServer)
        for name in ('_stop', '_listen', '_cancel', '_wake_enabled'):
            setattr(self.node, name, threading.Event())
        self.node._lock = threading.Lock()
        self.node._busy = False
        self.node._wake_model = None
        self.node._status = Mock()
        self.node.command_pub = Mock()
        self.node.transcript_pub = Mock()
        self.node._record = Mock(return_value=[b'a'])
        self.node._transcribe = Mock(return_value='Go to the chair.')
        self.logger = patch.object(VoiceServer, 'get_logger', return_value=Mock())
        self.logger.start()

    def tearDown(self):
        self.logger.stop()

    def start(self):
        return self.node._start(None, SimpleNamespace())

    def stop(self):
        return self.node._stop_listening(None, SimpleNamespace())

    def test_each_utterance_requires_another_start(self):
        idle = threading.Event()
        def status(value):
            if value == 'idle' and self.node._record.called:
                idle.set()
        self.node._status.side_effect = status
        self.assertTrue(self.start().success)
        worker = threading.Thread(target=self.node._run)
        worker.start()
        try:
            self.assertTrue(idle.wait(2))
            self.assertFalse(self.node._listen.is_set())
            self.assertFalse(self.node._busy)
            self.node._record.assert_called_once()
            self.node.command_pub.publish.assert_called_once()
            idle.clear()
            self.assertTrue(self.start().success)
            self.assertTrue(idle.wait(2))
            self.assertEqual(self.node.command_pub.publish.call_count, 2)
        finally:
            self.node._stop.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(self.node.command_pub.publish.call_count, 2)

    def test_no_speech_returns_to_idle_without_retry(self):
        idle = threading.Event()
        self.node._record.return_value = []
        def status(value):
            if value == 'idle' and self.node._record.called:
                idle.set()
        self.node._status.side_effect = status
        self.start()
        worker = threading.Thread(target=self.node._run)
        worker.start()
        try:
            self.assertTrue(idle.wait(2))
            self.assertFalse(self.node._listen.is_set())
            self.node._record.assert_called_once()
            self.node._transcribe.assert_not_called()
            self.node.command_pub.publish.assert_not_called()
        finally:
            self.node._stop.set()
            worker.join(2)

    def test_stop_discards_inflight_result_and_can_restart(self):
        entered, release = threading.Event(), threading.Event()
        def transcribe(frames):
            entered.set()
            release.wait(2)
            return 'Old command'
        self.node._transcribe.side_effect = transcribe
        self.start()
        worker = threading.Thread(target=self.node._run)
        worker.start()
        try:
            self.assertTrue(entered.wait(2))
            self.assertTrue(self.stop().success)
            self.assertFalse(self.start().success)  # Still draining old inference.
            release.set()
            self.node._stop.set()
            worker.join(2)
            self.node.command_pub.publish.assert_not_called()
            self.assertFalse(self.node._busy)
            self.node._stop.clear()
            self.assertTrue(self.start().success)
            self.assertFalse(self.node._cancel.is_set())
        finally:
            release.set()
            self.node._stop.set()
            worker.join(2)

    def test_input_failure_pauses_session_without_retry_loop(self):
        failed = threading.Event()
        self.node._record.side_effect = ValueError('No input channels')
        def status(value):
            if value == 'error':
                failed.set()
        self.node._status.side_effect = status
        self.start()
        worker = threading.Thread(target=self.node._run)
        worker.start()
        try:
            self.assertTrue(failed.wait(2))
            self.assertFalse(self.node._listen.is_set())
            self.assertTrue(self.node._cancel.is_set())
        finally:
            self.node._stop.set()
            worker.join(2)
        self.node._record.assert_called_once()
        self.node.command_pub.publish.assert_not_called()

    def test_repeated_start_does_not_spawn_another_session(self):
        self.assertTrue(self.start().success)
        self.assertFalse(self.start().success)
        self.node._busy = True
        self.assertFalse(self.start().success)
        self.assertTrue(self.node._listen.is_set())

    def test_stop_disables_wakeword_reactivation(self):
        self.node._wake_enabled.set()
        self.start()
        self.assertTrue(self.stop().success)
        self.assertFalse(self.node._wake_enabled.is_set())
        self.assertFalse(self.node._listen.is_set())
        self.assertTrue(self.node._cancel.is_set())


if __name__ == '__main__':
    unittest.main()
