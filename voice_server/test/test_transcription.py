"""Offline regression checks for the microphone-to-Whisper boundary."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from voice_server.voice_server import VoiceServer


class TranscriptionTests(unittest.TestCase):
    def setUp(self):
        self.node = VoiceServer.__new__(VoiceServer)
        self.node.settings = {'language_code': 'en'}
        self.node._stop = threading.Event()
        self.node._cancel = threading.Event()
        self.node._model = Mock()

    def test_waits_for_speech_beyond_trailing_silence_timeout(self):
        clock = [0.0]
        stream = Mock()
        def read(*args, **kwargs):
            clock[0] += 0.5
            amplitude = 500 if 4.0 <= clock[0] <= 4.5 else 0
            return np.full(1024, amplitude, dtype=np.int16).tobytes()
        stream.read.side_effect = read
        self.node.settings.update(
            max_record_seconds=15.0, silence_seconds=2.5,
            speech_start_timeout=8.0, silence_threshold=100.0)
        self.node._open_microphone = Mock(return_value=stream)
        self.node._status = Mock()
        with patch('voice_server.voice_server.time.monotonic', side_effect=lambda: clock[0]), \
                patch('voice_server.voice_server.pyaudio.PyAudio') as audio, \
                patch.object(VoiceServer, 'get_logger', return_value=Mock()):
            frames = self.node._record()
        self.assertTrue(frames)
        self.assertAlmostEqual(clock[0], 7.0)
        stream.close.assert_called_once()
        audio.return_value.terminate.assert_called_once()

    def test_pcm_normalization_and_complete_transcript(self):
        self.node._model.transcribe.return_value = (
            iter([SimpleNamespace(text=' Go to the chair. '),
                  SimpleNamespace(text=''), SimpleNamespace(text='Then turn left.')]), None)
        pcm = np.array([-32768, 0, 16384, 32767], dtype='<i2').tobytes()
        text = self.node._transcribe([pcm[:4], pcm[4:]])
        self.assertEqual(text, 'Go to the chair. Then turn left.')
        args, kwargs = self.node._model.transcribe.call_args
        np.testing.assert_allclose(args[0], [-1, 0, .5, 32767 / 32768])
        self.assertEqual(args[0].dtype, np.float32)
        self.assertEqual(kwargs['language'], 'en')
        self.assertTrue(kwargs['vad_filter'])
        self.assertEqual(kwargs['task'], 'transcribe')

    def test_empty_and_shutdown_skip_inference(self):
        self.assertEqual(self.node._transcribe([]), '')
        self.node._stop.set()
        self.assertEqual(self.node._transcribe([b'\x00\x00']), '')
        self.node._model.transcribe.assert_not_called()

    def test_vad_empty_result(self):
        self.node._model.transcribe.return_value = (iter([]), None)
        self.assertEqual(self.node._transcribe([b'\x00\x00']), '')

    def test_shutdown_discards_partial_text(self):
        def segments():
            yield SimpleNamespace(text='Go forward')
            self.node._stop.set()
            yield SimpleNamespace(text='one meter')
        self.node._model.transcribe.return_value = (segments(), None)
        self.assertEqual(self.node._transcribe([b'\x00\x00']), '')


if __name__ == '__main__':
    unittest.main()
