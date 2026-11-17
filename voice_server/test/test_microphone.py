"""Verify hardware format fallback and resampling without a microphone."""
import unittest
from unittest.mock import Mock

import numpy as np

from voice_server.microphone import MicrophoneStream, open_microphone


class MicrophoneTests(unittest.TestCase):
    def test_name_selects_input_even_when_index_changes(self):
        audio = Mock()
        devices = [
            {'index': 8, 'name': 'output hw:1,31', 'maxInputChannels': 0},
            {'index': 11, 'name': 'mic hw:1,6', 'maxInputChannels': 2,
             'defaultSampleRate': 48000}]
        audio.get_device_count.return_value = len(devices)
        audio.get_device_info_by_index.side_effect = lambda i: devices[i]
        audio.is_format_supported.return_value = True
        open_microphone(audio, 8, 1024, Mock(), device_name='hw:1,6')
        self.assertEqual(audio.open.call_args.kwargs['input_device_index'], 11)

    def test_missing_name_never_falls_back_to_wrong_device(self):
        audio = Mock()
        audio.get_device_count.return_value = 1
        audio.get_device_info_by_index.return_value = {
            'index': 8, 'name': 'output hw:1,31', 'maxInputChannels': 0}
        with self.assertRaisesRegex(ValueError, 'found 0'):
            open_microphone(audio, 8, 1024, Mock(), device_name='hw:1,6')
        audio.open.assert_not_called()

    def test_native_rate_fallback(self):
        audio = Mock()
        audio.get_default_input_device_info.return_value = {
            'index': 4, 'name': 'test', 'maxInputChannels': 2, 'defaultSampleRate': 48000}
        audio.is_format_supported.side_effect = lambda rate, **kw: rate == 48000
        stream = open_microphone(audio, -1, 1024, Mock())
        self.assertEqual(stream.rate, 48000)
        self.assertEqual(audio.open.call_args.kwargs['rate'], 48000)
        self.assertEqual(audio.open.call_args.kwargs['input_device_index'], 4)

    def test_stereo_only_device(self):
        audio = Mock()
        audio.get_device_info_by_index.return_value = {
            'index': 8, 'name': 'stereo', 'maxInputChannels': 2, 'defaultSampleRate': 48000}
        audio.is_format_supported.side_effect = (
            lambda rate, **kw: rate == 48000 and kw['input_channels'] == 2)
        stream = open_microphone(audio, 8, 1280, Mock())
        self.assertEqual(stream.channels, 2)

    def test_resampling_preserves_duration_and_pitch_across_reads(self):
        for rate in (16000, 44100, 48000):
            with self.subTest(rate=rate):
                position = 0
                def read(count, **kwargs):
                    nonlocal position
                    t = np.arange(position, position + count) / rate
                    position += count
                    wave = (10000 * np.sin(2*np.pi*1000*t)).astype(np.int16)
                    return np.column_stack([wave, wave]).tobytes()
                raw = Mock()
                raw.read.side_effect = read
                stream = MicrophoneStream(raw, rate, 2, 1280)
                output = np.frombuffer(b''.join(stream.read(1280) for _ in range(25)),
                                       dtype=np.int16)
                self.assertEqual(len(output), 32000)
                peak = np.argmax(np.abs(np.fft.rfft(output))) * 16000 / len(output)
                self.assertAlmostEqual(peak, 1000, delta=1)
                self.assertLessEqual(abs(position - 2 * rate), stream.input_chunk)
                stream.close()
                raw.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
