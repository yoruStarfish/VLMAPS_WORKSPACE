#!/usr/bin/env python3
"""Turn microphone input into prompts for the existing navigation node."""

import os
import threading
import time

import numpy as np
import pyaudio
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from voice_server.microphone import open_microphone


class VoiceServer(Node):
    """Own the microphone in one worker; keep ROS callbacks nonblocking."""

    def __init__(self):
        super().__init__('voice_server')
        defaults = {
            'command_topic': '/user_command',
            'language_code': 'en',
            'model_path': '/workspace/models/faster-whisper-small.en',
            'device': 'cpu',
            'compute_type': 'int8',
            'cpu_threads': 4,
            'input_device_index': -1,
            'input_device_name': '',
            'max_record_seconds': 15.0,
            'silence_seconds': 2.5,
            'speech_start_timeout': 8.0,
            'silence_threshold': 100.0,
            'wakeword_enabled': False,
            'wakeword_model_path': '',
            'wakeword_threshold': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_record_seconds', 'silence_seconds', 'speech_start_timeout'):
            if self.settings[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if self.settings['silence_threshold'] < 0:
            raise ValueError('silence_threshold must be nonnegative')
        if not 0 < self.settings['wakeword_threshold'] <= 1:
            raise ValueError('wakeword_threshold must be in (0, 1]')

        if self.settings['cpu_threads'] < 1:
            raise ValueError('cpu_threads must be positive')
        if self.settings['device'] not in ('cpu', 'cuda', 'auto'):
            raise ValueError('device must be cpu, cuda, or auto')
        model_path = os.path.expanduser(self.settings['model_path'])
        if not os.path.isdir(model_path):
            raise ValueError(f'Model directory not found: {model_path}. '
                             'Download the model before starting; see voice_server/README.md.')
        self.get_logger().info(f'Loading local Whisper model: {model_path}')
        self._model = WhisperModel(
            model_path, device=self.settings['device'],
            compute_type=self.settings['compute_type'],
            cpu_threads=self.settings['cpu_threads'], local_files_only=True)
        if not self._model.model.is_multilingual and self.settings['language_code'] != 'en':
            raise ValueError('English-only model requires language_code=en')

        self.command_pub = self.create_publisher(
            String, self.settings['command_topic'], 10)
        self.transcript_pub = self.create_publisher(String, '~/transcript', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self._lock = threading.Lock()
        self._busy = False
        self._stop = threading.Event()
        self._listen = threading.Event()
        self._cancel = threading.Event()
        self._wake_enabled = threading.Event()
        self._wake_model = None
        if self.settings['wakeword_enabled']:
            path = os.path.expanduser(self.settings['wakeword_model_path'])
            if not os.path.isfile(path):
                raise ValueError('wakeword_model_path must point to an existing ONNX model')
            from openwakeword.model import Model
            self._wake_model = Model(wakeword_models=[path], inference_framework='onnx')
            self._wake_enabled.set()

        self.start_service = self.create_service(Trigger, '~/start', self._start)
        self.stop_service = self.create_service(Trigger, '~/stop', self._stop_listening)
        self._worker = threading.Thread(target=self._run, name='voice_input', daemon=True)
        self._worker.start()
        self.get_logger().info(
            'Ready: /voice_server/start enables continuous listening; /voice_server/stop pauses it. Prompts go to '
            + self.settings['command_topic'])

    def _status(self, value):
        self.status_pub.publish(String(data=value))
        self.get_logger().info(value)

    def _start(self, request, response):
        del request
        with self._lock:
            if self._stop.is_set():
                response.success = False
                response.message = 'Node is shutting down.'
            elif self._listen.is_set():
                response.success = True
                response.message = 'Continuous listening is already enabled.'
            elif self._busy:
                response.success = False
                response.message = 'Previous session is stopping; retry after idle.'
            else:
                self._cancel.clear()
                self._listen.set()
                response.success = True
                response.message = 'Continuous listening enabled; wait for listening before speaking.'
        return response

    def _stop_listening(self, request, response):
        del request
        with self._lock:
            self._listen.clear()
            self._wake_enabled.clear()
            self._cancel.set()
            self._status('stopping' if self._busy else 'idle')
        response.success = True
        response.message = ('Listening disabled; pending text will be discarded. '
                            'Navigation is not cancelled.')
        return response

    def _open_microphone(self, audio, chunk):
        return open_microphone(
            audio, self.settings['input_device_index'], chunk, self.get_logger(),
            device_name=self.settings['input_device_name'])

    def _wait_for_wakeword(self):
        audio = pyaudio.PyAudio()
        stream = None
        try:
            stream = self._open_microphone(audio, 1280)
            while (not self._stop.is_set() and not self._listen.is_set()
                   and self._wake_enabled.is_set()):
                frame = np.frombuffer(
                    stream.read(1280, exception_on_overflow=False), dtype=np.int16)
                scores = self._wake_model.predict(frame)
                if any(score >= self.settings['wakeword_threshold']
                       for score in scores.values()):
                    with self._lock:
                        if self._wake_enabled.is_set() and not self._cancel.is_set():
                            self._listen.set()
                    break
        finally:
            if stream is not None:
                stream.close()
            audio.terminate()
            self._wake_model.reset()

    def _record(self):
        """Capture one utterance with bounded duration and trailing silence."""
        audio = pyaudio.PyAudio()
        stream = None
        frames = []
        heard_speech = False
        peak_level = 0.0
        max_mean_level = 0.0
        try:
            stream = self._open_microphone(audio, 1024)
            self._status('listening')
            started = last_sound = time.monotonic()
            while not self._stop.is_set() and not self._cancel.is_set():
                now = time.monotonic()
                if now - started >= self.settings['max_record_seconds']:
                    break
                data = stream.read(1024, exception_on_overflow=False)
                frames.append(data)
                # Convert before abs to avoid int16 overflow at -32768.
                amplitudes = np.abs(np.frombuffer(data, dtype=np.int16).astype(np.float32))
                level = float(amplitudes.mean())
                peak_level = max(peak_level, float(amplitudes.max()))
                max_mean_level = max(max_mean_level, level)
                if level > self.settings['silence_threshold']:
                    heard_speech = True
                    last_sound = time.monotonic()
                elif heard_speech:
                    if time.monotonic() - last_sound >= self.settings['silence_seconds']:
                        break
                elif time.monotonic() - started >= self.settings['speech_start_timeout']:
                    break
        finally:
            if stream is not None:
                stream.close()
            audio.terminate()
        if self._cancel.is_set() or self._stop.is_set():
            return []
        self.get_logger().info(
            f'Recording levels: peak={peak_level:.0f}, max_mean={max_mean_level:.1f}, '
            f'threshold={self.settings["silence_threshold"]:.1f}')
        if not heard_speech:
            self.get_logger().warning(
                'No audio exceeded the speech threshold; check input_device_index, '
                'microphone mute/input gain, and the recording levels above.')
        return frames if heard_speech else []

    def _transcribe(self, frames):
        """Transcribe 16 kHz mono PCM locally, collecting all speech segments."""
        if not frames or self._stop.is_set() or self._cancel.is_set():
            return ''
        audio = np.frombuffer(b''.join(frames), dtype='<i2').astype(np.float32) / 32768.0
        segments, _ = self._model.transcribe(
            audio, language=self.settings['language_code'], task='transcribe',
            beam_size=5, vad_filter=True, condition_on_previous_text=False)
        parts = []
        for segment in segments:
            if self._stop.is_set() or self._cancel.is_set():
                return ''
            text = segment.text.strip()
            if text:
                parts.append(text)
        return ' '.join(parts)

    def _run(self):
        self._status('idle')
        while not self._stop.is_set():
            try:
                if (self._wake_model is not None and self._wake_enabled.is_set()
                        and not self._listen.is_set()):
                    with self._lock:
                        self._busy = True
                    self._wait_for_wakeword()
                if not self._listen.wait(timeout=0.1):
                    continue
                with self._lock:
                    if not self._listen.is_set() or self._stop.is_set():
                        continue
                    self._busy = True
                frames = self._record()
                if self._stop.is_set() or self._cancel.is_set():
                    continue
                if frames:
                    self._status('recognizing')
                    text = self._transcribe(frames)
                else:
                    text = ''
                # Serialize publication with Stop so no late result is published
                # after a successful Stop response.
                with self._lock:
                    if self._stop.is_set() or self._cancel.is_set():
                        continue
                    if text:
                        self.transcript_pub.publish(String(data=text))
                        self.command_pub.publish(String(data=text))
                        self.get_logger().info(f'Published prompt: {text}')
                        self._status('published')
                    else:
                        self._status('no_speech')
            except Exception as exc:
                if not self._stop.is_set() and not self._cancel.is_set():
                    self.get_logger().error(f'Voice input failed: {exc}')
                    with self._lock:
                        self._listen.clear()
                        self._wake_enabled.clear()
                        self._cancel.set()
                    self._status('error')
                    self.get_logger().warning(
                        'Listening paused after failure. Fix the input device, then call Start again.')
            finally:
                with self._lock:
                    was_busy = self._busy
                    self._busy = False
                    if was_busy and not self._listen.is_set() and not self._stop.is_set():
                        self._status('idle')

    def close(self):
        """Stop recording and wait for the current local inference to finish."""
        self._stop.set()
        self._cancel.set()
        self._wake_enabled.clear()
        self._listen.set()
        self._worker.join()
        self._model = None


def main(args=None):
    """Run the voice input node."""
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
