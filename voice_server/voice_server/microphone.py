"""Adapt supported microphone formats to a continuous 16 kHz mono stream."""

import math

import av
import numpy as np
import pyaudio


class MicrophoneStream:
    """Resample continuously, preserving filter state between audio reads."""

    def __init__(self, stream, rate, channels, output_chunk):
        self.stream = stream
        self.rate = rate
        self.channels = channels
        self.input_chunk = max(1, math.ceil(output_chunk * rate / 16000))
        self.resampler = av.AudioResampler(format='s16', layout='mono', rate=16000)
        self.pending = bytearray()
        self.position = 0

    def read(self, frames, exception_on_overflow=False):
        required = frames * 2
        while len(self.pending) < required:
            raw = self.stream.read(
                self.input_chunk, exception_on_overflow=exception_on_overflow)
            samples = np.frombuffer(raw, dtype=np.int16).reshape(-1, self.channels)
            # Average stereo before resampling to keep the existing volume threshold scale.
            mono = np.rint(samples.astype(np.float32).mean(axis=1)).astype(np.int16)
            frame = av.AudioFrame.from_ndarray(mono.reshape(1, -1), format='s16', layout='mono')
            frame.sample_rate = self.rate
            frame.pts = self.position
            self.position += len(mono)
            for converted in self.resampler.resample(frame):
                self.pending.extend(converted.to_ndarray().tobytes())
        data = bytes(self.pending[:required])
        del self.pending[:required]
        return data

    def close(self):
        self.stream.close()


def open_microphone(audio, device_index, output_chunk, logger, device_name=""):
    """Probe the selected device instead of assuming hardware accepts 16 kHz."""
    if device_name:
        inputs = [audio.get_device_info_by_index(i) for i in range(audio.get_device_count())]
        inputs = [d for d in inputs if d['maxInputChannels'] > 0]
        matches = [d for d in inputs if device_name.casefold() in d['name'].casefold()]
        if len(matches) != 1:
            available = ', '.join(f"{d['index']}: {d['name']}" for d in inputs) or '(none)'
            raise ValueError(
                f'Expected one input matching {device_name!r}, found {len(matches)}. '
                f'Available inputs: {available}. Check whether the microphone is busy '
                'in another application or on the Docker host.')
        info = matches[0]
    else:
        info = (audio.get_device_info_by_index(device_index) if device_index >= 0
                else audio.get_default_input_device_info())
    index = int(info['index'])
    max_channels = int(info['maxInputChannels'])
    if max_channels < 1:
        raise ValueError(f'Audio device {index} ({info["name"]}) has no input channels; '
                         'device indices can change. Select input_device_name instead.')
    rates = list(dict.fromkeys([16000, int(info['defaultSampleRate']), 48000, 44100]))
    channels = [1] + ([2] if max_channels >= 2 else [])
    for rate in rates:
        for count in channels:
            try:
                supported = audio.is_format_supported(
                    rate, input_device=index, input_channels=count,
                    input_format=pyaudio.paInt16)
            except ValueError:
                continue
            if not supported:
                continue
            # Do not hide busy/permission errors by repeatedly reopening hardware.
            stream = audio.open(
                format=pyaudio.paInt16, channels=count, rate=rate,
                input=True, input_device_index=index,
                frames_per_buffer=max(1, math.ceil(output_chunk * rate / 16000)))
            logger.info(f'Microphone {index}: {info["name"]}; '
                        f'{rate} Hz, {count} channel(s) -> 16000 Hz mono')
            return MicrophoneStream(stream, rate, count, output_chunk)
    raise ValueError(f'Audio device {index} ({info["name"]}) supports none of '
                     f'the tested PCM input formats: rates={rates}, channels={channels}')
