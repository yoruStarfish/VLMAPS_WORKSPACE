"""Start voice input in its own Conda environment, retaining ROS discovery."""

import os
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _start_voice(context):
    conda = LaunchConfiguration('conda_executable').perform(context)
    environment = LaunchConfiguration('conda_env').perform(context)
    if not shutil.which(conda):
        raise RuntimeError('Conda not found; set conda_executable:=/path/to/conda')
    parameters = (
        'command_topic', 'input_device_index', 'input_device_name', 'language_code',
        'max_record_seconds', 'silence_seconds', 'silence_threshold', 'speech_start_timeout',
        'model_path', 'device', 'compute_type', 'cpu_threads',
        'wakeword_enabled', 'wakeword_model_path',
        'wakeword_threshold',
    )
    command = [conda, 'run', '--no-capture-output', '-n', environment,
               'python', '-m', 'voice_server.voice_server', '--ros-args']
    for name in parameters:
        value = LaunchConfiguration(name).perform(context)
        if value:  # Empty model path uses the node default.
            command.extend(['-p', f'{name}:={value}'])
    return [ExecuteProcess(cmd=command, output='screen',
                           additional_env={'PYTHONUNBUFFERED': '1',
                                           'PYTHONNOUSERSITE': '1'},
                           sigterm_timeout='40', sigkill_timeout='5')]


def generate_launch_description():
    """Expose Conda selection and the voice node's startup parameters."""
    defaults = {
        'conda_executable': os.environ.get('CONDA_EXE') or shutil.which('conda') or 'conda',
        'conda_env': 'voice_server',
        'command_topic': '/user_command',
        'input_device_index': '-1',
        'input_device_name': 'pulse',
        'language_code': 'en',
        'model_path': '/workspace/models/faster-whisper-small.en',
        'device': 'cpu',
        'compute_type': 'int8',
        'cpu_threads': '4',
        'max_record_seconds': '15.0',
        'silence_seconds': '2.5',
        'speech_start_timeout': '8.0',
        'silence_threshold': '100.0',
        'wakeword_enabled': 'false',
        'wakeword_model_path': '',
        'wakeword_threshold': '0.5',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        OpaqueFunction(function=_start_voice),
    ])
