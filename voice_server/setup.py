from glob import glob
from setuptools import find_packages, setup

package_name = 'voice_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'environment.yml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'PyAudio', 'faster-whisper==1.2.1', 'av>=11,<18'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='cooa6643@gmail.com',
    description='Microphone prompts for ROS 2 language navigation',
    license='TODO: License declaration',
    extras_require={
        'wakeword': ['openwakeword', 'onnxruntime'],
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_server = voice_server.voice_server:main',
        ],
    },
)
