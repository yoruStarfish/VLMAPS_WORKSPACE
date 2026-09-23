from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'navigatethroughposes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic',
    maintainer_email='robotic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigatethroughposes = navigatethroughposes.navigatethroughposes:main',
            'my_navigatethroughposes = navigatethroughposes.my_navigatethroughposes:main',
            'navi_vlmaps = navigatethroughposes.navi_vlmaps:main',
            'llm_cmd = navigatethroughposes.llm_cmd:main',
            'my_navigatethroughposes_11F = navigatethroughposes.my_navigatethroughposes_11F:main',
            'debug_goTo_dock = navigatethroughposes.debug_goTo_dock:main',
            'snapshot_server = navigatethroughposes.snapshot_server:main',
            'path_cmdvel = navigatethroughposes.path_cmdvel:main',
            'path = navigatethroughposes.path:main',
        ],
    },
)
